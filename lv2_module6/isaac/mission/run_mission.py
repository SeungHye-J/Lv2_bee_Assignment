#!/usr/bin/env python3
"""Isaac 전체 미션 한 판 — 신호등 · 시작 바코드는 읽었다고 치고 mission_manager에 start를 보내 끝까지 지켜본다.

  python3 isaac/mission/run_mission.py                          # R5 Y3 G4 · P2-01 (scene_sim 퍽으로 되는 조합)
  python3 isaac/mission/run_mission.py --seq RGRRGRGGYGYG --code P2-02
  python3 isaac/mission/run_mission.py --clear-basket           # 적재함 장부가 남아 시작이 거부될 때 먼저 비움
  python3 isaac/mission/run_mission.py --list                   # 장면 퍽으로 풀리는 R·Y·G 횟수 조합 목록
  python3 isaac/mission/run_mission.py --counts R4Y4G4 --order desc   # 횟수로 신호 (순서는 섞음) · desc = P2-02 내림차순 · asc = P2-01 오름차순
  python3 isaac/mission/run_mission.py --seed 7 --order asc     # 풀리는 조합 중 랜덤 신호 (같은 seed = 같은 신호)
  python3 isaac/mission/run_mission.py --counts R6Y2G4 --from-survey   # 관측 없이 start:FROM_SURVEY:<바코드>:<횟수> (신호등 점멸과 안 겹침)

환경: ros2_ws/install/setup.bash · ROS_DOMAIN_ID가 Isaac 쪽과 같아야 한다 (~/.bashrc). Fast DDS 프로파일은 터미널 값 (없으면 fastdds_local_only.xml).
흐름 (mission_manager): GO_OBSERVE → BARCODE → READ → SURVEY → BUILD → VERIFY → RETURN → DONE
  BARCODE 단계 동안 /mission/qr, READ 단계 동안 /mission/signal_status(ok=true)를 1초마다 낸다 — 단계가 시작된 뒤에 받은 값만 쓰기 때문.
  나머지(이동 · 인지 · 집기 · 놓기)는 실기와 같은 노드가 그대로 한다.
신호 조건 (mission_solver allocate): R ≤ 빨간 퍽 수 · G ≤ 초록 퍽 수 · 남는 퍽(Y)이 한 모양 — scene_sim은 빨강 7(box 4 · cyl 3) · 초록 5(box 2 · cyl 3)
  예: R5 Y3 G4 됨 · 공개 연습 Q1(R4 Y2 G6)은 초록이 모자라 no_allocation
Ctrl+C = /cmd/mission abort. 끝나면 DONE 0 · ERROR 1로 나간다. 회차 기록: ~/.ros/mission/<시각>/
"""
import argparse
import os
import random
import re
import sys
from collections import Counter

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from perception.msg import MissionSignal
from plan.msg import MissionState

NAMES = {'R': 'red', 'Y': 'yellow', 'G': 'green'}
IDLE = ('IDLE', 'DONE', 'ERROR')   # start를 받는 단계 (DONE · ERROR 뒤에도 phase는 그대로 남음)
ORDER = {'asc': 'P2-01', 'desc': 'P2-02'}   # 발제: P2-01 · P2-03 오름차순 · P2-02 · P2-04 내림차순
SCENE = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'usda', 'scene_sim.usda')


def scene_pucks(path):
    """장면 파일의 퍽 prim 이름(puck_<box|cyl>_<red|green>_<n>)으로 (색, 모양) 개수"""
    return Counter((c, s) for s, c in re.findall(r'def "puck_(box|cyl)_(red|green)_\d+"', open(path).read()))


def solvable(r, y, g, n):
    """mission_solver allocate와 같은 규칙: R = 빨강만 · G = 초록만 · 남는 퍽(Y)은 한 모양"""
    yr, yg = n['red', 'box'] + n['red', 'cyl'] - r, n['green', 'box'] + n['green', 'cyl'] - g
    return yr >= 0 and yg >= 0 and yr + yg == y and any(n['red', s] >= yr and n['green', s] >= yg for s in ('box', 'cyl'))


def combos(n):
    return [(r, y, 12 - r - y) for r in range(1, 7) for y in range(1, 7) if 1 <= 12 - r - y <= 6 and solvable(r, y, 12 - r - y, n)]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--seq', default='RGYRGRYGRGYR', help='신호등 12회 색 (R/Y/G) — 기본 R5 Y3 G4')
    ap.add_argument('--code', default='P2-01', help='시작 바코드 — P2-01 · P2-03 오름차순 · P2-02 · P2-04 내림차순')
    ap.add_argument('--order', choices=ORDER, help='asc = P2-01 오름차순 · desc = P2-02 내림차순 (--code 대신)')
    ap.add_argument('--counts', help='신호 횟수 예: R4Y4G4 — 순서는 --seed로 섞음 (--seq 대신)')
    ap.add_argument('--seed', type=int, help='--counts가 없으면 장면 퍽으로 풀리는 조합 중 랜덤 · 있으면 순서만 섞음')
    ap.add_argument('--from-survey', action='store_true', help='관측 단계 없이 start:FROM_SURVEY:<바코드>:<횟수> 한 번만 보냄')
    ap.add_argument('--scene', default=SCENE, help='퍽 개수를 읽을 장면 (기본 usda/scene_sim.usda)')
    ap.add_argument('--list', action='store_true', help='장면 퍽으로 풀리는 횟수 조합만 찍고 끝')
    ap.add_argument('--clear-basket', action='store_true', help='시작 전 /coordinator/basket_clear')
    a = ap.parse_args()
    pucks = scene_pucks(a.scene)
    ok = combos(pucks)
    if a.list:
        print(f'퍽 {sum(pucks.values())}개 {dict(pucks)} — 풀리는 조합 {len(ok)}개:')
        print('  ' + '  '.join(f'R{r}Y{y}G{g}' for r, y, g in ok))
        return
    rng = random.Random(a.seed)
    if a.counts:
        m = re.fullmatch(r'R(\d)Y(\d)G(\d)', a.counts.upper())
        if not m:
            sys.exit(f'--counts {a.counts}: R<n>Y<n>G<n> 꼴 (예: R4Y4G4)')
        seq = list('R' * int(m[1]) + 'Y' * int(m[2]) + 'G' * int(m[3]))
        rng.shuffle(seq)
        seq = ''.join(seq)
    elif a.seed is not None:
        r, y, g = rng.choice(ok)
        seq = list('R' * r + 'Y' * y + 'G' * g)
        rng.shuffle(seq)
        seq = ''.join(seq)
    else:
        seq = a.seq.upper()
    code = ORDER[a.order] if a.order else a.code
    if len(seq) != 12 or set(seq) - set(NAMES) or not all(1 <= seq.count(c) <= 6 for c in NAMES):
        sys.exit(f'신호 {seq}: R/Y/G 12개 · 색마다 1~6회여야 함')
    counts = f'R{seq.count("R")}Y{seq.count("Y")}G{seq.count("G")}'
    if not solvable(seq.count('R'), seq.count('Y'), seq.count('G'), pucks):
        sys.exit(f'신호 {counts}: 장면 퍽 {dict(pucks)}으로 안 풀림 (no_allocation) — --list로 되는 조합 확인')
    start = f'start:FROM_SURVEY:{code}:{counts}' if a.from_survey else 'start'

    # Isaac 쪽(isaac_sim.sh)과 같은 Fast DDS 프로파일 — 터미널 값을 그대로, 없으면 isaac_sim.sh 기본(local_only)
    os.environ.setdefault('FASTDDS_DEFAULT_PROFILES_FILE', os.path.join(os.path.dirname(SCENE), '..', '..', 'ros2_ws', 'fastdds_local_only.xml'))
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'SUBNET'
    rclpy.init()
    n = rclpy.create_node('run_mission', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
    log = n.get_logger()
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    cmd = n.create_publisher(String, '/cmd/mission', 10)
    qr = n.create_publisher(String, '/mission/qr', latched)
    sig = n.create_publisher(MissionSignal, '/mission/signal_status', 10)
    st = {'phase': None, 'last': None, 'started': False, 'running': False, 'exit': None}

    if a.clear_basket:
        c = n.create_client(Trigger, '/coordinator/basket_clear')
        if not c.wait_for_service(timeout_sec=20.0):
            sys.exit('/coordinator/basket_clear 안 보임 — coordinator가 떠 있는지 확인')
        f = c.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(n, f, timeout_sec=10.0)
        log.info(f'적재함 장부: {f.result().message if f.result() else "응답 없음"}')

    def on_state(m):
        st['phase'] = m.phase
        line = (m.phase, m.note, m.error, m.placed, tuple(m.results))
        if line != st['last']:
            st['last'] = line
            res = f' · 결과 {list(m.results)[-1]}' if m.results else ''
            log.info(f'[{m.phase}] {m.note} · 놓음 {m.placed}/{m.total}{res}' + (f' · 오류 {m.error}' if m.error else ''))
        if st['started'] and m.phase not in IDLE:
            st['running'] = True
        if st['running'] and m.phase in ('DONE', 'ERROR'):
            st['exit'] = 0 if m.phase == 'DONE' else 1
        if st['started'] and '시작 거부' in m.note:   # 적재함 장부가 남음 — --clear-basket
            st['exit'] = 1

    n.create_subscription(MissionState, '/mission/state', on_state, latched)

    def tick():
        if not st['started']:
            if st['phase'] in IDLE and cmd.get_subscription_count() > 0:
                cmd.publish(String(data=start))
                st['started'] = True
                log.info(f'{start} · 신호 {" ".join(seq)} ({counts}) · 바코드 {code}')
            elif st['phase'] not in (None, *IDLE):
                st['exit'] = 1
                log.error(f'미션이 이미 {st["phase"]} — 끝나거나 abort한 뒤에 다시')
            return
        if a.from_survey:
            return
        if st['phase'] == 'BARCODE':
            qr.publish(String(data=code))
        elif st['phase'] == 'READ':
            m = MissionSignal(sequence=[NAMES[c] for c in seq], red=seq.count('R'), yellow=seq.count('Y'),
                              green=seq.count('G'), ok=True, cycles=2, raw=' '.join(seq))
            m.header.stamp = n.get_clock().now().to_msg()
            sig.publish(m)

    n.create_timer(1.0, tick)
    log.info('/mission/state 기다리는 중 (mission_manager)')
    try:
        while rclpy.ok() and st['exit'] is None:
            rclpy.spin_once(n, timeout_sec=0.2)
    except KeyboardInterrupt:
        if st['started']:
            cmd.publish(String(data='abort'))
            rclpy.spin_once(n, timeout_sec=0.5)
            print('abort 보냄')
        st['exit'] = 1
    sys.exit(st['exit'])


if __name__ == '__main__':
    main()
