"""가르친 경로 녹화 (teach & repeat, 11-15) — 수동 조작으로 모는 동안 로봇 자리(map→base_footprint)를 5 cm마다 남겨
arena 좌표로 ~/.ros/routes/<이름>.txt에 저장. AUTO에서 skill_navigate가 시작이 로봇 근처 · 끝이 목표 근처인 경로를
경유점으로 씀 (거꾸로도 — 복귀). 끝 = 다음 navigate 목표 자리(상자 앞 다가설 자리)에서 상자를 보고 멈춘 뒤 Enter."""
from __future__ import annotations

import math
import select
import sys
import time
from pathlib import Path

ROUTES = Path.home() / '.ros' / 'routes'
START = (0.734, 0.950, math.pi)   # plan.yaml · skills.yaml start_pose_arena (map 원점 = 켠 자리)
STEP_M = 0.05
PLAN_YAML = Path(__file__).resolve().parents[3] / 'ros2_ws' / 'src' / 'plan' / 'config' / 'plan.yaml'


def leg_labels(plan_yaml: Path = PLAN_YAML) -> list[str]:
    """미션 주행 순서대로 구간 이름 — 출발 → survey_order 상자 → build_location_order 장소 → 출발 (재검증은 그 자리, 11-15).
    plan.yaml을 못 읽으면 [] (번호만)."""
    try:
        import yaml
        p = yaml.safe_load(plan_yaml.read_text())['mission_manager']['ros__parameters']
        stops = ['start'] + [f'S{i}' for i in p['survey_order']] + [f'L{i}' for i in p['build_location_order']] + ['start']
    except Exception:
        return []
    return [f'{a}_{b}' for a, b in zip(stops, stops[1:])]


def to_arena(mx: float, my: float, start=START) -> tuple[float, float]:
    """map → arena (skill_navigate의 arena → map 역변환: map = R(−θ0)·(arena − p0))."""
    c, s = math.cos(start[2]), math.sin(start[2])
    return start[0] + c * mx - s * my, start[1] + s * mx + c * my


def _yaw(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def _simplify(p, tol):
    """꺾이는 점만 (Douglas-Peucker)."""
    if len(p) < 3:
        return list(p)
    a, b = p[0], p[-1]
    dx, dy = b[0] - a[0], b[1] - a[1]
    L = math.hypot(dx, dy) or 1e-9
    d = [abs(dx * (q[1] - a[1]) - dy * (q[0] - a[0])) / L for q in p]
    k = max(range(len(p)), key=d.__getitem__)
    return _simplify(p[:k + 1], tol)[:-1] + _simplify(p[k:], tol) if d[k] > tol else [a, b]


def motions(odom, tol=0.08, min_turn_deg=5.0):
    """odom 자세 [(x, y, yaw)] → 움직임 정답표 [('turn', deg) | ('forward', m)] — 왼쪽 회전 +.
    처음 방향(녹화 시작 yaw)에서 첫 직진 방향으로 돌기 · 꺾일 때마다 돌기 · 끝 방향(마지막 yaw)으로 돌기."""
    out = []
    def turn(a, b):
        t = math.degrees((b - a + math.pi) % (2 * math.pi) - math.pi)
        if abs(t) >= min_turn_deg:
            out.append(('turn', round(t, 1)))
    pts = _simplify([(x, y) for x, y, _ in odom], tol)
    head = odom[0][2]
    for a, b in zip(pts, pts[1:]):
        d = math.hypot(b[0] - a[0], b[1] - a[1])
        if d < 0.05:
            continue
        h = math.atan2(b[1] - a[1], b[0] - a[0])
        turn(head, h)
        out.append(('forward', round(d, 2)))
        head = h
    turn(head, odom[-1][2])
    return out


def _save_motion(out: Path, name: str, odom) -> None:
    rows = motions(odom)
    out.write_text(f'# {name} · {time.strftime("%Y-%m-%d %H:%M")} · 바퀴+IMU odom 기준 움직임 정답표 · turn = 도 (왼쪽 +) · forward = m\n'
                   'action,value\n' + ''.join(f'{a},{v}\n' for a, v in rows))
    print(f'  움직임 표 {out.name}: ' + ' → '.join(f'{"직진" if a == "forward" else "회전"} {v:+g}{"m" if a == "forward" else "°"}'
                                                for a, v in rows))


def _save(out: Path, name: str, pts) -> None:
    length = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))
    out.write_text(f'# {name} · {time.strftime("%Y-%m-%d %H:%M")} · {len(pts)}점 · {length:.2f} m · arena x y\n'
                   + ''.join(f'{x:.3f} {y:.3f}\n' for x, y in pts))
    print(f'\n저장: {out.name} ({len(pts)}점 · {length:.2f} m) — 시작 ({pts[0][0]:.2f}, {pts[0][1]:.2f}) → 끝 ({pts[-1][0]:.2f}, {pts[-1][1]:.2f})')


def record(node, tf_buf, name: str, start=START, first: int = 1) -> int:
    """한 번에 쭉 몰면서 구간마다 Enter — <이름>_01.txt, _02.txt …로 나눠 저장 (끝 점 = 다음 구간 시작).
    q + Enter · Ctrl+C = 지금 구간까지 저장하고 끝."""
    import rclpy
    from rclpy.time import Time
    from geometry_msgs.msg import TwistStamped
    ROUTES.mkdir(parents=True, exist_ok=True)
    seg, pts, odom, saved = max(1, first), [], [], 0   # first = 이어서 할 때 시작 구간 번호 (--from)
    labels = leg_labels()
    cmds: list[tuple[float, float, float]] = []   # (t, v, w) — 사람이 준 조작 명령 (값이 바뀔 때만)
    t0 = [time.monotonic()]

    def on_cmd(m):
        v, w = round(m.twist.linear.x, 3), round(m.twist.angular.z, 3)
        if not cmds or cmds[-1][1:] != (v, w):
            cmds.append((round(time.monotonic() - t0[0], 2), v, w))
    sub = node.create_subscription(TwistStamped, '/cmd/manual/cmd_vel', on_cmd, 50)

    def stem(k: int) -> str:
        return f'{name}_{k:02d}' + (f'_{labels[k - 1]}' if k - 1 < len(labels) else '')

    def save_seg() -> bool:
        """지금 구간 저장 (0.3 m 넘게 움직였으면) — Enter · q · Ctrl+C 모두 (11-15: 마지막 복귀를 Ctrl+C로 끝내 메모리에서 사라짐)."""
        nonlocal seg, pts, odom, saved
        moved = len(odom) >= 2 and math.hypot(odom[-1][0] - odom[0][0], odom[-1][1] - odom[0][1]) >= 0.3
        if not (moved or (len(pts) >= 2 and math.hypot(pts[-1][0] - pts[0][0], pts[-1][1] - pts[0][1]) >= 0.3)):
            return False
        st = stem(seg)
        if len(pts) >= 2:
            _save(ROUTES / f'{st}.txt', f'{st}', pts)
        if len(odom) >= 2:
            _save_motion(ROUTES / f'{st}.motion.csv', st, odom)
        (ROUTES / f'{st}.cmd.csv').write_text(
            f'# {st} · 사람이 준 조작 명령 /cmd/manual/cmd_vel (값이 바뀔 때만) · t = 구간 시작부터 [s]\n'
            't,linear_mps,angular_radps\n' + ''.join(f'{t},{v},{w}\n' for t, v, w in cmds))
        saved += 1
        seg += 1
        pts, odom = pts[-1:], odom[-1:]   # 다음 구간은 이 자리에서 시작
        cmds.clear()
        t0[0] = time.monotonic()
        if seg - 1 < len(labels):
            print(f'  다음 구간 {seg}: {labels[seg - 1]}')
        return True
    if labels:
        print(f'구간 순서 (plan.yaml): ' + ' · '.join(f'{i}) {l}' for i, l in enumerate(labels, 1)))
    if labels and seg - 1 < len(labels):
        print(f'  이번 첫 구간 {seg}: {labels[seg - 1]}')
    print(f'경로 녹화: {name} — 수동으로 몰다가 구간 끝(상자 앞)에서 상자를 똑바로 보고 Enter = 그 구간 저장 · 이어서 다음 구간\n'
          f'  q + Enter 또는 Ctrl+C = 지금 구간까지 저장하고 끝 · 저장 폴더 {ROUTES}')
    try:
        while True:
            rclpy.spin_once(node, timeout_sec=0.1)
            try:
                tr = tf_buf.lookup_transform('map', 'base_footprint', Time()).transform.translation
                p = to_arena(tr.x, tr.y, start)
                if not pts or math.hypot(p[0] - pts[-1][0], p[1] - pts[-1][1]) >= STEP_M:
                    pts.append(p)
                    print(f'\r  구간 {seg} · 점 {len(pts)}개 · 지금 arena ({p[0]:.2f}, {p[1]:.2f})   ', end='', flush=True)
            except Exception:
                pass
            try:  # odom(바퀴+IMU)는 튀지 않음 — 움직임 정답표 (map 좌표가 흔들려도)
                o = tf_buf.lookup_transform('odom', 'base_footprint', Time()).transform
                q = (o.translation.x, o.translation.y, _yaw(o.rotation))
                if not odom or math.hypot(q[0] - odom[-1][0], q[1] - odom[-1][1]) >= STEP_M or abs(q[2] - odom[-1][2]) > 0.1:
                    odom.append(q)
            except Exception:
                pass
            if not select.select([sys.stdin], [], [], 0)[0]:
                continue
            quit_ = sys.stdin.readline().strip().lower() == 'q'
            if not save_seg() and not quit_:
                print('\n  구간이 0.3 m보다 짧음 — 저장 안 함 (계속 몰기)')
            if quit_:
                break
    except KeyboardInterrupt:
        print('\n  Ctrl+C — 지금 구간까지 저장하고 끝')
        save_seg()
    node.destroy_subscription(sub)
    print(f'끝 — 구간 {saved}개 저장 ({ROUTES}: .txt = map 좌표 · .motion.csv = 직진·회전 정답표 · .cmd.csv = 조작 명령)')
    return 0 if saved else 1


if __name__ == '__main__':   # 자체 확인: 출발 자리(map 원점) = arena 출발 · map +x(로봇 앞) = arena −x
    assert to_arena(0, 0) == START[:2]
    ax, ay = to_arena(1.0, 0.0)
    assert abs(ax - (START[0] - 1.0)) < 1e-9 and abs(ay - START[1]) < 1e-9
    ax, ay = to_arena(-2.202, -3.854)   # 11-15 실기 S3 목표(map) = 상자 3 (2.936, 5.354) −y 앞 0.55 m
    assert abs(ax - 2.936) < 1e-3 and abs(ay - 4.804) < 1e-3
    # 움직임 표: 동쪽을 보고 시작 → 1 m 직진 → 왼쪽 90° → 1 m 직진 → 끝에서 서쪽(180°)을 봄
    L = [(i * 0.05, 0.0, 0.0) for i in range(21)] + [(1.0, i * 0.05, math.pi / 2) for i in range(1, 21)]
    L[-1] = (1.0, 1.0, math.pi)
    m = motions(L)
    assert m == [('forward', 1.0), ('turn', 90.0), ('forward', 1.0), ('turn', 90.0)], m
    lb = leg_labels()
    assert not lb or (lb[0].startswith('start_S') and lb[-1].endswith('_start')), lb
    print('구간:', lb)
    print('ok routerec')
