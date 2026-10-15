"""SIL 시뮬레이터 (호스트에서 실행) — 컨테이너(ROS)와는 공유 폴더 lv2_module6/debug 의 파일로만 주고받는다 (호스트에 ROS 불필요)

  [지도: 미로 · 실제 로봇 · /odom · 경로 · 인지 물체 · 퍽 · 장애물 · 이동물체]  │ [pole_cam:    ROS 영상 │ 깊이 │ 점구름]
  [퍽 · 배경 · 카메라 송출]                                                  │ [gripper_cam: ROS 영상 │ 깊이 │ 점구름]
  [환경: 미끄럼 · 마찰 · 이동물체 추가 · 바닥 장애물 · 신호등 · 통신 끊기 · 팔 붙잡기]   │ [상태 · 진단(/diagnostics_agg)]
  [수동 조작 탭 · 비상정지]                                             │ [명령 입력 → /user_command]

SIL(Software-in-the-Loop): 실기 대신 이 시뮬레이터가 세상(코스·퍽·장애물·이동물체·카메라 장면·로봇 몸)을 만들고,
ROS 쪽은 실제 노드·컨트롤러가 그대로 돈다.
  ① 카메라: 로봇 자세로 두 카메라 시점 렌더(scene.py) → debug/input-live/ → fake_camera → RealSense 토픽
  ② 하드웨어(플랜트, plant.py): faker를 mock_hardware:=false로 띄우면 controller_manager가 가상 시리얼로 보내는
     C 명령(debug/serial-out)을 받아 바퀴·팔을 움직이고(미끄럼·마찰·충돌) S 상태(debug/serial-in)를 돌려준다.
     → 지도의 '실제 로봇'(채움)과 /odom(테두리)이 미끄럼만큼 어긋난다.  mock_hardware:=true(기본)면 /odom을 따라 그린다.
  ③ ROS → 화면: monitor_manager가 debug/topic/<토픽>/ 에 저장한 것을 표시
  ④ 화면 → ROS: input_manager가 debug/input/<토픽>/message 를 발행 (명령 · 수동 조작 · 비상정지)
- 지도: 퍽(정답, 채움 — 왼쪽 드래그로 이동) · 바닥 장애물(회색 상자) · 이동물체(파랑, 움직임) · 인지 물체(/tracked_objects, 보라 테두리)
- 수동 조작: 우측 [수동 조작] 탭 (manual_window.py) — 탭을 고르면 수동 모드, 나가면 바퀴 정지 + 자동 모드
             바퀴 버튼(앞·뒤·멈춤·좌/우 바퀴 앞·뒤·전체 속도 ▲▼) · 로봇 3D · 팔 3D(관절 마커 드래그)
             [비상정지] → /e_stop (twist_mux 잠금: 모든 바퀴 명령 차단)
- 배경: images/background/*.jpg (360° 파노라마, Poly Haven CC0 — PANORAMA_CREDITS.txt)

  컨테이너:  test-faker 0                        (mock 하드웨어 — 카메라·명령·수동 조작까지)
             test-faker 0 mock_hardware:=false   (이 시뮬레이터가 OpenCR 역할 — 미끄럼·충돌·통신 끊김까지)
  호스트:    cd lv2_module6/simulator
             python3 -m venv .venv && .venv/bin/pip install -r requirements.txt   # 최초 1회
             .venv/bin/python main.py
             .venv/bin/python main.py --snapshot out.png              # 창 없이 화면 한 장 (확인용)
"""
import argparse
import base64
import json
import math
import os
import random
import threading
import time
import tkinter as tk
from concurrent.futures import ThreadPoolExecutor
from tkinter import ttk

from pathlib import Path

import cv2
import numpy as np

import maze
import plant as plant_mod
import robot_view
import scene
from bridge import (DEBUG, INPUT_DIR, LIVE_DIR, SIM_DIR, TOPIC_DIR, current_joints, read_msg, send, send_arm, send_bool,
                    send_service_request,
                    send_twist, write_atomic, yaw_of)
from manual_window import ManualPanel

HERE = Path(__file__).resolve().parent
BG_DIR = HERE / 'images' / 'background'
FLOOR_DIR = HERE / 'images' / 'floor'
WALL_DIR = HERE / 'images' / 'wall'
CAMERAS = ('pole_cam', 'gripper_cam')
TILE = (256, 192)
ROBOT_VIEW = (320, 260)
MAP_SCALE = 0.7              # 지도 축소율 (낮은 해상도에서 잘림 방지)
STALE_S = 2.0
PX_PER_M = 110
MARGIN = 30
STATE_NAMES = {0: 'RUNNING', 1: 'SUCCEEDED', 2: 'FAILED'}
LEVEL = {0: ('OK', ''), 1: ('WARN', 'warn'), 2: ('ERROR', 'error'), 3: ('STALE', 'warn')}
QUICK_COMMANDS = ['빨간 퍽을 목표 구역으로 옮겨', '모든 퍽을 목표 구역으로 옮겨']
NO_BG = '(격자)'
NO_FLOOR = '(격자)'
NO_WALL = '(단색)'
GRIP_R = 0.03            # 그리퍼 집기 반경 [m] — 손끝 이 안에 퍽이 있고 닫히면 attach
GRIP_CLOSED = 0.003      # 이보다 오므라들면 집기 (URDF: 0=닫힘 · 0.0265=열림 — 닫기 버튼은 0.0)
GRIP_OPEN = 0.015        # 이보다 벌어지면 놓음 [m] (열기 버튼은 0.026)
GRIP_Z = 0.15            # (사용 안 함 — 아래 수직 창으로 대체, 호환용 유지)
VERT_ABOVE = 0.06        # 손끝이 물체 윗면보다 이만큼 위까지는 접촉으로 침
VERT_BELOW = 0.03        # 손끝이 물체 아랫면보다 이만큼 아래까지는 접촉으로 침
ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
# 적재함 (URDF tray_link과 같은 치수 — base_footprint 기준 [m], x 앞 · z 바닥)
TRAY = {'x0': -0.19, 'x1': -0.07, 'y': 0.10, 'surface_z': 0.150, 'wall_t': 0.005}
# 몸통 받침 (base_footprint 기준 — 트레이·데크가 우선, 나머지는 몸통 윗면)
BODY_TOP = {'x0': -0.197, 'x1': 0.069, 'y': 0.133, 'z': 0.145}
DECK_TOP = {'x0': -0.064, 'x1': 0.069, 'y': 0.133, 'z': 0.200}


def _support_at(lx, ly):
    """base 기준 받침 높이 — 트레이 > 데크 > 몸통, 없으면 None(바닥)"""
    if TRAY['x0'] <= lx <= TRAY['x1'] and abs(ly) <= TRAY['y']:
        return TRAY['surface_z']
    if DECK_TOP['x0'] <= lx <= DECK_TOP['x1'] and abs(ly) <= DECK_TOP['y']:
        return DECK_TOP['z']
    if BODY_TOP['x0'] <= lx <= BODY_TOP['x1'] and abs(ly) <= BODY_TOP['y']:
        return BODY_TOP['z']
    return None


def _to_base(px, py, bx, by, byaw):
    """world → base_footprint 2D"""
    dx, dy = px - bx, py - by
    c, s = math.cos(byaw), math.sin(byaw)
    return (c * dx + s * dy, -s * dx + c * dy)


def _to_world(lx, ly, bx, by, byaw):
    """base_footprint → world 2D"""
    c, s = math.cos(byaw), math.sin(byaw)
    return (bx + c * lx - s * ly, by + s * lx + c * ly)


def _tray_reach(p):
    if p['shape'] == 'cylinder':
        return p['size'][0]
    if p['shape'] == 'sphere':
        return p['size'][0]
    return max(p.get('sx', p['size'][0]), p.get('sy', p['size'][0])) / 2
PUSH_R = 0.04            # 팔이 물체를 미는 반경 [m] — 손끝이 닿고 팔이 움직이면 밀림 (베이스 이동분 제외)
PUSH_K = 1.0             # 미는 속도 = 손끝 속도 × K
SHOVE_MARGIN = 0.005     # 바퀴가 퍽을 밀 때 남기는 여유 [m]


# ---------- 시뮬레이션 상태
class Sim:
    def __init__(self, course, spawn=None):
        self.course = course
        self.spawn = spawn or course['spawn']
        self.walls = np.asarray(maze.wall_boxes(course), float)
        self.renderer = scene.Renderer(self.walls)
        self.cells = scene.maze_cells(course)
        self.kin = None
        self.body = None
        self.render_ts = {}          # 카메라 → 마지막 렌더 시각 (화면이 SIM 직결인지 판단)
        self.pool = ThreadPoolExecutor(max_workers=len(CAMERAS))
        self.lock = threading.RLock()  # grip_interact가 잠근 채 save_truth를 부름 (재진입)
        self.pucks = []
        self.props = []           # 바닥 장애물 · 이동물체
        self.renders = {}         # 카메라 → (bgr, depth, T_world_cam)
        self.streaming = True
        self.running = True
        self.plant = plant_mod.Plant(DEBUG / 'serial-in', DEBUG / 'serial-out', self.spawn, self.collide)
        self.randomize(6)

    # --- 물체
    def randomize(self, count):
        with self.lock:
            self.pucks = scene.random_pucks(count, self.cells, self.spawn, random.Random())
        self.save_truth()

    def add_obstacle(self):
        x, y, _, _ = self.robot_xy_yaw()
        with self.lock:
            avoid = [(x, y), tuple(self.spawn)] + [(p['x'], p['y']) for p in self.pucks + self.props]
            ob = scene.random_obstacle(self.cells, avoid)
            if ob:
                self.props.append(ob)
        self.save_truth()

    def clear_obstacles(self):
        with self.lock:
            self.props = []
        self.save_truth()

    def pop_prop(self, kind):
        """스택식 삭제 — 같은 종류 중 가장 나중에 추가된 것부터"""
        with self.lock:
            for i in range(len(self.props) - 1, -1, -1):
                if self.props[i].get('kind') == kind:
                    del self.props[i]
                    break
        self.save_truth()

    def spawn_mover(self):
        """미로 안을 돌아다니는 이동물체 — 빈 칸에서 시작해 칸 사이를 계속 이동 (장애물 지우기까지)"""
        with self.lock:
            cx, cy = random.choice(self.cells)
            p = scene.mover(cx, cy, 0.0)
            p['wander'] = True
            p['until'] = float('inf')
            self.props.append(p)
            self._retarget(p)

    def reset_robot(self):
        # 로봇 위치 초기화: 플랜트 리셋 + mock odom 리셋 + EKF 리셋 (map 원점 = spawn)
        self.plant.reset(self.spawn)
        self._base_prev = None
        send_service_request('diff_drive_controller/set_odometry', {'x': 0.0, 'y': 0.0, 'yaw': 0.0})
        send_service_request('reset', {})

    def teleport_robot(self, wx, wy):
        # 지도 드래그 이동: 플랜트는 즉시, mock은 odom 리셋 서비스로
        _, _, yaw, _ = self.robot_xy_yaw()
        if self.plant.alive:
            self.plant.reset((wx, wy), yaw)
            self._base_prev = None
        else:
            send_service_request('diff_drive_controller/set_odometry',
                                 {'x': round(wx - self.spawn[0], 3),
                                  'y': round(wy - self.spawn[1], 3), 'yaw': round(yaw, 3)})

    def spawn_traffic(self):
        """신호등 랜덤 배치 — 빈 칸, 로봇·퍽·기존 물체와 0.4m 이상"""
        x, y, _, _ = self.robot_xy_yaw()
        with self.lock:
            avoid = [(x, y), tuple(self.spawn)] + [(p['x'], p['y']) for p in self.pucks + self.props]
            for _ in range(100):
                cx, cy = random.choice(self.cells)
                tx, ty = cx + random.uniform(-0.2, 0.2), cy + random.uniform(-0.2, 0.2)
                if all(math.dist((tx, ty), a) > 0.4 for a in avoid):
                    self._tl_n = getattr(self, '_tl_n', 0) + 1
                    self.props.append(scene.traffic_light(tx, ty, f'T{self._tl_n}'))
                    break
        self.save_truth()

    def update_traffic(self, dt):
        """신호등 순환 green → (꺼짐) → yellow → (꺼짐) → red → (꺼짐) …"""
        with self.lock:
            changed = False
            for p in self.props:
                if p.get('kind') != 'traffic_light':
                    continue
                p['timer'] -= dt
                if p['timer'] <= 0:
                    idx = (p.get('cycle_idx', 5) + 1) % len(scene.TL_CYCLE)
                    nxt, dur = scene.TL_CYCLE[idx]
                    p['state'] = nxt
                    p['cycle_idx'] = idx
                    p['timer'] = dur
                    p['color'] = scene.TL_COLORS[nxt]
                    changed = True
            if changed:
                self.save_truth()

    def _retarget(self, p):
        """0.5m 이상 떨어진 칸을 새 목표로 (벽에 막히면 move_people이 다시 고름)"""
        for _ in range(20):
            tx, ty = random.choice(self.cells)
            if math.dist((p['x'], p['y']), (tx, ty)) > 0.5:
                p['tx'], p['ty'] = tx, ty
                return
        p['tx'], p['ty'] = p['x'], p['y']

    def wall_hit(self, x, y, r):
        lo, hi = self.walls[:, :2] - self.walls[:, 3:5] / 2, self.walls[:, :2] + self.walls[:, 3:5] / 2
        nearest = np.clip([x, y], lo, hi)
        return bool((np.sum((nearest - [x, y]) ** 2, axis=1) < r * r).any())

    def move_people(self, dt):
        turn_max, speed = 2.0, 0.35  # 최대 회전 [rad/s] · 이동 속도 [m/s] — 급회전 없이 부드럽게
        with self.lock:
            for p in self.props:
                if p['kind'] != 'mover':
                    continue
                if p.get('wander'):
                    if math.dist((p['x'], p['y']), (p.get('tx', p['x']), p.get('ty', p['y']))) < 0.15:
                        self._retarget(p)
                    want = math.atan2(p['ty'] - p['y'], p['tx'] - p['x'])
                    got = p.get('heading', want)
                    diff = (want - got + math.pi) % (2 * math.pi) - math.pi
                    got += max(-turn_max * dt, min(turn_max * dt, diff))
                    p['heading'] = got
                    p['vx'], p['vy'] = math.cos(got) * speed, math.sin(got) * speed
                    nx, ny = p['x'] + p['vx'] * dt, p['y'] + p['vy'] * dt
                    if self.wall_hit(nx, ny, 0.2):
                        self._retarget(p)
                    else:
                        p['x'], p['y'] = nx, ny
                else:
                    p['x'] += p['vx'] * dt
                    p['y'] += p['vy'] * dt
                    p['until'] -= dt
            self.props = [p for p in self.props if p['kind'] != 'mover' or p.get('wander') or p['until'] > 0]

    def save_truth(self):
        """정답 물체 목록 (미로 좌표 + map 좌표) — 인지 결과와 비교용"""
        with self.lock:
            out = {'pucks': [dict(p, map_x=round(p['x'] - self.spawn[0], 3), map_y=round(p['y'] - self.spawn[1], 3))
                             for p in self.pucks],
                   'obstacles': [dict(p, color=list(p['color'])) for p in self.props if p['kind'] == 'obstacle'],
                   'traffic_lights': [{'id': p.get('id'), 'x': p['x'], 'y': p['y'], 'state': p['state'],
                                        'map_x': round(p['x'] - self.spawn[0], 3),
                                        'map_y': round(p['y'] - self.spawn[1], 3)}
                                       for p in self.props if p.get('kind') == 'traffic_light']}
        write_atomic(SIM_DIR / 'objects.json', lambda p: p.write_text(json.dumps(out, indent=1)))

    # --- 충돌 (플랜트가 부름): 벽·바닥장애물·이동물체는 막힘, 퍽은 바퀴에 밀려남
    def blocked(self, x, y, r):
        # 이동 차단 여부만 (퍽 밀기 없음) - 수동 조종 예측용
        if self.wall_hit(x, y, r):
            return True
        with self.lock:
            for p in self.props:
                reach = p['size'][0] if p['shape'] == 'cylinder' else max(p.get('sx', 0.1), p.get('sy', 0.1)) / 2
                if math.dist((x, y), (p['x'], p['y'])) < r + reach:
                    return True
        return False

    def collide(self, x, y, r):
        if self.blocked(x, y, r):
            return True
        with self.lock:
            for p in self.pucks:
                if p.get('held') or 'z' in p:
                    continue  # 든 것·로봇 위 적재는 바퀴 shove 대상 아님
                reach = p['size'][0]
                d = math.dist((x, y), (p['x'], p['y']))
                if d < r + reach:  # 퍽은 막지 않고 로봇 바깥으로 밀어냄
                    if d < 1e-9:
                        dx, dy = 1.0, 0.0
                    else:
                        dx, dy = (p['x'] - x) / d, (p['y'] - y) / d
                    nx, ny = x + dx * (r + reach + SHOVE_MARGIN), y + dy * (r + reach + SHOVE_MARGIN)
                    if self.wall_hit(nx, ny, reach):
                        return True
                    p['x'], p['y'] = round(nx, 3), round(ny, 3)
                    if time.time() - getattr(self, '_truth_t', 0) > 1.0:
                        self._truth_t = time.time()
                        self.save_truth()
        return False

    # --- 팔·물체 상호작용: 집기(attach) · 놓기(detach) · 밀기(push)
    def grip_interact(self, T_wb, q, dt):
        try:
            tip = (T_wb @ self.kin.pose('end_effector_link', q))[:3, 3]
        except KeyError:
            return
        tip = (float(tip[0]), float(tip[1]), float(tip[2]))
        # 밀기용 속도는 팔 관절 움직임분만 — 베이스가 움직여 손끝이 이동한 건 제외
        qprev = getattr(self, '_q_prev', None)
        self._q_prev = dict(q)
        if qprev and dt > 0:
            try:
                ptip = (T_wb @ self.kin.pose('end_effector_link', {**q, **{k: qprev.get(k, v) for k, v in q.items() if k in ARM_JOINTS}}))[:3, 3]
            except KeyError:
                ptip = None
            if ptip is not None:
                vx, vy, vz = (tip[0] - float(ptip[0])) / dt, (tip[1] - float(ptip[1])) / dt, (tip[2] - float(ptip[2])) / dt
            else:
                vx, vy, vz = 0.0, 0.0, 0.0
        else:
            vx, vy, vz = 0.0, 0.0, 0.0
        if math.sqrt(vx * vx + vy * vy + vz * vz) > 5.0:  # 텔레포트성 점프(리셋 등)는 무시
            vx, vy, vz = 0.0, 0.0, 0.0
        grip = q.get('gripper_left_joint', 0.0)
        with self.lock:
            held = next((p for p in self.pucks if p.get('held')), None)
            if held is not None:
                held['x'], held['y'] = round(tip[0], 3), round(tip[1], 3)
                h = held['size'][0] * 2 if held['shape'] == 'sphere' else held['size'][1]
                held['z'] = round(max(tip[2] - h / 2, 0.0), 3)  # 손끝 높이로 들어올림
                if grip > GRIP_OPEN:  # 벌리면 놓음 — z는 settle_cargo가 판단 (트레이 위면 적재, 아니면 바닥)
                    held.pop('held', None)
                    self.save_truth()
                return
            near = [p for p in self.pucks
                    if math.dist((tip[0], tip[1]), (p['x'], p['y'])) < GRIP_R + p['size'][0]
                    and tip[2] <= p.get('z', 0.0) + (p['size'][0] * 2 if p['shape'] == 'sphere' else p['size'][1]) + VERT_ABOVE
                    and tip[2] >= p.get('z', 0.0) - VERT_BELOW]
            if near and grip < GRIP_CLOSED:  # 닫히면 가장 가까운 퍽을 집음 (손끝 높이가 맞아야)
                pick = min(near, key=lambda p: math.dist((tip[0], tip[1]), (p['x'], p['y'])))
                pick['held'] = True
                pick['x'], pick['y'] = round(tip[0], 3), round(tip[1], 3)
                ph = pick['size'][0] * 2 if pick['shape'] == 'sphere' else pick['size'][1]
                pick['z'] = round(max(tip[2] - ph / 2, 0.0), 3)
                self.save_truth()
                return
            if math.sqrt(vx * vx + vy * vy + vz * vz) > 0.05:  # 손끝이 물체를 밀면 밀려남 (퍽·바닥장애물)
                moved = False
                for p in self.pucks + [o for o in self.props if o.get('kind') == 'obstacle']:
                    reach = p['size'][0] if p['shape'] == 'cylinder' else max(p.get('sx', 0.1), p.get('sy', 0.1)) / 2
                    ptop = p.get('z', 0.0) + (p['size'][0] * 2 if p['shape'] == 'sphere' else p['size'][1])
                    if math.dist((tip[0], tip[1]), (p['x'], p['y'])) >= PUSH_R + reach:
                        continue
                    if not (p.get('z', 0.0) - VERT_BELOW <= tip[2] <= ptop + VERT_ABOVE):
                        continue  # 손끝이 물체 높이와 안 맞으면 통과 (위에서 지나감)
                    nx, ny = p['x'] + vx * PUSH_K * dt, p['y'] + vy * PUSH_K * dt
                    if not self.wall_hit(nx, ny, reach):
                        p['x'], p['y'] = round(nx, 3), round(ny, 3)
                        moved = True
                if moved and time.time() - getattr(self, '_truth_t', 0) > 1.0:
                    self._truth_t = time.time()
                    self.save_truth()

    def carry_cargo(self, x, y, yaw):
        """로봇 위 퍽(z 있음, 든 것 제외)을 베이스와 함께 운반"""
        prev = getattr(self, '_base_prev', None)
        if prev is None:
            return
        px, py, pyaw = prev
        with self.lock:
            for p in self.pucks:
                if p.get('held') or 'z' not in p:
                    continue
                lx, ly = _to_base(p['x'], p['y'], px, py, pyaw)
                wx, wy = _to_world(lx, ly, x, y, yaw)
                p['x'], p['y'] = round(wx, 3), round(wy, 3)

    def settle_cargo(self, x, y, yaw):
        """로봇 위 퍽은 받침 높이로 · 트레이 안에선 가이드벽 안쪽으로 가둠, 받침 밖은 바닥으로"""
        with self.lock:
            for p in self.pucks:
                if p.get('held') or 'z' not in p:
                    continue
                lx, ly = _to_base(p['x'], p['y'], x, y, yaw)
                reach = _tray_reach(p)
                lo_x, hi_x = TRAY['x0'] + TRAY['wall_t'] + reach, TRAY['x1'] - TRAY['wall_t'] - reach
                lo_y, hi_y = -TRAY['y'] + TRAY['wall_t'] + reach, TRAY['y'] - TRAY['wall_t'] - reach
                if TRAY['x0'] <= lx <= TRAY['x1'] and abs(ly) <= TRAY['y']:
                    cx = min(max(lx, lo_x), hi_x)  # 가이드벽 가둠
                    cy = min(max(ly, lo_y), hi_y)
                    wx, wy = _to_world(cx, cy, x, y, yaw)
                    p['x'], p['y'], p['z'] = round(wx, 3), round(wy, 3), TRAY['surface_z']
                    continue
                s = _support_at(lx, ly)
                if s is None:
                    del p['z']  # 받침 밖 — 바닥으로
                else:
                    p['z'] = s

    # --- 로봇 자세: 플랜트가 돌고 있으면 실제 자세, 아니면 /odom (mock 하드웨어)
    def robot_xy_yaw(self):
        if self.plant.alive:
            return self.plant.pose()
        odom, _ = read_msg('odom')
        js, _ = read_msg('joint_states')
        q = {n: v for n, v in zip(js['name'], js['position']) if isinstance(v, float) and math.isfinite(v)} if js else {}
        if not odom:
            return self.spawn[0], self.spawn[1], 0.0, q
        pose = odom['pose']['pose']
        return pose['position']['x'] + self.spawn[0], pose['position']['y'] + self.spawn[1], yaw_of(pose['orientation']), q

    def step(self):
        if self.kin is None:
            desc, _ = read_msg('robot_description')
            if desc:
                self.kin = scene.Kinematics(desc['data'])
                self.body = scene.BodyModel(desc['data'])
        if self.kin is None:
            return False
        x, y, yaw, q = self.robot_xy_yaw()
        T_wb = scene.transform(scene.rpy_matrix(0, 0, yaw), (x, y, 0))
        now = time.time()
        self.carry_cargo(x, y, yaw)
        self.grip_interact(T_wb, q, min(now - getattr(self, '_step_t', now), 0.5) or 1e-3)
        self.settle_cargo(x, y, yaw)
        self._base_prev = (x, y, yaw)
        self._step_t = now
        with self.lock:
            things = [dict(p) for p in self.pucks + self.props]
        kin, body, renderer, twb = self.kin, self.body, self.renderer, T_wb

        def render_one(cam):
            T = twb @ kin.pose(f'{cam}_color_optical_frame', q)
            own = f'{cam}_link'
            boxes = [(twb @ kin.pose(link, q) @ To, sx, sy, sz, rgb)
                     for link, To, (sx, sy, sz), rgb in body.boxes if link != own]
            return cam, T, *renderer.render(T, things, boxes)

        for cam, T, bgr, depth in self.pool.map(render_one, CAMERAS):
            with self.lock:
                self.renders[cam] = (bgr, depth, T)
                self.render_ts[cam] = time.time()
            if self.streaming:
                write_atomic(LIVE_DIR / f'{cam}.png',
                             lambda p: cv2.imwrite(str(p), bgr, [cv2.IMWRITE_PNG_COMPRESSION, 1]) or None)
                write_atomic(LIVE_DIR / f'{cam}_depth.png',
                             lambda p: cv2.imwrite(str(p), depth, [cv2.IMWRITE_PNG_COMPRESSION, 1]) or None)
        return True

    def stop_streaming(self):
        for cam in CAMERAS:
            for f in (LIVE_DIR / f'{cam}.png', LIVE_DIR / f'{cam}_depth.png'):
                f.unlink(missing_ok=True)

    def render_loop(self):
        t = time.time()
        while self.running:
            now = time.time()
            self.move_people(now - t)
            self.update_traffic(now - t)
            t = now
            if not self.step():
                time.sleep(0.5)  # 로봇 URDF(/robot_description) 대기 — test-faker 0 실행 중인지
            else:
                time.sleep(0.01)  # GUI 스레드에 양보 (렌더율 영향 거의 없음)


# ---------- 지도
class MapView:
    def __init__(self, sim):
        self.sim = sim
        self._base = None  # (base 미로좌표, 수신시각) — /tf 캐시
        self.ppm = PX_PER_M * MAP_SCALE
        self.zoom = 1.0    # 보기 확대 (로봇 위치가 아니라 지도 보기만)
        self.cx, self.cy = 0, 0  # 확대 중심 (전체 지도 px) — viewport()에서 화면 크기로 초기화
        w, h = sim.course['size']
        self.half = (w / 2 + 0.2, h / 2 + 0.2)
        self.size = (int(2 * self.half[0] * self.ppm) + 2 * MARGIN, int(2 * self.half[1] * self.ppm) + 2 * MARGIN)
        self.walls = maze.walls_mask(sim.course, self.px_array, (self.size[1], self.size[0]))

    def px_array(self, xy):
        return np.stack([MARGIN + (xy[..., 0] + self.half[0]) * self.ppm,
                         MARGIN + (self.half[1] - xy[..., 1]) * self.ppm], -1).astype(np.int32)

    def px(self, x, y):
        return (int(MARGIN + (x + self.half[0]) * self.ppm), int(MARGIN + (self.half[1] - y) * self.ppm))

    def world(self, u, v):
        u, v = self._to_full(u, v)
        return ((u - MARGIN) / self.ppm - self.half[0], self.half[1] - (v - MARGIN) / self.ppm)

    def _roi(self):
        """확대 표시 영역 (전체 지도 px: x0, y0, w, h)"""
        h, w = self.size[1], self.size[0]
        if self.cx == 0 and self.cy == 0:
            self.cx, self.cy = w / 2, h / 2
        rw, rh = w / self.zoom, h / self.zoom
        return (min(max(self.cx - rw / 2, 0), w - rw), min(max(self.cy - rh / 2, 0), h - rh), rw, rh)

    def _to_full(self, u, v):
        """라벨 px → 전체 지도 px (확대 역변환)"""
        if self.zoom <= 1.0:
            return u, v
        x0, y0, _, _ = self._roi()
        return x0 + u / self.zoom, y0 + v / self.zoom

    def viewport(self, img):
        """전체 지도 → 화면 표시 (확대 잘라내기)"""
        if self.zoom <= 1.0:
            return img
        h, w = img.shape[:2]
        x0, y0, rw, rh = self._roi()
        roi = img[int(y0):int(y0 + rh), int(x0):int(x0 + rw)]
        return cv2.resize(roi, (w, h), interpolation=cv2.INTER_LINEAR)

    def reset_view(self):
        h, w = self.size[1], self.size[0]
        self.zoom, self.cx, self.cy = 1.0, w / 2, h / 2

    def from_map(self, x, y):
        return x + self.sim.spawn[0], y + self.sim.spawn[1]

    def base_from_tf(self):
        """/tf에서 odom→base_footprint를 찾아 미로 좌표 base 위치. 없으면 None.
        /odom 테두리와 같은 규약 (map 원점 = spawn, map→odom 보정은 무시).
        /tf 메시지는 건당 1개 transform이라 마지막을 캐시해 깜빡임을 막는다 (2초 만료)."""
        tf, _ = read_msg('tf')
        if tf:
            ob = next((t for t in tf.get('transforms', [])
                       if t.get('child_frame_id') == 'base_footprint'), None)
            if ob:
                tr = ob['transform']['translation']
                self._base = ((tr['x'] + self.sim.spawn[0], tr['y'] + self.sim.spawn[1],
                               yaw_of(ob['transform']['rotation'])), time.time())
        if self._base and time.time() - self._base[1] < STALE_S:
            return self._base[0]
        return None

    def dashed(self, img, a, b, color, dash=8, gap=6):
        ax, ay = self.px(*a)
        bx, by = self.px(*b)
        length = math.hypot(bx - ax, by - ay)
        if length < 4:
            return
        ux, uy = (bx - ax) / length, (by - ay) / length
        s = 0.0
        while s < length:
            e = min(s + dash, length)
            cv2.line(img, (int(ax + ux * s), int(ay + uy * s)), (int(ax + ux * e), int(ay + uy * e)), color, 1)
            s = e + gap

    def robot(self, img, x, y, yaw, color, filled):
        c = self.px(x, y)
        r = int(plant_mod.ROBOT_R * self.ppm)
        if filled:
            cv2.circle(img, c, r, color, -1)
            cv2.circle(img, c, r, (30, 30, 30), 1)
        else:
            cv2.circle(img, c, r, color, 2)
        cv2.line(img, c, self.px(x + 0.25 * math.cos(yaw), y + 0.25 * math.sin(yaw)), (30, 30, 30) if filled else color, 2)

    def render(self):
        img = np.full((self.size[1], self.size[0], 3), 245, np.uint8)
        for g in np.arange(-math.floor(self.half[0]), self.half[0], 0.5):  # 0.5m 격자
            cv2.line(img, self.px(g, -self.half[1]), self.px(g, self.half[1]), (225, 225, 225), 1)
        for g in np.arange(-math.floor(self.half[1]), self.half[1], 0.5):
            cv2.line(img, self.px(-self.half[0], g), self.px(self.half[0], g), (225, 225, 225), 1)
        img[self.walls > 0] = (90, 80, 70)
        sx, sy = self.px(*self.sim.spawn)
        cv2.drawMarker(img, (sx, sy), (0, 160, 0), cv2.MARKER_CROSS, 18, 2)
        cv2.putText(img, 'spawn (map 0,0)', (sx + 8, sy - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 140, 0), 1)
        # 적재함 외곽 (베이스 기준 → 월드)
        if self.sim.plant.alive:
            bx, by, byaw, _ = self.sim.plant.pose()
        else:
            odom0, _ = read_msg('odom')
            if odom0:
                op = odom0['pose']['pose']
                bx, by = op['position']['x'] + self.sim.spawn[0], op['position']['y'] + self.sim.spawn[1]
                byaw = yaw_of(op['orientation'])
            else:
                bx, by, byaw = None, None, None
        if bx is not None:
            corners = [(TRAY['x0'], -TRAY['y']), (TRAY['x1'], -TRAY['y']),
                       (TRAY['x1'], TRAY['y']), (TRAY['x0'], TRAY['y'])]
            tpts = np.array([self.px(*_to_world(lx, ly, bx, by, byaw)) for lx, ly in corners], np.int32)
            cv2.polylines(img, [tpts], True, (60, 60, 60), 1)

        with self.sim.lock:
            things = [dict(p) for p in self.sim.pucks + self.sim.props]
        for p in things:
            color = scene.PALETTE.get(p['color'], (128, 128, 128)) if isinstance(p['color'], str) else tuple(p['color'])
            c = self.px(p['x'], p['y'])
            if p['shape'] == 'box':
                pts = np.array([self.px(*q) for q in _rect(p['x'], p['y'], p['yaw'], p.get('sx', p['size'][0]),
                                                           p.get('sy', p['size'][0]))], np.int32)
                cv2.fillPoly(img, [pts], color)
                cv2.polylines(img, [pts], True, (30, 30, 30), 1)
            else:
                r = max(4, int(p['size'][0] * self.ppm))
                cv2.circle(img, c, r, color, -1)
                cv2.circle(img, c, r, (30, 30, 30), 1)
                if p['shape'] == 'sphere':
                    cv2.circle(img, (c[0] - r // 3, c[1] - r // 3), max(1, r // 3), (255, 255, 255), -1)
            if 'id' in p:
                cv2.putText(img, f"#{p['id']}", (c[0] + 8, c[1] + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (60, 60, 60), 1)
            if p.get('kind') == 'traffic_light':  # 신호등 — 상태색 링 + 상태 글자
                cv2.circle(img, c, max(8, int(0.09 * self.ppm)), (255, 255, 255), 2)
                cv2.circle(img, c, max(8, int(0.09 * self.ppm)), color, 1)
                cv2.putText(img, scene.TL_STATE_KO.get(p.get('state', ''), ''), (c[0] + 8, c[1] + 14),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (60, 60, 60), 1)
            if p.get('held'):  # 든 퍽 — 흰 테두리 + 표시
                cv2.circle(img, c, max(6, int(p['size'][0] * self.ppm) + 3), (255, 255, 255), 2)
                cv2.putText(img, 'held', (c[0] + 8, c[1] + 14), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (60, 60, 60), 1)

        objects, _ = read_msg('tracked_objects')
        for o in (objects or {}).get('objects', []):  # 인지 결과 (보라 테두리)
            pos = o['pose']['pose']['position']
            x, y = self.from_map(pos['x'], pos['y'])
            hx, hy = max(o['size']['x'], 0.06) / 2, max(o['size']['y'], 0.06) / 2
            cv2.rectangle(img, self.px(x - hx, y + hy), self.px(x + hx, y - hy), (150, 0, 150), 2)
            cv2.putText(img, f"{o['class_name']}#{o['id']}", self.px(x + hx + 0.03, y + hy),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150, 0, 150), 1)

        path, _ = read_msg('plan')
        pts = [self.px(*self.from_map(ps['pose']['position']['x'], ps['pose']['position']['y']))
               for ps in (path or {}).get('poses', [])]
        if len(pts) > 1:
            cv2.polylines(img, [np.array(pts, np.int32)], False, (220, 120, 0), 2)
            cv2.circle(img, pts[-1], 6, (220, 120, 0), 2)

        for cam, (_, _, T) in list(self.sim.renders.items()):  # 카메라 시야
            o, f = T[:3, 3], T[:3, 2]
            for side in (-1, 1):
                a = math.atan2(f[1], f[0]) + side * math.radians(scene.HFOV_DEG) / 2
                cv2.line(img, self.px(o[0], o[1]), self.px(o[0] + 0.6 * math.cos(a), o[1] + 0.6 * math.sin(a)),
                         (0, 170, 255) if cam == 'pole_cam' else (255, 0, 170), 1)

        odom, age = read_msg('odom')
        if self.sim.plant.alive:  # 실제 로봇(채움) + /odom(테두리) — 미끄럼만큼 어긋남
            x, y, yaw, _ = self.sim.plant.pose()
            self.robot(img, x, y, yaw, (60, 180, 240) if not self.sim.plant.bumped else (40, 40, 220), True)
            c = self.px(x, y)
            cv2.putText(img, 'real', (c[0] + 22, c[1] - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (60, 60, 60), 1)
        ox, oy = None, None
        if odom:
            pose = odom['pose']['pose']
            ox, oy = self.from_map(pose['position']['x'], pose['position']['y'])
            self.robot(img, ox, oy, yaw_of(pose['orientation']), (0, 0, 200) if age > STALE_S else (200, 60, 0), False)
            c = self.px(ox, oy)
            cv2.putText(img, 'odom', (c[0] + 22, c[1] + 14), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (60, 60, 60), 1)
        base = self.base_from_tf()
        if base is not None and ox is not None:  # base_footprint(초록점) + /odom까지 점선·거리
            bx, by, byaw = base
            self.dashed(img, (ox, oy), (bx, by), (60, 60, 60))
            c = self.px(bx, by)
            cv2.circle(img, c, 6, (0, 200, 0), -1)
            cv2.circle(img, c, 6, (30, 30, 30), 1)
            cv2.line(img, c, self.px(bx + 0.15 * math.cos(byaw), by + 0.15 * math.sin(byaw)), (0, 200, 0), 2)
            d = math.dist((ox, oy), (bx, by))
            cv2.putText(img, f'base {d:.2f}m', self.px((ox + bx) / 2 + 0.03, (oy + by) / 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 130, 0), 1)
        cv2.putText(img, 'robot: fill=real(plant) ring=/odom green=base_footprint(/tf) path=/plan  purple=/tracked_objects  grey=obstacle  blue=mover',
                    (MARGIN - 25, self.size[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.34, (80, 80, 80), 1)
        return img


def _rect(x, y, yaw, sx, sy):
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + c * dx - s * dy, y + s * dx + c * dy) for dx, dy in
            ((-sx / 2, -sy / 2), (sx / 2, -sy / 2), (sx / 2, sy / 2), (-sx / 2, sy / 2))]


# ---------- 카메라 타일 (SIM 렌더 우선 — ROS 왕복분 빠름, 없으면 ROS가 낸 것)
def camera_tiles(sim, cam, view):
    with sim.lock:
        fresh = time.time() - sim.render_ts.get(cam, 0) < 3.0
        cached = sim.renders.get(cam)
    if fresh and cached is not None:
        color, depth, _ = cached
        source = 'SIM'
    else:
        color = cv2.imread(str(TOPIC_DIR / cam / 'color/image_raw/image.jpg'))
        depth = cv2.imread(str(TOPIC_DIR / cam / 'aligned_depth_to_color/image_raw/depth.png'), cv2.IMREAD_ANYDEPTH)
        source = 'ROS'
        if color is None or depth is None:
            if cached is None:
                blank = np.zeros((TILE[1], TILE[0], 3), np.uint8)
                cv2.putText(blank, 'waiting (test-faker 0?)', (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                            (200, 200, 200), 1)
                return blank, blank, blank
            color, depth, _ = cached
            source = 'SIM'
    if color.shape[:2] != depth.shape:
        color = cv2.resize(color, (depth.shape[1], depth.shape[0]))
    rgb = cv2.resize(color, TILE)
    dep = cv2.resize(scene.depth_colormap(depth), TILE, interpolation=cv2.INTER_NEAREST)
    cloud = scene.cloud_view(depth, TILE, view[0], view[1], color=color, step=4 if depth.shape[1] > 400 else 2)
    cv2.putText(rgb, source, (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
    return rgb, dep, cloud


def status_lines(sim=None):
    """[(글, 태그)] — 작업 상태 + 플랜트"""
    lines = []

    def add(label, topic, fmt):
        data, age = read_msg(topic)
        if data is None:
            lines.append((f'{label}: (없음)', 'warn'))
            return
        try:
            text = fmt(data)
        except (KeyError, IndexError, TypeError) as e:
            text = f'(형식 다름: {e})'
        lines.append((f'{label}: {text}  [{age:.1f}s 전]', 'warn' if age > STALE_S else ''))

    add('작업', 'task_status', lambda d: f"plan {d['plan_id']} step {d['step']} {STATE_NAMES.get(d['state'], d['state'])} {d['reason']}")
    add('계획', 'sub_goals', lambda d: f"plan {d['plan_id']}: " + ' → '.join(f"{g['skill']}({g['target']})" for g in d['goals']))
    add('로봇 말', 'robot_say', lambda d: d['data'])
    add('운영 모드', 'operation_mode', lambda d: d['data'])
    add('속도 상한', 'speed_limit', lambda d: f"{d['speed_limit']:.0f}{'%' if d['percentage'] else ' m/s'}")
    add('위치(odom)', 'odom', lambda d: '({x:.2f}, {y:.2f})'.format(**d['pose']['pose']['position'])
        + f" yaw {math.degrees(yaw_of(d['pose']['pose']['orientation'])):.0f}°")
    add('팔', 'joint_states', lambda d: ' '.join(
        f'{n.replace("_joint", "")}={p:.2f}' for n, p in zip(d['name'], d['position']) if not n.startswith('wheel')))
    if sim:
        p = sim.plant
        if p.alive:
            x, y, yaw, _ = p.pose()
            lines.append((f'플랜트(OpenCR 흉내): 연결 · C {p.cmds}줄 · 실제 ({x - sim.spawn[0]:.2f}, {y - sim.spawn[1]:.2f}) '
                          f'yaw {math.degrees(yaw):.0f}°{" · 충돌!" if p.bumped else ""}', 'error' if p.bumped else ''))
        else:
            lines.append(('플랜트(OpenCR 흉내): 대기 — 컨테이너에서 test-faker 0 mock_hardware:=false', 'warn'))
    return lines


def diagnostics_lines():
    """/diagnostics_agg → [(글, 태그)] — 노드별 항목만 (묶음 줄은 뺌)"""
    data, age = read_msg('diagnostics_agg')
    if not data:
        return [('(진단 없음 — bringup의 diagnostic_aggregator)', 'warn')]
    out = []
    for s in sorted(data['status'], key=lambda s: s['name']):
        if ': ' not in s['name']:
            continue
        level, tag = LEVEL.get(s['level'], ('?', 'warn'))
        out.append((f"{level:5} {s['name'].split('/')[-1]} — {s['message']}", tag))
    if age > STALE_S:
        out.insert(0, (f'(진단이 {age:.0f}s 전 값)', 'warn'))
    return out


def to_photo(img):
    ok, buf = cv2.imencode('.png', img)
    return tk.PhotoImage(data=base64.b64encode(buf.tobytes())) if ok else None


# ---------- 창
class App:
    def __init__(self, root, sim, view):
        self.root, self.sim, self.view = root, sim, view
        self.cloud_view = {cam: [0.6, -0.4] for cam in CAMERAS}
        self.drag = None
        root.title('lv2_module6 SIL 시뮬레이터')
        left = tk.Frame(root)
        left.pack(side=tk.LEFT, padx=6, pady=6, anchor=tk.N)
        right = tk.Frame(root)
        right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=6, pady=6)

        top = tk.Frame(left)  # 지도 + 로봇상태 좌우 절반씩
        top.pack(fill=tk.X)
        mapbox = tk.Frame(top)
        mapbox.pack(side=tk.LEFT, anchor=tk.N)
        self.map_label = tk.Label(mapbox, cursor='hand2', takefocus=1)
        self.map_label.pack()
        self.map_label.bind('<ButtonPress-1>', self.on_map_press)
        self.map_label.bind('<B1-Motion>', self.on_map_drag)
        self.map_label.bind('<ButtonRelease-1>', self.on_map_release)
        self.map_label.bind('<ButtonPress-3>', self.on_map_pan_press)
        self.map_label.bind('<B3-Motion>', self.on_map_pan_drag)
        self.map_label.bind('<MouseWheel>', self.on_map_wheel)
        self.map_label.bind('<Button-4>', lambda e: self.on_map_wheel_step(1))
        self.map_label.bind('<Button-5>', lambda e: self.on_map_wheel_step(-1))
        tk.Button(mapbox, text='지도 리셋', command=self.view.reset_view).pack(anchor=tk.W)
        robotbox = tk.Frame(top)
        robotbox.pack(side=tk.LEFT, anchor=tk.N, padx=(6, 0))
        tk.Label(robotbox, text='로봇 (지금 상태) — 드래그 회전 · 휠 줌', anchor=tk.W).pack(fill=tk.X)
        self.robot_label = tk.Label(robotbox, bg='black', width=ROBOT_VIEW[0], height=ROBOT_VIEW[1])
        self.robot_label.pack()
        self.robot_label.bind('<ButtonPress-1>', self.on_robot_press)
        self.robot_label.bind('<B1-Motion>', self.on_robot_drag)
        self.robot_label.bind('<MouseWheel>', lambda e: self.robot_cam.zoom(1 if e.delta > 0 else -1))
        self.robot_label.bind('<Button-4>', lambda e: self.robot_cam.zoom(1))
        self.robot_label.bind('<Button-5>', lambda e: self.robot_cam.zoom(-1))
        self.robot_model = None
        self.robot_cam = robot_view.OrbitView(target=(0, 0, 0.2), dist=1.3)
        self.show_frames = tk.BooleanVar(value=False)
        tk.Checkbutton(left, text='관절축 보기 (X빨·Y초·Z파)', variable=self.show_frames).pack(anchor=tk.W)
        self.coords_label = tk.Label(left, justify=tk.LEFT, font=('Menlo', 9), anchor=tk.W)
        self.coords_label.pack(anchor=tk.W)

        # 퍽 · 배경 · 카메라
        row = tk.Frame(left)
        row.pack(fill=tk.X, pady=2)
        tk.Button(row, text='퍽 랜덤 배치', command=lambda: self.sim.randomize(int(self.count.get()))).pack(side=tk.LEFT)
        self.count = tk.Spinbox(row, from_=0, to=20, width=3)
        self.count.delete(0, tk.END)
        self.count.insert(0, '6')
        self.count.pack(side=tk.LEFT)
        tk.Label(row, text='개  배경').pack(side=tk.LEFT)
        names = [NO_BG] + [p.name for p in scene.backgrounds(BG_DIR)]
        self.bg = tk.StringVar(value=names[1] if len(names) > 1 else NO_BG)
        tk.OptionMenu(row, self.bg, *names, command=self.on_background).pack(side=tk.LEFT)
        self.on_background(self.bg.get())
        tk.Label(row, text='바닥').pack(side=tk.LEFT)
        fnames = [NO_FLOOR] + [p.name for p in scene.backgrounds(FLOOR_DIR) if p.name != 'FLOOR_CREDITS.txt']
        self.floor = tk.StringVar(value=fnames[1] if len(fnames) > 1 else NO_FLOOR)
        tk.OptionMenu(row, self.floor, *fnames, command=self.on_floor).pack(side=tk.LEFT)
        self.on_floor(self.floor.get())
        tk.Label(row, text='벽').pack(side=tk.LEFT)
        wnames = [NO_WALL] + [p.name for p in scene.backgrounds(WALL_DIR) if p.suffix == '.jpg']
        self.wall = tk.StringVar(value=wnames[1] if len(wnames) > 1 else NO_WALL)
        tk.OptionMenu(row, self.wall, *wnames, command=self.on_wall).pack(side=tk.LEFT)
        self.on_wall(self.wall.get())
        self.stream = tk.BooleanVar(value=True)
        tk.Checkbutton(row, text='카메라 송출', variable=self.stream, command=self.on_stream).pack(side=tk.LEFT)

        # 환경
        env = tk.LabelFrame(left, text='환경 (플랜트: mock_hardware:=false일 때 로봇 몸에 적용)')
        env.pack(fill=tk.X, pady=2)
        self.slip = tk.Scale(env, label='미끄럼', from_=0.0, to=0.6, resolution=0.05, orient=tk.HORIZONTAL, length=150,
                             command=lambda v: setattr(self.sim.plant, 'slip', float(v)))
        self.slip.pack(side=tk.LEFT)
        self.friction = tk.Scale(env, label='마찰(가속)', from_=0.2, to=1.5, resolution=0.1, orient=tk.HORIZONTAL,
                                 length=150, command=lambda v: setattr(self.sim.plant, 'friction', float(v)))
        self.friction.set(1.0)
        self.friction.pack(side=tk.LEFT)
        col = tk.Frame(env)
        col.pack(side=tk.LEFT, padx=4)
        tk.Button(col, text='이동물체 추가', command=self.sim.spawn_mover).pack(fill=tk.X)
        tk.Button(col, text='이동물체 삭제', command=lambda: self.sim.pop_prop('mover')).pack(fill=tk.X)
        tk.Button(col, text='바닥 장애물 추가', command=self.sim.add_obstacle).pack(fill=tk.X)
        tk.Button(col, text='바닥 장애물 삭제', command=lambda: self.sim.pop_prop('obstacle')).pack(fill=tk.X)
        tk.Button(col, text='신호등 추가', command=self.sim.spawn_traffic).pack(fill=tk.X)
        tk.Button(col, text='신호등 삭제', command=lambda: self.sim.pop_prop('traffic_light')).pack(fill=tk.X)
        # faker를 다시 띄우면 /odom이 0부터 — 실제 로봇도 시작 칸으로 돌려 맞춘다
        tk.Button(col, text='로봇 위치 초기화', command=self.sim.reset_robot).pack(fill=tk.X)
        col = tk.Frame(env)
        col.pack(side=tk.LEFT, padx=4)
        self.comm = tk.BooleanVar(value=False)
        tk.Checkbutton(col, text='OpenCR 통신 끊기', variable=self.comm,
                       command=lambda: setattr(self.sim.plant, 'comm_down', self.comm.get())).pack(anchor=tk.W)
        self.grab = tk.BooleanVar(value=False)
        tk.Checkbutton(col, text='팔 붙잡기(과전류)', variable=self.grab,
                       command=lambda: setattr(self.sim.plant, 'arm_grabbed', self.grab.get())).pack(anchor=tk.W)


        # 카메라
        self.tiles = {}
        for cam in CAMERAS:
            tk.Label(right, text=f'{cam}   ROS 영상 │ 깊이 │ 점구름 (드래그로 회전)', anchor=tk.W).pack(fill=tk.X)
            r = tk.Frame(right)
            r.pack(anchor=tk.W)
            labels = [tk.Label(r, bg='black', width=TILE[0], height=TILE[1]) for _ in range(3)]
            for lab in labels:
                lab.pack(side=tk.LEFT, padx=1)
            labels[2].bind('<B1-Motion>', lambda e, c=cam: self.on_cloud_drag(e, c))
            labels[2].bind('<ButtonPress-1>', lambda e, c=cam: self.on_cloud_press(e, c))
            self.tiles[cam] = labels

        texts = ttk.Notebook(right)
        texts.pack(fill=tk.BOTH, expand=True, pady=4)
        status_tab = tk.Frame(texts)
        texts.add(status_tab, text='상태 · 진단')
        self.status = tk.Text(status_tab, width=60, height=12, font=('Menlo', 10))
        self.status.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.diag = tk.Text(status_tab, width=60, height=12, font=('Menlo', 10))
        self.diag.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        for t in (self.status, self.diag):
            t.tag_config('warn', foreground='#b26a00')
            t.tag_config('error', foreground='#c62828')
        self.manual_tab = tk.Frame(texts)
        texts.add(self.manual_tab, text='수동 조작')
        self.manual_panel = ManualPanel(self.manual_tab, sim)
        self.manual_panel.pack(fill=tk.BOTH, expand=True)
        self.tabs = texts
        self.tabs.bind('<<NotebookTabChanged>>', lambda _: self.on_tab())
        cmd = tk.Frame(right)
        cmd.pack(fill=tk.X)
        tk.Label(cmd, text='명령').pack(side=tk.LEFT)
        self.entry = tk.Entry(cmd, width=34)
        self.entry.pack(side=tk.LEFT, fill=tk.X, expand=True)
        self.entry.bind('<Return>', lambda _: self.send_command())
        tk.Button(cmd, text='보내기', command=self.send_command).pack(side=tk.LEFT)
        for text in QUICK_COMMANDS:
            tk.Button(cmd, text=text, command=lambda t=text: self.send_command(t)).pack(side=tk.LEFT)
        self.sent = tk.Label(right, text='', fg='#2e7d32')
        self.sent.pack(anchor=tk.W)
        self.photos = []
        self.tile_photos = {}  # 타일 사진 별도 보관 — 타일을 격틱 갱신해도 홀틱에 GC로 지워지지 않게
        self.tick()

    # --- 조작
    def on_background(self, name):
        self.sim.renderer.set_background(None if name == NO_BG else BG_DIR / name)

    def on_floor(self, name):
        self.sim.renderer.set_floor(None if name == NO_FLOOR else FLOOR_DIR / name)

    def on_wall(self, name):
        self.sim.renderer.set_wall(None if name == NO_WALL else WALL_DIR / name)

    def on_stream(self):
        self.sim.streaming = self.stream.get()
        if not self.sim.streaming:
            self.sim.stop_streaming()

    def on_tab(self):
        if self.tabs.select() == str(self.manual_tab):
            self.manual_panel.show()
        else:
            self.manual_panel.hide()

    def on_map_press(self, e):
        x, y = self.view.world(e.x, e.y)
        rx, ry, _, _ = self.sim.robot_xy_yaw()
        self.drag_robot = math.dist((x, y), (rx, ry)) < 0.25
        if self.drag_robot:
            self.drag = None
            if self.sim.plant.alive:
                self.sim.teleport_robot(x, y)
            self.map_pan = None
            return
        with self.sim.lock:
            near = [p for p in self.sim.pucks + self.sim.props if math.dist((x, y), (p['x'], p['y'])) < 0.2]
        self.drag = min(near, key=lambda p: math.dist((x, y), (p['x'], p['y']))) if near else None
        # 빈 곳 왼쪽 드래그 = 지도 이동 (확대 상태에서만)
        self.map_pan = (e.x, e.y, self.view.cx, self.view.cy)

    def on_map_drag(self, e):
        if getattr(self, 'drag_robot', False):
            if self.sim.plant.alive:  # 플랜트는 실시간, mock은 릴리스 때 1회
                x, y = self.view.world(e.x, e.y)
                self.sim.teleport_robot(x, y)
            return
        if self.drag:
            with self.sim.lock:
                self.drag['x'], self.drag['y'] = (round(v, 3) for v in self.view.world(e.x, e.y))
        elif self.view.zoom > 1.0:
            x0, y0, cx0, cy0 = self.map_pan
            self.view.cx, self.view.cy = cx0 - (e.x - x0) / self.view.zoom, cy0 - (e.y - y0) / self.view.zoom

    def on_map_release(self, _):
        if getattr(self, 'drag_robot', False):
            self.drag_robot = False
            if not self.sim.plant.alive:
                x, y = self.view.world(_.x, _.y)
                self.sim.teleport_robot(x, y)
            return
        if self.drag:
            self.sim.save_truth()
        self.drag = None

    def on_map_pan_press(self, e):
        self.map_pan = (e.x, e.y, self.view.cx, self.view.cy)

    def on_map_pan_drag(self, e):
        if self.view.zoom <= 1.0:
            return
        x0, y0, cx0, cy0 = self.map_pan
        self.view.cx, self.view.cy = cx0 - (e.x - x0) / self.view.zoom, cy0 - (e.y - y0) / self.view.zoom

    def on_map_wheel(self, e):
        self.on_map_wheel_step(1 if e.delta > 0 else -1, e.x, e.y)

    def on_map_wheel_step(self, direction, lx=None, ly=None):
        old = self.view.zoom
        new = min(4.0, max(1.0, old * (1.15 if direction > 0 else 1 / 1.15)))
        if new == old:
            return
        if lx is not None:
            # 커서 아래 점 고정 — 중심 이동 (zoom 1에서도 커서 기준 확대)
            x0, y0, _, _ = self.view._roi()
            fx, fy = x0 + lx / old, y0 + ly / old
            h, w = self.view.size[1], self.view.size[0]
            self.view.cx = fx - (lx - w / 2) / new
            self.view.cy = fy - (ly - h / 2) / new
        self.view.zoom = new

    def on_cloud_press(self, e, cam):
        self.cloud_press = (e.x, e.y, *self.cloud_view[cam])

    def on_cloud_drag(self, e, cam):
        x0, y0, yaw, pitch = self.cloud_press
        self.cloud_view[cam] = [yaw + (e.x - x0) * 0.01, max(-1.5, min(1.5, pitch - (e.y - y0) * 0.01))]
        self.refresh_cam(cam)  # 틱을 기다리지 않고 즉시

    def on_robot_press(self, e):
        self.robot_drag = (e.x, e.y)

    def on_robot_drag(self, e):
        x0, y0 = self.robot_drag
        self.robot_cam.orbit(e.x - x0, e.y - y0)
        self.robot_drag = (e.x, e.y)
        self.refresh_robot()  # 틱을 기다리지 않고 즉시 (드래그가 부드럽게)

    def refresh_robot(self):
        q = getattr(self, '_last_q', None)
        if self.robot_model is None or not q:
            return
        rx, ry, ryaw, _ = self.sim.robot_xy_yaw()
        T_wb = scene.transform(scene.rpy_matrix(0, 0, ryaw), (rx, ry, 0))
        faces = self.robot_model.faces(q) + self.world_object_faces(T_wb)
        img = self.robot_cam.render(faces, ROBOT_VIEW)
        self.draw_wheels(img, q, getattr(self, '_last_wheel', [0.0, 0.0]))
        if self.show_frames.get():
            self.draw_frames(img, q)
        self._robot_ph = to_photo(img)
        self.robot_label.config(image=self._robot_ph, width=ROBOT_VIEW[0], height=ROBOT_VIEW[1])

    def refresh_cam(self, cam):
        for i, (lab, img) in enumerate(zip(self.tiles[cam], camera_tiles(self.sim, cam, self.cloud_view[cam]))):
            ph = to_photo(img)
            self.tile_photos[(cam, i)] = ph
            lab.config(image=ph, width=TILE[0], height=TILE[1])

    def draw_wheels(self, img, q, wheel_vel):
        """바퀴 회전 표시 — 바퀴 중심에 회오리(스포크): 각도=관절각, 색=회전 중 주황·정지 회색"""
        import numpy as _np
        frames = self.robot_model.joint_frames(q)
        for i, (joint, w) in enumerate((('wheel_left_joint', wheel_vel[0]),
                                        ('wheel_right_joint', wheel_vel[1]))):
            if joint not in frames or joint not in self.robot_model.joints:
                continue
            origin, axis = frames[joint]
            axis = _np.asarray(axis, float)
            axis = axis / max(_np.linalg.norm(axis), 1e-9)
            up = _np.array([0.0, 0.0, 1.0])
            p0 = _np.cross(axis, up)
            if _np.linalg.norm(p0) < 1e-6:
                p0 = _np.array([1.0, 0.0, 0.0])
            p0 = p0 / _np.linalg.norm(p0)
            ang = q.get(joint, 0.0)
            rim = origin + (math.cos(ang) * p0 + math.sin(ang) * _np.cross(axis, p0)) * 0.033
            uv, z = self.robot_cam.project([origin, rim], ROBOT_VIEW)
            if (z < 0.02).any():
                continue
            c, r_ = tuple(uv[0].astype(int)), tuple(uv[1].astype(int))
            col = (0, 140, 255) if abs(w) > 0.05 else (150, 150, 150)
            cv2.line(img, c, r_, col, 2, cv2.LINE_AA)
            cv2.circle(img, r_, 3, col, -1, cv2.LINE_AA)
            cv2.putText(img, f'{"L" if i == 0 else "R"}{w:+.1f}', (c[0] + 8, c[1]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, col, 1, cv2.LINE_AA)

    def draw_frames(self, img, q):
        """관절축 트라이어드 (자식 링크 프레임: X빨·Y초·Z파). 바퀴는 자식 프레임이 같이 돌아 회전도 보임"""
        import numpy as _np
        cols = [(0, 0, 255), (0, 255, 0), (255, 0, 0)]
        for _, (child, _, _, _, _) in self.robot_model.joints.items():
            T = self.robot_model.kin.pose(child, q)
            o, R = T[:3, 3], T[:3, :3]
            pts = [o] + [o + R[:, i] * 0.05 for i in range(3)]
            uv, z = self.robot_cam.project(pts, ROBOT_VIEW)
            if (z < 0.02).any():
                continue
            p = uv.astype(int)
            for i in range(3):
                cv2.line(img, tuple(p[0]), tuple(p[i + 1]), cols[i], 2, cv2.LINE_AA)

    def world_object_faces(self, T_wb):
        """퍽·장애물·이동물체·신호등을 로봇 루트 프레임 면 목록으로 — 상태 3D에 같이 그림"""
        import numpy as _np
        R, t = _np.asarray(T_wb[:3, :3], float), _np.asarray(T_wb[:3, 3], float)
        out = []
        with self.sim.lock:
            things = [dict(p) for p in self.sim.pucks + self.sim.props]
        for p in things:
            color = scene.PALETTE.get(p['color'], (128, 128, 128)) if isinstance(p['color'], str) else tuple(p['color'])
            color = tuple(int(v) for v in color)
            shape = p['shape']
            r_or_sx, h = p['size'][0], p['size'][1]
            zo = float(p.get('z', 0.0))
            if p.get('kind') == 'traffic_light':  # 폴 + 헤드 + 3램프(켜진 것만 상태색)
                from scene import _tl_dim as _dim
                parts = [(robot_view.primitive_faces('box', 0.04, scene.TL_POLE_H, 0.04), scene.TL_POLE_H / 2,
                          (70, 70, 70)),
                         (robot_view.primitive_faces('box', *scene.TL_HEAD[:2], scene.TL_HEAD[3] - scene.TL_HEAD[2]),
                          (scene.TL_HEAD[2] + scene.TL_HEAD[3]) / 2, (40, 40, 40))]
                for _name, _lz in scene.TL_LAMPS:
                    _on = p.get('state') == _name
                    _rgb = tuple(int(v) for v in scene.TL_COLORS[_name]) if _on else _dim(scene.TL_COLORS[_name])
                    _sph = robot_view.primitive_faces('sphere', scene.TL_LAMP_R, 0)
                    for _sx in (0.06, -0.06):
                        parts.append(([(vv + _np.array([_sx, 0.0, 0.0])) for vv in _sph], _lz, _rgb))
            elif shape == 'sphere':
                parts = [(robot_view.primitive_faces(shape, r_or_sx, 0), r_or_sx + zo, color)]
            elif shape == 'cylinder':
                parts = [(robot_view.primitive_faces(shape, r_or_sx, h), h / 2 + zo, color)]
            else:
                parts = [(robot_view.primitive_faces(shape, p.get('sx', r_or_sx), h, p.get('sy', r_or_sx)),
                          h / 2 + zo, color)]
            cy, sy = math.cos(p.get('yaw', 0.0)), math.sin(p.get('yaw', 0.0))
            Rz = _np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
            c = _np.array([p['x'], p['y'], 0.0])
            for faces, z0, col in parts:
                cc = c + _np.array([0.0, 0.0, z0])
                for f in faces:
                    out.append(((_np.asarray(f, float) @ Rz.T + cc - t) @ R, col))
        return out

    def coords_text(self):
        """map · odom · base_footprint 좌표 (map = odom + spawn)"""
        odom, _ = read_msg('odom')
        if not odom:
            return '좌표: (데이터 없음 — test-faker 실행 중인지 확인)'
        pose = odom['pose']['pose']
        ox, oy = pose['position']['x'], pose['position']['y']
        oyaw = math.degrees(yaw_of(pose['orientation']))
        mx, my = self.view.from_map(ox, oy)
        base = self.view.base_from_tf()
        lines = [f'map   ({mx:+.2f}, {my:+.2f}) yaw {oyaw:+.0f}°',
                 f'odom  ({ox:+.2f}, {oy:+.2f}) yaw {oyaw:+.0f}°']
        if base is not None:
            lines.append(f'base  ({base[0]:+.2f}, {base[1]:+.2f}) yaw {math.degrees(base[2]):+.0f}°')
        else:
            lines.append('base  (없음 — /tf 대기)')
        return '\n'.join(lines)

    def send_command(self, text=None):
        text = (text or self.entry.get()).strip()
        if not text:
            return
        send('/user_command', 'std_msgs/msg/String', {'data': text})
        self.sent.config(text=f'보냄: {text}  ({time.strftime("%H:%M:%S")}) — input_manager가 /user_command로 발행')
        self.entry.delete(0, tk.END)

    def tick(self):
        self.tick_n = getattr(self, 'tick_n', 0) + 1
        photos = [to_photo(self.view.viewport(self.view.render()))]  # Tk가 이미지를 붙잡도록 보관
        self.map_label.config(image=photos[0])
        if self.robot_model is None:
            desc, _ = read_msg('robot_description')
            if desc:
                self.robot_model = robot_view.RobotModel(desc['data'])
        q, wheel_vel = current_joints(self.sim)
        self._last_q, self._last_wheel = q, wheel_vel
        if self.robot_model is not None and q:
            self.refresh_robot()
            photos.append(self._robot_ph)
        else:
            blank = np.zeros((ROBOT_VIEW[1], ROBOT_VIEW[0], 3), np.uint8)
            cv2.putText(blank, 'waiting /robot_description', (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                        (200, 200, 200), 1)
            photos.append(to_photo(blank))
        self.robot_label.config(image=photos[-1], width=ROBOT_VIEW[0], height=ROBOT_VIEW[1])
        self.coords_label.config(text=self.coords_text())
        if self.tick_n % 2 == 0:  # 카메라 타일은 0.5초마다 (Tk 디코딩 부하 분산)
            for cam in self.tiles:
                self.refresh_cam(cam)
                for i, lab in enumerate(self.tiles[cam]):
                    photos.append(self.tile_photos[(cam, i)])
        if self.tick_n % 2 == 0:  # 상태 텍스트도 0.5초마다
            for widget, lines in ((self.status, status_lines(self.sim)),):
                widget.delete('1.0', tk.END)
                for text, tag in lines:
                    widget.insert(tk.END, text + '\n', (tag,) if tag else ())
        if self.tick_n % 4 == 0:  # 진단(diagnostics_agg 대용량)은 1초마다
            self.diag.delete('1.0', tk.END)
            for text, tag in diagnostics_lines():
                self.diag.insert(tk.END, text + '\n', (tag,) if tag else ())
        self.photos = photos
        self.root.after(250, self.tick)


def snapshot(sim, view, path):
    """창 없이 한 장: 지도 | 카메라 타일 2줄"""
    sim.step()
    tiles = np.vstack([np.hstack(camera_tiles(sim, cam, (0.6, -0.4))) for cam in CAMERAS])
    m = view.render()
    h = max(m.shape[0], tiles.shape[0])
    pad = lambda a: np.vstack([a, np.full((h - a.shape[0], a.shape[1], 3), 255, np.uint8)])
    cv2.imwrite(path, np.hstack([pad(m), pad(tiles)]))
    for text, tag in status_lines(sim) + diagnostics_lines():
        print(('! ' if tag else '  ') + text)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--snapshot', help='창 없이 화면 한 장을 이 PNG로 저장하고 상태를 출력한 뒤 끝냄')
    parser.add_argument('--spawn', help='map 원점의 미로 좌표 "x,y" (기본: 코스 spawn)')
    args = parser.parse_args()
    spawn = tuple(float(v) for v in args.spawn.split(',')) if args.spawn else None
    sim = Sim(maze.load(), spawn)
    view = MapView(sim)
    bgs = scene.backgrounds(BG_DIR)
    sim.renderer.set_background(bgs[0] if bgs else None)
    fls = [p for p in scene.backgrounds(FLOOR_DIR) if p.name != 'FLOOR_CREDITS.txt']
    sim.renderer.set_floor(fls[0] if fls else None)
    wls = [p for p in scene.backgrounds(WALL_DIR) if p.suffix == '.jpg']
    sim.renderer.set_wall(wls[0] if wls else None)
    if args.snapshot:
        snapshot(sim, view, args.snapshot)
        return
    threading.Thread(target=sim.plant.loop, daemon=True).start()
    threading.Thread(target=sim.render_loop, daemon=True).start()
    root = tk.Tk()
    App(root, sim, view)
    try:
        root.mainloop()
    finally:
        sim.running = sim.plant.running = False
        sim.stop_streaming()  # 창을 닫으면 송출도 끝 (fake_camera는 회색으로)


if __name__ == '__main__':
    main()
