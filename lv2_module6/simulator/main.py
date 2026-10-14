"""SIL 시뮬레이터 (호스트에서 실행) — 컨테이너(ROS)와는 공유 폴더 lv2_module6/debug 의 파일로만 주고받는다 (호스트에 ROS 불필요)

  [지도: 미로 · 실제 로봇 · /odom · 경로 · 인지 물체 · 퍽 · 장애물 · 사람]  │ [pole_cam:    ROS 영상 │ 깊이 │ 점구름]
  [퍽 · 배경 · 카메라 송출]                                                  │ [gripper_cam: ROS 영상 │ 깊이 │ 점구름]
  [환경: 미끄럼 · 마찰 · 사람 끼어들기 · 바닥 장애물 · 통신 끊기 · 팔 붙잡기]   │ [상태 · 진단(/diagnostics_agg)]
  [수동 조작 창 열기 · 비상정지]                                             │ [명령 입력 → /user_command]

SIL(Software-in-the-Loop): 실기 대신 이 시뮬레이터가 세상(코스·퍽·장애물·사람·카메라 장면·로봇 몸)을 만들고,
ROS 쪽은 실제 노드·컨트롤러가 그대로 돈다.
  ① 카메라: 로봇 자세로 두 카메라 시점 렌더(scene.py) → debug/input-live/ → fake_camera → RealSense 토픽
  ② 하드웨어(플랜트, plant.py): faker를 mock_hardware:=false로 띄우면 controller_manager가 가상 시리얼로 보내는
     C 명령(debug/serial-out)을 받아 바퀴·팔을 움직이고(미끄럼·마찰·충돌) S 상태(debug/serial-in)를 돌려준다.
     → 지도의 '실제 로봇'(채움)과 /odom(테두리)이 미끄럼만큼 어긋난다.  mock_hardware:=true(기본)면 /odom을 따라 그린다.
  ③ ROS → 화면: monitor_manager가 debug/topic/<토픽>/ 에 저장한 것을 표시
  ④ 화면 → ROS: input_manager가 debug/input/<토픽>/message 를 발행 (명령 · 수동 조작 · 비상정지)
- 지도: 퍽(정답, 채움 — 왼쪽 드래그로 이동) · 바닥 장애물(회색 상자) · 사람(파랑, 움직임) · 인지 물체(/tracked_objects, 보라 테두리)
- 수동 조작: [수동 조작 창 열기] → 별도 창 (manual_window.py) — 열면 수동 모드, 닫으면 자동 모드
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
from pathlib import Path

import cv2
import numpy as np

import maze
import plant as plant_mod
import scene
from bridge import (DEBUG, INPUT_DIR, LIVE_DIR, SIM_DIR, TOPIC_DIR, read_msg, send, send_arm, send_bool, send_twist,
                    write_atomic, yaw_of)
from manual_window import ManualWindow

HERE = Path(__file__).resolve().parent
BG_DIR = HERE / 'images' / 'background'
CAMERAS = ('pole_cam', 'gripper_cam')
TILE = (256, 192)
STALE_S = 2.0
PX_PER_M = 110
MARGIN = 30
STATE_NAMES = {0: 'RUNNING', 1: 'SUCCEEDED', 2: 'FAILED'}
LEVEL = {0: ('OK', ''), 1: ('WARN', 'warn'), 2: ('ERROR', 'error'), 3: ('STALE', 'warn')}
QUICK_COMMANDS = ['빨간 퍽을 목표 구역으로 옮겨', '모든 퍽을 목표 구역으로 옮겨']
NO_BG = '(격자)'
ARM_LIMITS = {'joint1': (-3.14, 3.14), 'joint2': (-1.5, 1.5), 'joint3': (-1.5, 1.4), 'joint4': (-1.7, 1.97),
              'joint5': (-3.14, 3.14), 'gripper_left_joint': (-0.011, 0.02)}


# ---------- 시뮬레이션 상태
class Sim:
    def __init__(self, course, spawn=None):
        self.course = course
        self.spawn = spawn or course['spawn']
        self.walls = np.asarray(maze.wall_boxes(course), float)
        self.renderer = scene.Renderer(self.walls)
        self.cells = scene.maze_cells(course)
        self.kin = None
        self.lock = threading.Lock()
        self.pucks = []
        self.props = []           # 바닥 장애물 · 사람
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

    def spawn_person(self):
        """로봇 앞 1.2m를 가로질러 지나가는 사람"""
        x, y, yaw, _ = self.robot_xy_yaw()
        ax, ay = x + 1.2 * math.cos(yaw), y + 1.2 * math.sin(yaw)
        side = random.choice((-1, 1))
        heading = yaw + side * math.pi / 2
        start = (ax - 1.0 * math.cos(heading), ay - 1.0 * math.sin(heading))
        with self.lock:
            self.props.append(scene.person(*start, heading))

    def move_people(self, dt):
        with self.lock:
            for p in self.props:
                if p['kind'] == 'person':
                    p['x'] += p['vx'] * dt
                    p['y'] += p['vy'] * dt
                    p['until'] -= dt
            self.props = [p for p in self.props if p['kind'] != 'person' or p['until'] > 0]

    def save_truth(self):
        """정답 물체 목록 (미로 좌표 + map 좌표) — 인지 결과와 비교용"""
        with self.lock:
            out = {'pucks': [dict(p, map_x=round(p['x'] - self.spawn[0], 3), map_y=round(p['y'] - self.spawn[1], 3))
                             for p in self.pucks],
                   'obstacles': [dict(p, color=list(p['color'])) for p in self.props if p['kind'] == 'obstacle']}
        write_atomic(SIM_DIR / 'objects.json', lambda p: p.write_text(json.dumps(out, indent=1)))

    # --- 충돌 (플랜트가 부름)
    def collide(self, x, y, r):
        lo, hi = self.walls[:, :2] - self.walls[:, 3:5] / 2, self.walls[:, :2] + self.walls[:, 3:5] / 2
        nearest = np.clip([x, y], lo, hi)
        if (np.sum((nearest - [x, y]) ** 2, axis=1) < r * r).any():
            return True
        with self.lock:
            for p in self.props:
                reach = p['size'][0] if p['shape'] == 'cylinder' else max(p.get('sx', 0.1), p.get('sy', 0.1)) / 2
                if math.dist((x, y), (p['x'], p['y'])) < r + reach:
                    return True
        return False

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
        if self.kin is None:
            return False
        x, y, yaw, q = self.robot_xy_yaw()
        T_wb = scene.transform(scene.rpy_matrix(0, 0, yaw), (x, y, 0))
        with self.lock:
            things = [dict(p) for p in self.pucks + self.props]
        for cam in CAMERAS:
            T = T_wb @ self.kin.pose(f'{cam}_color_optical_frame', q)
            bgr, depth = self.renderer.render(T, things)
            self.renders[cam] = (bgr, depth, T)
            if self.streaming:
                write_atomic(LIVE_DIR / f'{cam}.png', lambda p: cv2.imwrite(str(p), bgr) or None)
                write_atomic(LIVE_DIR / f'{cam}_depth.png', lambda p: cv2.imwrite(str(p), depth) or None)
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
            t = now
            if not self.step():
                time.sleep(0.5)  # 로봇 URDF(/robot_description) 대기 — test-faker 0 실행 중인지


# ---------- 지도
class MapView:
    def __init__(self, sim):
        self.sim = sim
        w, h = sim.course['size']
        self.half = (w / 2 + 0.2, h / 2 + 0.2)
        self.size = (int(2 * self.half[0] * PX_PER_M) + 2 * MARGIN, int(2 * self.half[1] * PX_PER_M) + 2 * MARGIN)
        self.walls = maze.walls_mask(sim.course, self.px_array, (self.size[1], self.size[0]))

    def px_array(self, xy):
        return np.stack([MARGIN + (xy[..., 0] + self.half[0]) * PX_PER_M,
                         MARGIN + (self.half[1] - xy[..., 1]) * PX_PER_M], -1).astype(np.int32)

    def px(self, x, y):
        return (int(MARGIN + (x + self.half[0]) * PX_PER_M), int(MARGIN + (self.half[1] - y) * PX_PER_M))

    def world(self, u, v):
        return ((u - MARGIN) / PX_PER_M - self.half[0], self.half[1] - (v - MARGIN) / PX_PER_M)

    def from_map(self, x, y):
        return x + self.sim.spawn[0], y + self.sim.spawn[1]

    def robot(self, img, x, y, yaw, color, filled):
        c = self.px(x, y)
        r = int(plant_mod.ROBOT_R * PX_PER_M)
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
                r = max(4, int(p['size'][0] * PX_PER_M))
                cv2.circle(img, c, r, color, -1)
                cv2.circle(img, c, r, (30, 30, 30), 1)
                if p['shape'] == 'sphere':
                    cv2.circle(img, (c[0] - r // 3, c[1] - r // 3), max(1, r // 3), (255, 255, 255), -1)
            if 'id' in p:
                cv2.putText(img, f"#{p['id']}", (c[0] + 8, c[1] + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (60, 60, 60), 1)

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
        if odom:
            pose = odom['pose']['pose']
            x, y = self.from_map(pose['position']['x'], pose['position']['y'])
            self.robot(img, x, y, yaw_of(pose['orientation']), (0, 0, 200) if age > STALE_S else (200, 60, 0), False)
        cv2.putText(img, 'robot: fill=real(plant) ring=/odom  path=/plan  purple=/tracked_objects  grey=obstacle  blue=person',
                    (MARGIN - 25, self.size[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.34, (80, 80, 80), 1)
        return img


def _rect(x, y, yaw, sx, sy):
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + c * dx - s * dy, y + s * dx + c * dy) for dx, dy in
            ((-sx / 2, -sy / 2), (sx / 2, -sy / 2), (sx / 2, sy / 2), (-sx / 2, sy / 2))]


# ---------- 카메라 타일 (ROS가 실제로 낸 것 우선, 없으면 시뮬레이터 렌더)
def camera_tiles(sim, cam, view):
    color = cv2.imread(str(TOPIC_DIR / cam / 'color/image_raw/image.jpg'))
    depth = cv2.imread(str(TOPIC_DIR / cam / 'aligned_depth_to_color/image_raw/depth.png'), cv2.IMREAD_ANYDEPTH)
    source = 'ROS'
    if color is None or depth is None:
        if cam not in sim.renders:
            blank = np.zeros((TILE[1], TILE[0], 3), np.uint8)
            cv2.putText(blank, 'waiting (test-faker 0?)', (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
            return blank, blank, blank
        color, depth, _ = sim.renders[cam]
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

        self.map_label = tk.Label(left, cursor='hand2', takefocus=1)
        self.map_label.pack()
        self.map_label.bind('<ButtonPress-1>', self.on_map_press)
        self.map_label.bind('<B1-Motion>', self.on_map_drag)
        self.map_label.bind('<ButtonRelease-1>', self.on_map_release)

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
        tk.Button(col, text='사람 끼어들기', command=self.sim.spawn_person).pack(fill=tk.X)
        tk.Button(col, text='바닥 장애물 추가', command=self.sim.add_obstacle).pack(fill=tk.X)
        tk.Button(col, text='장애물 지우기', command=self.sim.clear_obstacles).pack(fill=tk.X)
        # faker를 다시 띄우면 /odom이 0부터 — 실제 로봇도 시작 칸으로 돌려 맞춘다
        tk.Button(col, text='로봇 시작 칸으로', command=lambda: self.sim.plant.reset(self.sim.spawn)).pack(fill=tk.X)
        col = tk.Frame(env)
        col.pack(side=tk.LEFT, padx=4)
        self.comm = tk.BooleanVar(value=False)
        tk.Checkbutton(col, text='OpenCR 통신 끊기', variable=self.comm,
                       command=lambda: setattr(self.sim.plant, 'comm_down', self.comm.get())).pack(anchor=tk.W)
        self.grab = tk.BooleanVar(value=False)
        tk.Checkbutton(col, text='팔 붙잡기(과전류)', variable=self.grab,
                       command=lambda: setattr(self.sim.plant, 'arm_grabbed', self.grab.get())).pack(anchor=tk.W)

        # 수동 조작 — 별도 창 (열면 수동 모드, 닫으면 자동 모드)
        man = tk.LabelFrame(left, text='수동 조작 (→ control/manual_control)')
        man.pack(fill=tk.X, pady=2)
        self.manual_button = tk.Button(man, text='수동 조작 창 열기', command=self.open_manual)
        self.manual_button.pack(side=tk.LEFT, padx=4, pady=2)
        self.estop = tk.BooleanVar(value=False)
        tk.Checkbutton(man, text='비상정지(/e_stop)', variable=self.estop, fg='#c62828',
                       command=lambda: send_bool('/e_stop', self.estop.get())).pack(side=tk.LEFT)
        self.manual_window = None

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

        texts = tk.Frame(right)
        texts.pack(fill=tk.BOTH, expand=True, pady=4)
        self.status = tk.Text(texts, width=60, height=12, font=('Menlo', 10))
        self.status.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.diag = tk.Text(texts, width=60, height=12, font=('Menlo', 10))
        self.diag.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        for t in (self.status, self.diag):
            t.tag_config('warn', foreground='#b26a00')
            t.tag_config('error', foreground='#c62828')
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
        self.tick()

    # --- 조작
    def on_background(self, name):
        self.sim.renderer.set_background(None if name == NO_BG else BG_DIR / name)

    def on_stream(self):
        self.sim.streaming = self.stream.get()
        if not self.sim.streaming:
            self.sim.stop_streaming()

    def open_manual(self):
        if self.manual_window is not None and self.manual_window.winfo_exists():
            self.manual_window.lift()
            return
        self.manual_window = ManualWindow(self.root, self.sim, on_close=lambda: setattr(self, 'manual_window', None))

    def on_map_press(self, e):
        x, y = self.view.world(e.x, e.y)
        with self.sim.lock:
            near = [p for p in self.sim.pucks + self.sim.props if math.dist((x, y), (p['x'], p['y'])) < 0.2]
        self.drag = min(near, key=lambda p: math.dist((x, y), (p['x'], p['y']))) if near else None

    def on_map_drag(self, e):
        if self.drag:
            with self.sim.lock:
                self.drag['x'], self.drag['y'] = (round(v, 3) for v in self.view.world(e.x, e.y))

    def on_map_release(self, _):
        if self.drag:
            self.sim.save_truth()
        self.drag = None

    def on_cloud_press(self, e, cam):
        self.cloud_press = (e.x, e.y, *self.cloud_view[cam])

    def on_cloud_drag(self, e, cam):
        x0, y0, yaw, pitch = self.cloud_press
        self.cloud_view[cam] = [yaw + (e.x - x0) * 0.01, max(-1.5, min(1.5, pitch - (e.y - y0) * 0.01))]

    def send_command(self, text=None):
        text = (text or self.entry.get()).strip()
        if not text:
            return
        send('/user_command', 'std_msgs/msg/String', {'data': text})
        self.sent.config(text=f'보냄: {text}  ({time.strftime("%H:%M:%S")}) — input_manager가 /user_command로 발행')
        self.entry.delete(0, tk.END)

    def tick(self):
        photos = [to_photo(self.view.render())]  # Tk가 이미지를 붙잡도록 보관
        self.map_label.config(image=photos[0])
        for cam, labels in self.tiles.items():
            for lab, img in zip(labels, camera_tiles(self.sim, cam, self.cloud_view[cam])):
                photos.append(to_photo(img))
                lab.config(image=photos[-1], width=TILE[0], height=TILE[1])
        self.photos = photos
        for widget, lines in ((self.status, status_lines(self.sim)), (self.diag, diagnostics_lines())):
            widget.delete('1.0', tk.END)
            for text, tag in lines:
                widget.insert(tk.END, text + '\n', (tag,) if tag else ())
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
