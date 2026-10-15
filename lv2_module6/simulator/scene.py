"""시뮬레이터 장면 — 미로 벽 · 퍽 · 바닥 · 360° 배경을 로봇 카메라 시점으로 렌더 (컬러 + 깊이)

- 렌더: 광선 추적(numpy). 카메라 optical 프레임(z 앞 · x 오른쪽 · y 아래)에서 픽셀마다 광선을 쏴서
    벽(축 정렬 상자) · 퍽(원기둥 · 상자 · 구) · 바닥(z=0) 중 가장 가까운 것, 없으면 배경 파노라마.
    깊이 = optical z [m] → 16비트 mm. MIN_DEPTH보다 가깝거나 MAX_DEPTH보다 멀거나 배경이면 0 (RealSense처럼 측정 안 됨)
- 카메라 자세: 로봇이 내보내는 URDF(/robot_description)로 순기구학 + /joint_states 관절값 + /odom 위치
    world(미로) ← odom(원점 = 미로 spawn) ← base_footprint ← … ← <카메라>_color_optical_frame
- 화각: HFOV_DEG — faker.yaml의 fake_camera hfov_deg와 같아야 ROS 쪽 camera_info·점구름이 맞는다.
- 자기 몸: URDF visual 상자를 광선 추적에 넣어 그린다 (뒤쪽 상판에 실은 퍽 확인용). 렌더 중인 카메라 자체 마운트 상자만 뺀다.
"""
import math
import random
import xml.etree.ElementTree as ET
from pathlib import Path

import cv2
import numpy as np

HFOV_DEG = 69.0
RENDER_W, RENDER_H = 320, 240          # 출력 크기 (fake_camera가 640×480으로 키운다)
SCALE = 0.5                            # 내부 렌더 배율 — 160×120으로 쏘고 320×240으로 확대 (약 4배 빠름)
IW, IH = int(RENDER_W * SCALE), int(RENDER_H * SCALE)
MIN_DEPTH, MAX_DEPTH = 0.2, 6.0         # D435 최소 측정 거리 약 0.2m
VIEW_RANGE = 4.5                        # ponytail: 이보다 먼 벽은 안 그림 (렌더 속도) — 넓은 코스면 늘릴 것
LIGHT = np.array([0.3, 0.5, 0.8]) / np.linalg.norm([0.3, 0.5, 0.8])
WALL_BGR = (204, 184, 166)              # 미로 판넬 (원본 model.sdf 색 0.65 0.72 0.8)
WALL_TILE_M = 0.5                        # 벽 텍스처 가로 반복 [m] (세로는 벽 높이로 늘이기)
PALETTE = {  # 퍽 색 (BGR) — 인지에서 구분하기 쉬운 원색 위주
    'red': (40, 40, 220), 'green': (60, 180, 60), 'blue': (200, 90, 30), 'yellow': (40, 220, 230),
    'orange': (30, 140, 250), 'purple': (170, 60, 140), 'white': (235, 235, 235), 'black': (40, 40, 40)}
SHAPES = ('cylinder', 'box', 'sphere')


# ---------- 좌표 변환
def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def axis_angle(axis, angle):
    k = np.asarray(axis, float) / np.linalg.norm(axis)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * K @ K


def transform(R=None, t=(0, 0, 0)):
    T = np.eye(4)
    if R is not None:
        T[:3, :3] = R
    T[:3, 3] = t
    return T


def quat_matrix(q):
    x, y, z, w = q['x'], q['y'], q['z'], q['w']
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class Kinematics:
    """URDF 문자열 → 링크 자세 (루트 링크 기준). revolute·continuous·prismatic·fixed, mimic 지원"""

    def __init__(self, urdf_xml):
        root = ET.fromstring(urdf_xml)
        self.parent = {}  # child link → joint
        for j in root.findall('joint'):  # 최상위만 (<ros2_control> 안의 joint는 다른 것)
            origin = j.find('origin')
            xyz = [float(v) for v in (origin.get('xyz', '0 0 0') if origin is not None else '0 0 0').split()]
            rpy = [float(v) for v in (origin.get('rpy', '0 0 0') if origin is not None else '0 0 0').split()]
            axis = j.find('axis')
            mimic = j.find('mimic')
            self.parent[j.find('child').get('link')] = {
                'name': j.get('name'), 'type': j.get('type'), 'parent': j.find('parent').get('link'),
                'origin': transform(rpy_matrix(*rpy), xyz),
                'axis': [float(v) for v in axis.get('xyz').split()] if axis is not None else [1, 0, 0],
                'mimic': (mimic.get('joint'), float(mimic.get('multiplier', 1)), float(mimic.get('offset', 0)))
                if mimic is not None else None}

    def pose(self, link, q):
        T = np.eye(4)
        while link in self.parent:
            j = self.parent[link]
            value = q.get(j['name'], 0.0)
            if j['mimic']:
                value = q.get(j['mimic'][0], 0.0) * j['mimic'][1] + j['mimic'][2]
            motion = np.eye(4)
            if j['type'] in ('revolute', 'continuous'):
                motion = transform(axis_angle(j['axis'], value))
            elif j['type'] == 'prismatic':
                motion = transform(t=np.asarray(j['axis']) * value)
            T = j['origin'] @ motion @ T
            link = j['parent']
        return T


# ---------- 퍽
def random_pucks(count, cells, spawn, rng=random):
    """cells: 놓을 수 있는 칸 중심 [(x, y)] (미로 좌표). spawn 칸은 피한다. 서로 0.15m 넘게 떨어뜨림"""
    pucks = []
    free = [c for c in cells if math.dist(c, spawn) > 0.3]
    tries = 0
    while len(pucks) < count and tries < 500:
        tries += 1
        cx, cy = rng.choice(free)
        x, y = cx + rng.uniform(-0.2, 0.2), cy + rng.uniform(-0.2, 0.2)
        if any(math.dist((x, y), (p['x'], p['y'])) < 0.15 for p in pucks):
            continue
        shape = rng.choice(SHAPES)
        color = rng.choice(list(PALETTE))
        # 파지 포락선: 닫힘 18mm < 퍽 < 열림 76mm → 지름 40~60mm (박스는 대각선 기준)
        size = {'cylinder': (rng.uniform(0.020, 0.030), rng.uniform(0.03, 0.05)),   # 반지름, 높이
                'box': (rng.uniform(0.040, 0.045), rng.uniform(0.03, 0.05)),        # 한 변, 높이
                'sphere': (rng.uniform(0.020, 0.030), 0.0)}[shape]                    # 반지름
        pucks.append({'id': len(pucks) + 1, 'shape': shape, 'color': color, 'x': round(x, 3), 'y': round(y, 3),
                      'yaw': round(rng.uniform(-math.pi, math.pi), 3), 'size': [round(v, 3) for v in size]})
    return pucks


def random_obstacle(cells, avoid, rng=random):
    """바닥 장애물 하나 — 칸 안 무작위, avoid [(x, y)]에서 0.35m 넘게 떨어진 곳 · 못 찾으면 None"""
    for _ in range(100):
        cx, cy = rng.choice(cells)
        x, y = cx + rng.uniform(-0.2, 0.2), cy + rng.uniform(-0.2, 0.2)
        if all(math.dist((x, y), a) > 0.35 for a in avoid):
            return {'kind': 'obstacle', 'shape': 'box', 'color': (130, 130, 130), 'x': round(x, 3), 'y': round(y, 3),
                    'yaw': round(rng.uniform(-math.pi, math.pi), 3), 'sx': round(rng.uniform(0.08, 0.2), 3),
                    'sy': round(rng.uniform(0.08, 0.2), 3), 'size': [0.1, round(rng.uniform(0.05, 0.2), 3)]}
    return None


def mover(x, y, heading, speed=0.35, life_s=12.0):
    """이동물체 — 원기둥 (반지름 0.2, 키 1.7), heading 방향으로 speed m/s, life_s 뒤 사라짐"""
    return {'kind': 'mover', 'shape': 'cylinder', 'color': (200, 120, 50), 'x': x, 'y': y, 'yaw': 0.0,
            'size': [0.2, 1.7], 'vx': speed * math.cos(heading), 'vy': speed * math.sin(heading), 'until': life_s}


TL_COLORS = {'red': (40, 40, 220), 'yellow': (40, 220, 230), 'green': (60, 180, 60),
               'off': (50, 50, 50)}
TL_CYCLE = [('green', 5.0), ('off', 1.0), ('yellow', 2.0), ('off', 1.0), ('red', 5.0), ('off', 1.0)]
TL_POLE_H = 0.39                           # 폴 높이 [m] — 벽(0.39m)과 같게
TL_LAMPS = (('red', 0.38), ('yellow', 0.30), ('green', 0.22))  # 위부터 빨·노·초
TL_LAMP_R = 0.028
TL_HEAD = (0.14, 0.14, 0.17, 0.43)         # 헤드박스 sx, sy, z0, z1
TL_STATE_KO = {'red': '빨강', 'yellow': '노랑', 'green': '초록', 'off': '꺼짐'}


def _tl_dim(rgb):
    return tuple(int(c * 0.25) for c in rgb)  # 꺼진 램프


def traffic_light(x, y, tid, rng=random):
    """신호등 — 폴 + 헤드박스 + 3개 램프(빨·노·초, 평소 꺼짐). cycle_idx·timer로 순환"""
    return {'kind': 'traffic_light', 'shape': 'box', 'color': TL_COLORS['off'],
            'x': round(x, 3), 'y': round(y, 3), 'yaw': 0.0, 'id': tid,
            'sx': 0.14, 'sy': 0.14, 'size': [0.14, 0.3],
            'state': 'off', 'cycle_idx': 5, 'timer': round(rng.uniform(0, 1.0), 2)}


def maze_cells(course):
    """spawn이 있는 절반의 칸 중심 (미로는 y=0 벽으로 위·아래가 막혀 있음). 칸 = 안쪽 기둥 격자 사이"""
    xs = sorted({round(x, 3) for x, _ in course['posts']})
    inner = [(round(x, 3), round(y, 3)) for x, y in course['posts'] if xs[0] < x < xs[-1]]
    count = {}
    for _, y in inner:
        count[y] = count.get(y, 0) + 1
    # 행 = 안쪽 기둥이 모든 열에 있는 y (세로 벽에만 있는 반 칸 기둥은 뺀다)
    rows = sorted(y for y, n in count.items() if n == max(count.values()))
    side = 1 if course['spawn'][1] > 0 else -1
    return [((x0 + x1) / 2, (y0 + y1) / 2) for x0, x1 in zip(xs, xs[1:]) for y0, y1 in zip(rows, rows[1:])
            if (y0 + y1) / 2 * side > 0]


class BodyModel:
    """URDF visual → 로봇 자기 몸 렌더용. 퍽 싣는 뒤쪽 상판까지 카메라에 보여야 해서 그린다.
    link별 (T_origin, 크기, BGR). cylinder·sphere는 맞춤 상자로 근사 (mesh는 건너뜀)"""

    def __init__(self, urdf_xml):
        root = ET.fromstring(urdf_xml)
        colors = {m.get('name'): m.find('color').get('rgba') for m in root.findall('material')
                  if m.find('color') is not None}
        self.boxes = []  # (link, T_origin 4x4, (sx, sy, sz), BGR)
        for link in root.findall('link'):
            # mesh 모양은 못 그리니 visual이 전부 mesh인 링크는 collision 도형(상자 · 원기둥 · 구)으로 대신한다
            vis_all = link.findall('visual')
            if vis_all and all(v.find('geometry') is not None and v.find('geometry')[0].tag == 'mesh' for v in vis_all):
                vis_all = [c for c in link.findall('collision') if c.find('geometry') is not None and c.find('geometry')[0].tag != 'mesh']
            for vis in vis_all:
                geom = vis.find('geometry')
                if geom is None or len(geom) == 0:
                    continue
                g = geom[0]
                if g.tag == 'box':
                    size = tuple(float(v) for v in g.get('size').split())
                elif g.tag == 'cylinder':
                    r, h = float(g.get('radius')), float(g.get('length'))
                    size = (2 * r, 2 * r, h)
                elif g.tag == 'sphere':
                    r = float(g.get('radius'))
                    size = (2 * r, 2 * r, 2 * r)
                else:
                    continue
                origin = vis.find('origin')
                xyz = [float(v) for v in (origin.get('xyz', '0 0 0') if origin is not None else '0 0 0').split()]
                rpy = [float(v) for v in (origin.get('rpy', '0 0 0') if origin is not None else '0 0 0').split()]
                mat = vis.find('material')
                rgba = None
                if mat is not None:
                    c = mat.find('color')
                    rgba = c.get('rgba') if c is not None else colors.get(mat.get('name'))
                r_, gg, b = ([float(v) for v in rgba.split()[:3]] if rgba else [0.6, 0.6, 0.6])
                self.boxes.append((link.get('name'), transform(rpy_matrix(*rpy), xyz), size,
                                   (int(b * 255), int(gg * 255), int(r_ * 255))))


# ---------- 렌더
class Renderer:
    def __init__(self, walls, background=None):
        w = np.asarray(walls, float)
        self.wall_min = w[:, :3] - w[:, 3:] / 2
        self.wall_max = w[:, :3] + w[:, 3:] / 2
        self.wall_xy = w[:, :2]
        self.wall_h = float((w[:, 2] + w[:, 5] / 2).max()) if len(w) else 0.39  # 벽 높이 (텍스처 세로 기준)
        self.wall_tex = None
        self.set_background(background)
        fx = IW / 2 / math.tan(math.radians(HFOV_DEG) / 2)
        u, v = np.meshgrid(np.arange(IW) + 0.5, np.arange(IH) + 0.5)
        self.rays_opt = np.stack([(u - IW / 2) / fx, (v - IH / 2) / fx, np.ones_like(u)], -1).reshape(-1, 3)

    def set_background(self, path):
        img = cv2.imread(str(path)) if path else None
        if img is None:  # 격자 방
            img = np.full((512, 1024, 3), 170, np.uint8)
            img[::32, :] = 120
            img[:, ::32] = 120
        self.pano = cv2.resize(img, (1024, 512))

    def set_wall(self, path):
        """벽 텍스처 (가로 0.5m 반복 · 세로 벽 높이). 없으면 단색"""
        tex = cv2.imread(str(path)) if path else None
        self.wall_tex = tex
        self.wall_name = Path(path).name if path else None

    def set_floor(self, path):
        """바닥 텍스처 (1타일 = 1m 반복). 없으면 격자무늬"""
        tex = cv2.imread(str(path)) if path else None
        self.floor_tex = tex
        self.floor_name = Path(path).name if path else None

    def render(self, T_world_cam, pucks, body=()):
        """→ (BGR uint8 H×W×3, depth uint16 mm H×W)
        body: [(T_world 4x4, sx, sy, sz, BGR)] 로봇 자기 몸 (관절값 반영된 월드 자세) — 임의 회전 상자"""
        R, o = T_world_cam[:3, :3], T_world_cam[:3, 3]
        D = (self.rays_opt @ R.T).astype(np.float32)  # 광선 방향 (optical z = 1 → t가 곧 깊이)
        o = o.astype(np.float32)
        n = len(D)
        best = np.full(n, np.inf)
        color = np.zeros((n, 3))

        def take(t, rgb, shade):  # t·shade: 광선마다
            hit = t < best
            best[hit] = t[hit]
            color[hit] = np.asarray(rgb, float)[None] * shade[hit, None]

        # 바닥 z=0 — 텍스처 있으면 1m 타일 샘플, 없으면 0.25m 체크무늬
        with np.errstate(divide='ignore', invalid='ignore'):
            t = np.where(D[:, 2] < -1e-6, -o[2] / D[:, 2], np.inf)
        p = o + D * np.where(np.isfinite(t), t, 0)[:, None]
        hit = t < best
        best[hit] = t[hit]
        if getattr(self, 'floor_tex', None) is not None:
            fh, fw = self.floor_tex.shape[:2]
            fu = np.clip(((p[:, 0] % 1.0) * fw).astype(int), 0, fw - 1)
            fv = np.clip(((p[:, 1] % 1.0) * fh).astype(int), 0, fh - 1)
            color[hit] = self.floor_tex[fv[hit], fu[hit]].astype(float)
        else:
            checker = ((np.floor(p[:, 0] / 0.25) + np.floor(p[:, 1] / 0.25)) % 2)[:, None]
            floor_rgb = 95 + 20 * checker
            color[hit] = np.repeat(floor_rgb, 3, 1)[hit]

        # 벽 (축 정렬 상자) — 카메라 앞쪽 VIEW_RANGE 안만, 묶음으로 slab 판정
        rel = self.wall_xy - o[:2]
        forward = R[:2, 2] / max(np.linalg.norm(R[:2, 2]), 1e-6)
        near = (np.linalg.norm(rel, axis=1) < VIEW_RANGE) & (rel @ forward > -0.6)
        lo, hi = self.wall_min[near].astype(np.float32), self.wall_max[near].astype(np.float32)
        with np.errstate(divide='ignore', invalid='ignore'):
            inv = 1.0 / D
        for s in range(0, len(lo), 24):
            t1 = (lo[None, s:s + 24] - o) * inv[:, None]
            t2 = (hi[None, s:s + 24] - o) * inv[:, None]
            tmin = np.nanmax(np.minimum(t1, t2), axis=2)
            tmax = np.nanmin(np.maximum(t1, t2), axis=2)
            face = np.argmax(np.minimum(t1, t2), axis=2)
            ok = (tmax >= tmin) & (tmin > 1e-4)
            tmin = np.where(ok, tmin, np.inf)
            k = np.argmin(tmin, axis=1)
            t = tmin[np.arange(n), k]
            normal = np.zeros((n, 3))
            normal[np.arange(n), face[np.arange(n), k]] = 1
            shade = 0.55 + 0.45 * np.abs(normal @ LIGHT)
            hitm = t < best
            best[hitm] = t[hitm]
            if getattr(self, 'wall_tex', None) is not None:
                ax = face[np.arange(n), k]
                ph = o + D * np.where(np.isfinite(t), t, 0)[:, None]
                uu = np.where(ax == 0, ph[:, 1] / WALL_TILE_M, ph[:, 0] / WALL_TILE_M)
                vv = np.where(ax == 2, ph[:, 1] / WALL_TILE_M, ph[:, 2] / self.wall_h)
                fh, fw = self.wall_tex.shape[:2]
                ui = np.clip((uu % 1.0 * fw).astype(int), 0, fw - 1)
                vi = np.clip(((1.0 - vv % 1.0) * fh).astype(int), 0, fh - 1)
                wcol = self.wall_tex[vi, ui].astype(float)
                color[hitm] = wcol[hitm] * shade[hitm, None]
            else:
                color[hitm] = np.asarray(WALL_BGR, float)[None] * shade[hitm, None]

        for puck in pucks:
            if puck.get('kind') == 'traffic_light':
                self._traffic_light(puck, o, D, take)
            else:
                self._puck(puck, o, D, take)

        for T, sx, sy, sz, rgb in body:  # 자기 몸 — 상자 로컬좌표로 돌려 slab 판정
            c = np.asarray(T[:3, 3], np.float32)
            if (c - o) @ R[:, 2] < -0.2:  # 렌즈 뒤는 어차피 안 보임
                continue
            Rb = np.asarray(T[:3, :3], np.float32)
            ol = (o - c) @ Rb
            Dl = D @ Rb
            lo = np.array([-sx / 2, -sy / 2, -sz / 2], np.float32)
            hi = np.array([sx / 2, sy / 2, sz / 2], np.float32)
            with np.errstate(divide='ignore', invalid='ignore'):
                t1, t2 = (lo - ol) / Dl, (hi - ol) / Dl
            tmin = np.nanmax(np.minimum(t1, t2), 1)
            tmax = np.nanmin(np.maximum(t1, t2), 1)
            face = np.argmax(np.minimum(t1, t2), 1)
            t = np.where((tmax >= tmin) & (tmin > 1e-4), tmin, np.inf)
            normal = np.zeros((len(D), 3))
            normal[np.arange(len(D)), face] = 1
            take(t, rgb, 0.45 + 0.55 * np.abs((normal @ Rb.T) @ LIGHT))

        depth = np.where((best >= MIN_DEPTH) & (best <= MAX_DEPTH), best * 1000, 0).astype(np.uint16)
        # 배경: 아무것도 안 맞은 광선 → 파노라마 (가운데 = world +x, 위 = +z)
        miss = ~np.isfinite(best)
        if miss.any():
            d = D[miss] / np.linalg.norm(D[miss], axis=1, keepdims=True)
            yaw = np.arctan2(d[:, 1], d[:, 0])
            pitch = np.arcsin(np.clip(d[:, 2], -1, 1))
            pu = ((0.5 - yaw / (2 * math.pi)) * self.pano.shape[1]).astype(int) % self.pano.shape[1]
            pv = np.clip(((0.5 - pitch / math.pi) * self.pano.shape[0]).astype(int), 0, self.pano.shape[0] - 1)
            color[miss] = self.pano[pv, pu]
        bgr = np.clip(color, 0, 255).astype(np.uint8).reshape(IH, IW, 3)
        if SCALE != 1.0:  # 출력 크기로 확대 (컬러 선형 · 깊이 최근접 — 값 보존)
            bgr = cv2.resize(bgr, (RENDER_W, RENDER_H), interpolation=cv2.INTER_LINEAR)
            depth = cv2.resize(depth.reshape(IH, IW), (RENDER_W, RENDER_H), interpolation=cv2.INTER_NEAREST)
        return bgr, depth.reshape(RENDER_H, RENDER_W)

    @staticmethod
    def _puck(puck, o, D, take):
        """퍽·장애물·이동물체 공용: shape cylinder(size=[반지름, 높이]) · sphere(size=[반지름]) ·
        box(size=[한 변, 높이], 직사각형이면 sx·sy). color = PALETTE 이름 또는 (B, G, R).
        z = 바닥에서 띄운 높이 (든 퍽)"""
        rgb = PALETTE.get(puck['color'], (128, 128, 128)) if isinstance(puck['color'], str) else puck['color']
        c = np.array([puck['x'], puck['y']])
        zo = float(puck.get('z', 0.0))
        r_or_side, height = puck['size']
        if puck['shape'] == 'sphere':
            center = np.array([c[0], c[1], r_or_side + zo])
            oc = o - center
            a = np.einsum('ij,ij->i', D, D)
            b = 2 * D @ oc
            disc = b * b - 4 * a * (oc @ oc - r_or_side ** 2)
            t = np.where(disc >= 0, (-b - np.sqrt(np.maximum(disc, 0))) / (2 * a), np.inf)
            t = np.where(t > 1e-4, t, np.inf)
            nrm = (o + D * np.where(np.isfinite(t), t, 0)[:, None] - center) / r_or_side
            take(t, rgb, 0.45 + 0.55 * np.clip(nrm @ LIGHT, 0, 1))
        elif puck['shape'] == 'cylinder':
            r = r_or_side
            dx, dy = D[:, 0], D[:, 1]
            ox, oy = o[0] - c[0], o[1] - c[1]
            a = dx * dx + dy * dy
            b = 2 * (ox * dx + oy * dy)
            disc = b * b - 4 * a * (ox * ox + oy * oy - r * r)
            with np.errstate(divide='ignore', invalid='ignore'):
                t_side = np.where(disc >= 0, (-b - np.sqrt(np.maximum(disc, 0))) / (2 * a), np.inf)
                z = o[2] + D[:, 2] * t_side
                t_side = np.where((t_side > 1e-4) & (z >= zo) & (z <= zo + height), t_side, np.inf)
                t_top = np.where(D[:, 2] < 0, (zo + height - o[2]) / D[:, 2], np.inf)
            ptop = o[:2] + D[:, :2] * np.where(np.isfinite(t_top), t_top, 0)[:, None]
            t_top = np.where((t_top > 1e-4) & (np.sum((ptop - c) ** 2, 1) <= r * r), t_top, np.inf)
            side_n = np.stack([ox + dx * np.where(np.isfinite(t_side), t_side, 0),
                               oy + dy * np.where(np.isfinite(t_side), t_side, 0), np.zeros_like(dx)], 1) / r
            take(t_side, rgb, 0.45 + 0.55 * np.clip(side_n @ LIGHT, 0, 1))
            take(t_top, rgb, np.full(len(D), 0.45 + 0.55 * LIGHT[2]))
        else:  # box — 퍽 좌표계로 돌려 slab 판정
            sx, sy = puck.get('sx', r_or_side), puck.get('sy', r_or_side)
            cyaw, syaw = math.cos(-puck['yaw']), math.sin(-puck['yaw'])
            Rz = np.array([[cyaw, -syaw, 0], [syaw, cyaw, 0], [0, 0, 1]])
            ol = Rz @ (o - np.array([c[0], c[1], 0]))
            Dl = D @ Rz.T
            lo, hi = np.array([-sx / 2, -sy / 2, zo]), np.array([sx / 2, sy / 2, zo + height])
            with np.errstate(divide='ignore', invalid='ignore'):
                t1, t2 = (lo - ol) / Dl, (hi - ol) / Dl
            tmin = np.nanmax(np.minimum(t1, t2), 1)
            tmax = np.nanmin(np.maximum(t1, t2), 1)
            face = np.argmax(np.minimum(t1, t2), 1)
            t = np.where((tmax >= tmin) & (tmin > 1e-4), tmin, np.inf)
            normal = np.zeros((len(D), 3))
            normal[np.arange(len(D)), face] = 1
            take(t, rgb, 0.45 + 0.55 * np.abs((normal @ Rz) @ LIGHT))

    @staticmethod
    @staticmethod
    def _tl_sphere(center, r, o, D, take, rgb, emissive):
        oc = o - center
        a = np.einsum('ij,ij->i', D, D)
        b = 2 * D @ oc
        disc = b * b - 4 * a * (oc @ oc - r * r)
        t = np.where(disc >= 0, (-b - np.sqrt(np.maximum(disc, 0))) / (2 * a), np.inf)
        t = np.where(t > 1e-4, t, np.inf)
        if emissive:
            take(t, rgb, np.ones(len(D)))  # 켜진 램프는 발광이라 음영 없이 밝게
        else:
            nrm = (o + D * np.where(np.isfinite(t), t, 0)[:, None] - center) / r
            take(t, rgb, 0.45 + 0.55 * np.clip(nrm @ LIGHT, 0, 1))

    def _traffic_light(self, tl, o, D, take):
        """신호등: 회색 폴 + 어두운 헤드박스 + 3개 램프(켜진 것만 밝게, 앞뒤 양면)"""
        cx, cy = tl['x'], tl['y']
        state = tl.get('state', 'off')
        r, h = 0.02, TL_POLE_H
        dx, dy = D[:, 0], D[:, 1]
        ox, oy = o[0] - cx, o[1] - cy
        a = dx * dx + dy * dy
        b = 2 * (ox * dx + oy * dy)
        disc = b * b - 4 * a * (ox * ox + oy * oy - r * r)
        with np.errstate(divide='ignore', invalid='ignore'):
            t_side = np.where(disc >= 0, (-b - np.sqrt(np.maximum(disc, 0))) / (2 * a), np.inf)
            z = o[2] + D[:, 2] * t_side
            t_side = np.where((t_side > 1e-4) & (z >= 0) & (z <= h), t_side, np.inf)
        side_n = np.stack([ox + dx * np.where(np.isfinite(t_side), t_side, 0),
                           oy + dy * np.where(np.isfinite(t_side), t_side, 0), np.zeros_like(dx)], 1) / r
        take(t_side, (70, 70, 70), 0.45 + 0.55 * np.clip(side_n @ LIGHT, 0, 1))
        hsx, hsy, hz0, hz1 = TL_HEAD
        lo = np.array([cx - hsx / 2, cy - hsy / 2, hz0], np.float32)
        hi = np.array([cx + hsx / 2, cy + hsy / 2, hz1], np.float32)
        with np.errstate(divide='ignore', invalid='ignore'):
            t1, t2 = (lo - o) / D, (hi - o) / D
        tmin = np.nanmax(np.minimum(t1, t2), 1)
        tmax = np.nanmin(np.maximum(t1, t2), 1)
        t = np.where((tmax >= tmin) & (tmin > 1e-4), tmin, np.inf)
        take(t, (40, 40, 40), np.full(len(D), 0.6))
        for name, lz in TL_LAMPS:
            base = np.asarray(TL_COLORS[name], float)
            rgb = tuple(int(v) for v in base) if state == name else _tl_dim(base)
            for side in (hsx / 2 - 0.01, -(hsx / 2 - 0.01)):
                self._tl_sphere(np.array([cx + side, cy, lz]), TL_LAMP_R, o, D, take, rgb, state == name)


def depth_colormap(depth_mm):
    """16비트 깊이 → 컬러 (가까움 빨강 · 멂 파랑, 0 = 검정)"""
    d = depth_mm.astype(np.float32)
    norm = np.clip((d - MIN_DEPTH * 1000) / ((MAX_DEPTH - MIN_DEPTH) * 1000), 0, 1)
    img = cv2.applyColorMap((255 - norm * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
    img[d == 0] = 0
    return img


def cloud_view(depth_mm, size, yaw, pitch, color=None, step=3):
    """깊이 영상 → 점구름을 (yaw, pitch)로 돌려 본 그림. 색은 영상(같은 크기의 BGR)이 있으면 그 색, 없으면 깊이"""
    h, w = depth_mm.shape
    fx = w / 2 / math.tan(math.radians(HFOV_DEG) / 2)
    v, u = np.mgrid[0:h:step, 0:w:step]
    z = depth_mm[::step, ::step].astype(np.float32) / 1000
    ok = z > 0
    pts = np.stack([(u - w / 2) * z / fx, (v - h / 2) * z / fx, z], -1)[ok]
    out = np.zeros((size[1], size[0], 3), np.uint8)
    if len(pts) == 0:
        cv2.putText(out, 'no depth', (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        return out
    cols = (color[::step, ::step][ok] if color is not None else depth_colormap((z[ok] * 1000).astype(np.uint16)[None])[0])
    pts = pts - np.median(pts, axis=0)
    rot = rpy_matrix(pitch, 0, 0) @ rpy_matrix(0, yaw, 0)
    p = pts @ rot.T
    scale = 0.45 * min(size) / max(np.percentile(np.abs(p[:, :2]), 95), 1e-3)
    order = np.argsort(-p[:, 2])  # 먼 점부터 → 가까운 점이 덮음
    px = (p[order, 0] * scale + size[0] / 2).astype(int)
    py = (p[order, 1] * scale + size[1] / 2).astype(int)
    keep = (px >= 0) & (px < size[0] - 1) & (py >= 0) & (py < size[1] - 1)
    for dx, dy in ((0, 0), (1, 0), (0, 1), (1, 1)):  # 2×2 점
        out[py[keep] + dy, px[keep] + dx] = cols[order][keep]
    return out


def backgrounds(folder):
    return sorted(Path(folder).glob('*.jpg'))
