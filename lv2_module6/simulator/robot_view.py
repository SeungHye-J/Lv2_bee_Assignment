"""로봇 3D 뷰 — URDF(로봇이 내보내는 /robot_description)의 visual 도형을 관절값대로 그린다 (OpenCV, 별도 3D 라이브러리 없음)

- RobotModel: URDF의 link visual(box · cylinder · sphere, origin, material 색)과 관절(종류 · 축 · 한계)을 읽는다.
  자세는 scene.Kinematics 순기구학.
- OrbitView: 궤도 카메라(드래그로 회전 · 휠로 줌). 면을 먼 것부터 칠하는 단순 렌더(painter's algorithm) + 조명.
- 팔 뷰: 관절 원점에 마커를 그리고, 마커를 잡아 위아래로 끌면 그 관절 목표값이 바뀐다 (수동 조작 창).
"""
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import cv2
import numpy as np

import scene

MESH_DIR = Path(__file__).resolve().parent.parent / 'urdf' / 'meshes'  # URDF의 package://…/meshes/x.stl 이 가리키는 곳
MESH_CELL = 0.008  # 메시 간소화 격자 [m] — 꼭짓점을 이 격자로 뭉쳐 삼각형 수를 줄인다 (순수 파이썬 렌더가 감당할 만큼)
_mesh_cache = {}

LIGHT = np.array([0.4, 0.3, 0.85]) / np.linalg.norm([0.4, 0.3, 0.85])


def _box(sx, sy, sz):
    x, y, z = sx / 2, sy / 2, sz / 2
    v = np.array([[-x, -y, -z], [x, -y, -z], [x, y, -z], [-x, y, -z], [-x, -y, z], [x, -y, z], [x, y, z], [-x, y, z]])
    return [v[[0, 3, 2, 1]], v[[4, 5, 6, 7]], v[[0, 1, 5, 4]], v[[1, 2, 6, 5]], v[[2, 3, 7, 6]], v[[3, 0, 4, 7]]]


def _cylinder(r, h, n=14):
    a = np.linspace(0, 2 * math.pi, n, endpoint=False)
    ring = np.stack([r * np.cos(a), r * np.sin(a)], 1)
    bottom = np.column_stack([ring, np.full(n, -h / 2)])
    top = np.column_stack([ring, np.full(n, h / 2)])
    faces = [bottom[::-1], top]
    for i in range(n):
        j = (i + 1) % n
        faces.append(np.array([bottom[i], bottom[j], top[j], top[i]]))
    return faces


def _sphere(r, n=8):
    faces = []
    for i in range(n // 2):
        t0, t1 = math.pi * i / (n // 2) - math.pi / 2, math.pi * (i + 1) / (n // 2) - math.pi / 2
        for j in range(n):
            p0, p1 = 2 * math.pi * j / n, 2 * math.pi * (j + 1) / n
            pt = lambda t, p: [r * math.cos(t) * math.cos(p), r * math.cos(t) * math.sin(p), r * math.sin(t)]
            faces.append(np.array([pt(t0, p0), pt(t0, p1), pt(t1, p1), pt(t1, p0)]))
    return faces


def load_stl(name, scale):
    """바이너리 STL → (N,3,3) 삼각형. 꼭짓점을 격자(MESH_CELL)로 뭉쳐 간소화 · 같은 파일은 한 번만 읽는다"""
    key = (name, scale)
    if key not in _mesh_cache:
        data = np.fromfile(MESH_DIR / name, np.uint8)
        n = int(np.frombuffer(data[80:84], np.uint32)[0])
        rec = np.frombuffer(data[84:84 + 50 * n], np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')]))
        pts = rec['v'].astype(float).reshape(-1, 3) * scale
        _, inv = np.unique(np.round(pts / MESH_CELL).astype(np.int64), axis=0, return_inverse=True)
        inv = inv.reshape(-1)
        verts = np.zeros((inv.max() + 1, 3))
        np.add.at(verts, inv, pts)
        verts /= np.bincount(inv)[:, None]  # 뭉친 꼭짓점 = 격자 안 원래 꼭짓점의 평균
        idx = inv.reshape(-1, 3)
        idx = idx[(idx[:, 0] != idx[:, 1]) & (idx[:, 1] != idx[:, 2]) & (idx[:, 0] != idx[:, 2])]  # 찌그러진 삼각형 제거
        _, first = np.unique(np.sort(idx, axis=1), axis=0, return_index=True)  # 같은 삼각형 한 번만
        _mesh_cache[key] = verts[idx[np.sort(first)]]
    return _mesh_cache[key]


def primitive_faces(shape, a, h, b=None):
    """상태 뷰용 물체 원형 — a=반지름(원기둥·구) 또는 x폭(상자), h=높이, b=y폭(상자, 없으면 a)"""
    if shape == 'cylinder':
        return _cylinder(a, h)
    if shape == 'sphere':
        return _sphere(a)
    return _box(a, b if b else a, h)


class RobotModel:
    def __init__(self, urdf_xml):
        root = ET.fromstring(urdf_xml)
        self.kin = scene.Kinematics(urdf_xml)
        colors = {m.get('name'): m.find('color').get('rgba') for m in root.findall('material') if m.find('color') is not None}
        self.visuals = []  # (link, T_origin, faces_local, BGR)
        for link in root.findall('link'):
            for vis in link.findall('visual'):
                geom = vis.find('geometry')
                if geom is None or len(geom) == 0:
                    continue
                g = geom[0]
                if g.tag == 'box':
                    faces = _box(*[float(v) for v in g.get('size').split()])
                elif g.tag == 'cylinder':
                    faces = _cylinder(float(g.get('radius')), float(g.get('length')))
                elif g.tag == 'sphere':
                    faces = _sphere(float(g.get('radius')))
                elif g.tag == 'mesh':
                    faces = load_stl(Path(g.get('filename')).name, float(g.get('scale', '1').split()[0]))
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
                r, gg, b = ([float(v) for v in rgba.split()[:3]] if rgba else [0.7, 0.7, 0.7])
                self.visuals.append((link.get('name'), scene.transform(scene.rpy_matrix(*rpy), xyz), faces,
                                     (int(b * 255), int(gg * 255), int(r * 255))))
        self.joints = {}  # 움직이는 관절 이름 → (자식 링크, 종류, 축, 하한, 상한)
        for j in root.findall('joint'):
            if j.get('type') in ('revolute', 'continuous', 'prismatic') and j.find('mimic') is None:
                lim = j.find('limit')
                lo, hi = (float(lim.get('lower', -math.pi)), float(lim.get('upper', math.pi))) if lim is not None else (-math.pi, math.pi)
                axis = j.find('axis')
                self.joints[j.get('name')] = (j.find('child').get('link'), j.get('type'),
                                              [float(v) for v in axis.get('xyz').split()] if axis is not None else [1, 0, 0], lo, hi)

    def descendants(self, link):
        """link와 그 아래 링크 전부 (팔 뷰용)"""
        out, frontier = {link}, [link]
        while frontier:
            parent = frontier.pop()
            for child, j in self.kin.parent.items():
                if j['parent'] == parent and child not in out:
                    out.add(child)
                    frontier.append(child)
        return out

    def faces(self, q, links=None, tint=None):
        """월드(루트 링크) 기준 면 목록 [(꼭짓점 N×3, BGR)]"""
        out, cache = [], {}
        for link, T_o, faces, color in self.visuals:
            if links is not None and link not in links:
                continue
            if link not in cache:
                cache[link] = self.kin.pose(link, q)
            T = cache[link] @ T_o
            col = tint or color
            if isinstance(faces, np.ndarray):  # 메시: 삼각형 묶음 하나로 (render가 한꺼번에 처리)
                out.append((faces @ T[:3, :3].T + T[:3, 3], col))
                continue
            for f in faces:
                out.append((f @ T[:3, :3].T + T[:3, 3], col))
        return out

    def joint_frames(self, q):
        """관절 → (원점 xyz, 축 방향) 월드 기준"""
        out = {}
        for name, (child, _, axis, _, _) in self.joints.items():
            T = self.kin.pose(child, q)
            out[name] = (T[:3, 3], T[:3, :3] @ np.asarray(axis, float))
        return out


class OrbitView:
    def __init__(self, target=(0, 0, 0.2), dist=1.2, yaw=-2.3, pitch=0.45, fov_deg=45):
        self.target = np.array(target, float)
        self.dist, self.yaw, self.pitch, self.fov = dist, yaw, pitch, math.radians(fov_deg)

    def basis(self):
        cp, sp, cy, sy = math.cos(self.pitch), math.sin(self.pitch), math.cos(self.yaw), math.sin(self.yaw)
        eye = self.target + self.dist * np.array([cp * cy, cp * sy, sp])
        fwd = (self.target - eye) / np.linalg.norm(self.target - eye)
        right = np.cross(fwd, [0, 0, 1])
        right /= np.linalg.norm(right)
        up = np.cross(right, fwd)
        return eye, right, up, fwd

    def project(self, pts, size):
        eye, right, up, fwd = self.basis()
        rel = np.asarray(pts, float) - eye
        x, y, z = rel @ right, rel @ up, rel @ fwd
        f = size[1] / 2 / math.tan(self.fov / 2)
        with np.errstate(divide='ignore', invalid='ignore'):
            u = size[0] / 2 + f * x / z
            v = size[1] / 2 - f * y / z
        return np.stack([u, v], -1), z

    def render(self, faces, size, background=(40, 40, 46), grid=True):
        img = np.full((size[1], size[0], 3), background, np.uint8)
        if grid:  # 바닥 격자 (z=0, 0.1m)
            for k in np.arange(-1.0, 1.01, 0.1):
                for a, b in (([k, -1, 0], [k, 1, 0]), ([-1, k, 0], [1, k, 0])):
                    uv, z = self.project([a, b], size)
                    if (z > 0.05).all():
                        cv2.line(img, tuple(uv[0].astype(int)), tuple(uv[1].astype(int)), (70, 70, 78), 1, cv2.LINE_AA)
        eye = self.basis()[0]
        polys = []
        for verts, color in faces:
            if verts.ndim == 3:  # 삼각형 묶음 (N,3,3) — 뒷면 제거 · 투영을 한꺼번에
                n = np.cross(verts[:, 1] - verts[:, 0], verts[:, 2] - verts[:, 0])
                norm = np.linalg.norm(n, axis=1)
                n = n / np.maximum(norm, 1e-12)[:, None]
                ok = (norm > 1e-12) & (np.einsum('ij,ij->i', n, eye - verts.mean(1)) > 0)
                uv, z = self.project(verts.reshape(-1, 3), size)
                uv, z = uv.reshape(-1, 3, 2), z.reshape(-1, 3)
                ok &= (z >= 0.02).all(1)
                shade = 0.35 + 0.65 * np.maximum(0.0, n @ LIGHT)
                for i in np.flatnonzero(ok):
                    polys.append((float(z[i].mean()), uv[i].astype(np.int32), tuple(int(c * shade[i]) for c in color)))
                continue
            n = np.cross(verts[1] - verts[0], verts[2] - verts[0])
            norm = np.linalg.norm(n)
            if norm < 1e-12:
                continue
            n /= norm
            center = verts.mean(0)
            if n @ (eye - center) <= 0:  # 뒷면
                continue
            uv, z = self.project(verts, size)
            if (z < 0.02).any():
                continue
            shade = 0.35 + 0.65 * max(0.0, float(n @ LIGHT))
            polys.append((float(z.mean()), uv.astype(np.int32), tuple(int(c * shade) for c in color)))
        for _, uv, col in sorted(polys, key=lambda p: -p[0]):
            cv2.fillConvexPoly(img, uv, col, cv2.LINE_AA)
        return img

    def orbit(self, du, dv):
        self.yaw -= du * 0.01
        self.pitch = max(-0.2, min(1.5, self.pitch + dv * 0.01))

    def zoom(self, steps):
        self.dist = max(0.25, min(4.0, self.dist * (0.9 ** steps)))
