#!/usr/bin/env python3
"""plan.yaml 실측 벽(arena_walls)으로 Nav2 지도(map.pgm + map.yaml)를 그린다 — 출발 자리(start)를 원점으로.

  python3 isaac/mission/plan_to_map.py ros2_ws/map/scene_sim                       # plan.yaml start_pose_arena 기준
  python3 isaac/mission/plan_to_map.py ros2_ws/map/start_b --start 3.670,0.950,3.14159   # 다른 자리에서 켤 때
  python3 isaac/mission/map_to_nvblox.py ros2_ws/map/start_b ros2_ws/map/start_b   # 그다음 실기 nvblox 지도

왜: 저장 지도의 map 프레임 = 로봇을 켠 자리. 출발 자리가 바뀌면 지도도 그 자리 기준으로 다시 그려야 한다
  (mission_manager는 bringup start_pose:=로 따라가지만 저장 지도는 못 따라감). 씬(scene_sim.usda)은 벽 묶음째만
  옮겨져 실측과 최대 ~11 cm 어긋나서, 벽은 plan.yaml 실측 값으로 직접 그린다 (시작 박스·장소 받침대는 make_map.py처럼 뺌).
그다음 bringup: map:=saved map_name:=<폴더 이름> start_pose:=<같은 x,y,yaw>
"""
import argparse
import math
import os

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
PLAN = os.path.join(HERE, '..', '..', 'ros2_ws', 'src', 'plan', 'config', 'plan.yaml')


def find(d, k):
    if isinstance(d, dict):
        for kk, v in d.items():
            if kk == k:
                return v
            r = find(v, k)
            if r is not None:
                return r
    return None


def seg_dist(px, py, x0, y0, x1, y1):
    """점(배열)에서 선분까지 거리"""
    dx, dy = x1 - x0, y1 - y0
    t = np.clip(((px - x0) * dx + (py - y0) * dy) / max(dx * dx + dy * dy, 1e-12), 0, 1)
    return np.hypot(px - (x0 + t * dx), py - (y0 + t * dy))


def make(walls, start, res=0.05, half=0.035, margin=0.3):
    """arena 벽 선분 [N,4] · 출발 자세 (x, y, yaw) → (점유 격자[행 0 = y 아래], 원점 x, 원점 y)"""
    sx, sy, st = start
    c, s = math.cos(st), math.sin(st)
    a = np.asarray(walls, float).reshape(-1, 4)
    seg = np.c_[c * (a[:, 0] - sx) + s * (a[:, 1] - sy), -s * (a[:, 0] - sx) + c * (a[:, 1] - sy),
                c * (a[:, 2] - sx) + s * (a[:, 3] - sy), -s * (a[:, 2] - sx) + c * (a[:, 3] - sy)]
    xs, ys = np.r_[seg[:, 0], seg[:, 2]], np.r_[seg[:, 1], seg[:, 3]]
    ox = math.floor((xs.min() - margin) / res) * res
    oy = math.floor((ys.min() - margin) / res) * res
    w = int(math.ceil((xs.max() + margin - ox) / res))
    h = int(math.ceil((ys.max() + margin - oy) / res))
    yy, xx = np.mgrid[0:h, 0:w]
    cx, cy = ox + (xx + 0.5) * res, oy + (yy + 0.5) * res
    d = np.full((h, w), np.inf)
    for x0, y0, x1, y1 in seg:
        d = np.minimum(d, seg_dist(cx, cy, x0, y0, x1, y1))
    return d <= half, ox, oy


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('out', help='map.pgm · map.yaml을 쓸 폴더')
    ap.add_argument('--start', help='출발 자세 arena "x,y,yaw" (비면 plan.yaml start_pose_arena)')
    ap.add_argument('--plan', default=PLAN)
    ap.add_argument('--res', type=float, default=0.05, help='[m/칸] — Nav2 costmap · nvblox voxel과 같게')
    ap.add_argument('--half', type=float, default=0.035, help='[m] 벽 반두께 (씬에서 그린 예전 지도의 벽 면적 ÷ 길이 ≈ 0.034)')
    a = ap.parse_args()
    p = yaml.safe_load(open(a.plan))
    start = [float(v) for v in a.start.split(',')] if a.start else [float(v) for v in find(p, 'start_pose_arena')]
    occ, ox, oy = make(find(p, 'arena_walls'), start, a.res, a.half)
    os.makedirs(a.out, exist_ok=True)
    h, w = occ.shape
    img = np.where(np.flipud(occ), 0, 254).astype(np.uint8)   # pgm 행 0 = 위
    with open(os.path.join(a.out, 'map.pgm'), 'wb') as f:
        f.write(b'P5\n# plan_to_map.py: plan.yaml arena_walls\n%d %d\n255\n' % (w, h) + img.tobytes())
    with open(os.path.join(a.out, 'map.yaml'), 'w') as f:
        f.write(f'# isaac/mission/plan_to_map.py — plan.yaml arena_walls · map 원점 = 출발 자세 arena {start}\n'
                f'image: map.pgm\nmode: trinary\nresolution: {a.res}\norigin: [{ox:.3f}, {oy:.3f}, 0.0]\n'
                f'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
    print(f'{a.out}: {w}x{h} · 원점 ({ox:.3f}, {oy:.3f}) · 벽 칸 {int(occ.sum())} · 출발 {start}')


if __name__ == '__main__':
    # 점검: 출발 자리에서 본 바깥 벽 — arena (0,0)-(4.404,0) 아래 벽이, 출발(0.734,0.95,π)이면 map에서 y=+0.95 줄
    o, x0, y0 = make([0, 0, 4.404, 0], [0.734, 0.950, math.pi], half=0.03)
    r = int((0.95 - y0) / 0.05)
    assert o[r].any() and not o[r + 3].any(), '좌표 변환 점검 실패'
    main()
