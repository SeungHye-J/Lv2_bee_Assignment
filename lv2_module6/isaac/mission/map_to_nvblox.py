"""2D 지도(map.pgm + map.yaml) → 실기 nvblox 지도 파일(nvblox.nvblx) — map:=saved로 바로 위치추정에 쓰려고.

  python3 isaac/mission/map_to_nvblox.py isaac/mission/map/scene_approach_matte ros2_ws/map/scene_approach_matte

nvblox.nvblx = nvblox save_map이 쓰는 SQLite (레이어마다 <층>_layer_metadata · <층>_layer_data(블록 index_x·y·z, data BLOB)).
  블록 0.4 m = 8×8×8 복셀(0.05 m) · 복셀 순서 [x][y][z] — 실기 저장 파일(map/kk)을 읽어 맞춘 형식
  tsdf      복셀 {float distance[m], float weight} — 바닥(z=0) + 벽(높이 WALL_H)을 합친 부호 거리, ±TRUNC로 자름
  esdf      복셀 {float sq_dist[복셀²], int3 parent, bool inside, observed, site} — 2D 모드라 블록 z=0의 슬라이스 한 장(z 칸 SLICE_Z)만
  freespace 복셀 {int64 last_occupied_ms, int64 duration_ms, bool high_conf} — 실기처럼 tsdf 블록마다 (dynamic mapping)
pgm 밖은 모름 칸(블록 없음). 지도 좌표 = map 프레임 = 로봇을 켠 자리 (make_map.py와 같음) — 실기도 같은 자리·방향에서 켤 것.
"""
import argparse
import os
import sqlite3
import time

import numpy as np
import yaml
from PIL import Image
from scipy.ndimage import distance_transform_edt

VOXEL = 0.05       # [m] config/nvblox.yaml voxel_size
N = 8              # 블록 한 변 복셀 수
BLOCK = VOXEL * N  # 0.4 m
TRUNC = 4 * VOXEL  # nvblox truncation_distance_vox 4 (실기 파일 최대 거리 0.2)
WEIGHT = 5.0       # 실기 파일 가중치 최대값
WALL_H = 0.6       # [m] 벽 높이 (esdf_slice_max_height 0.3 위까지)
Z_BLOCKS = (-1, 0, 1)  # 실기 파일과 같은 높이 범위 -0.4 ~ 0.8 m
SLICE_Z = 2        # esdf_slice_height 0.1 m → 블록 z=0의 z 칸 2
LAYERS = ('tsdf', 'esdf', 'occupancy', 'color', 'freespace', 'feature')

ESDF = np.dtype([('sq', '<f4'), ('par', '<i4', 3), ('inside', 'u1'), ('obs', 'u1'), ('site', 'u1'), ('pad', 'u1')])
FREE = np.dtype([('last', '<i8'), ('dur', '<i8'), ('hc', 'u1'), ('pad', 'u1', 7)])
assert ESDF.itemsize == 20 and FREE.itemsize == 24


def load_grid(d):
    """→ occ(bool, [행=y 아래→위, 열=x]), origin(x, y), res — map_server 규칙(trinary, negate 0)"""
    m = yaml.safe_load(open(os.path.join(d, 'map.yaml')))
    img = np.array(Image.open(os.path.join(d, m['image'])).convert('L'), np.float32)[::-1]  # 맨 아래 행 = origin y
    p = (255 - img) / 255.0 if not m.get('negate', 0) else img / 255.0
    occ = p > m['occupied_thresh']
    free = p < m['free_thresh']
    return occ, free, m['origin'][:2], m['resolution']


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('src', help='map.pgm · map.yaml 폴더')
    ap.add_argument('dst', help='nvblox.nvblx를 쓸 폴더')
    a = ap.parse_args()
    occ, free, (ox, oy), res = load_grid(a.src)
    known = occ | free
    h, w = occ.shape
    # 2D 부호 거리 [m]: 빈 칸 = 가장 가까운 벽까지(+), 벽 칸 = 가장 가까운 빈 칸까지(−)
    d_out, idx_out = distance_transform_edt(~occ, return_indices=True)
    d_in = distance_transform_edt(occ)
    sd2 = np.where(occ, -(d_in - 0.5), d_out - 0.5) * res

    # 복셀 중심 (x, y) 격자 — 지도가 덮는 블록 전체
    bx0, by0 = int(np.floor(ox / BLOCK)), int(np.floor(oy / BLOCK))
    bx1, by1 = int(np.floor((ox + w * res) / BLOCK)), int(np.floor((oy + h * res) / BLOCK))
    nx, ny = (bx1 - bx0 + 1) * N, (by1 - by0 + 1) * N
    cx = (bx0 * N + np.arange(nx) + 0.5) * VOXEL
    cy = (by0 * N + np.arange(ny) + 0.5) * VOXEL
    gi = np.floor((cx - ox) / res).astype(int)   # 지도 열
    gj = np.floor((cy - oy) / res).astype(int)   # 지도 행
    inx, iny = (gi >= 0) & (gi < w), (gj >= 0) & (gj < h)
    I, J = np.meshgrid(np.clip(gi, 0, w - 1), np.clip(gj, 0, h - 1), indexing='ij')  # [x, y]
    vk = known[J, I] & inx[:, None] & iny[None, :]
    vd = sd2[J, I]
    vocc = occ[J, I] & vk
    # 가장 가까운 벽 칸 (esdf parent) — 지도 칸 → 복셀 칸 차이 (해상도가 같다고 봄)
    pj, pi = idx_out[0][J, I], idx_out[1][J, I]
    par = np.stack([pi - I, pj - J], -1)

    os.makedirs(a.dst, exist_ok=True)
    out = os.path.join(a.dst, 'nvblox.nvblx')
    if os.path.exists(out):
        os.remove(out)
    db = sqlite3.connect(out)
    for L in LAYERS:
        db.execute(f'CREATE TABLE {L}_layer_metadata(param_name TEXT PRIMARY KEY UNIQUE NOT NULL,'
                   'value_string TEXT,value_int INT,value_float FLOAT)')
        db.execute(f'CREATE TABLE {L}_layer_data(index_x INT NOT NULL,index_y INT NOT NULL,index_z INT NOT NULL,'
                   'data BLOB,PRIMARY KEY(index_x, index_y, index_z))')
        db.execute(f'INSERT INTO {L}_layer_metadata VALUES (?,?,NULL,NULL)', ('type', f'{L}_layer'))
        db.execute(f'INSERT INTO {L}_layer_metadata VALUES (?,NULL,NULL,?)', ('block_size', BLOCK))

    now_ms = int(time.time() * 1000)
    n_t = n_e = 0
    for bx in range(nx // N):
        for by in range(ny // N):
            s = (slice(bx * N, bx * N + N), slice(by * N, by * N + N))
            k, d2 = vk[s], vd[s]
            if not k.any():
                continue
            for bz in Z_BLOCKS:
                z = (bz * N + np.arange(N) + 0.5) * VOXEL               # [z]
                D2, Z = d2[:, :, None], z[None, None, :]
                above = np.where(D2 > 0, np.hypot(np.maximum(D2, 0), Z - WALL_H), Z - WALL_H)
                d_wall = np.where(Z <= WALL_H, D2, above)
                d = np.clip(np.minimum(Z, d_wall), -TRUNC, TRUNC)       # 바닥 ∪ 벽 = min
                t = np.zeros((N, N, N, 2), np.float32)
                t[..., 0] = np.where(k[:, :, None], d, 0.0)
                t[..., 1] = np.where(k[:, :, None], WEIGHT, 0.0)
                db.execute('INSERT INTO tsdf_layer_data VALUES (?,?,?,?)', (bx0 + bx, by0 + by, bz, t.tobytes()))
                f = np.zeros((N, N, N), FREE)
                f['last'] = now_ms
                db.execute('INSERT INTO freespace_layer_data VALUES (?,?,?,?)', (bx0 + bx, by0 + by, bz, f.tobytes()))
                n_t += 1
            # esdf 슬라이스 (블록 z=0, z 칸 SLICE_Z) — 거리는 복셀 단위 제곱
            e = np.zeros((N, N, N), ESDF)
            sl = e[:, :, SLICE_Z]
            sl['sq'] = np.where(k, (np.abs(d2) / VOXEL + 0.5) ** 2, 0.0)   # 칸 중심 사이 거리로 되돌림
            sl['obs'] = k
            sl['inside'] = vocc[s]
            sl['site'] = vocc[s] & (np.abs(d2) <= 0.5 * res + 1e-6)
            sl['par'][..., :2] = np.where(k[..., None], par[s], 0)
            e[:, :, SLICE_Z] = sl
            db.execute('INSERT INTO esdf_layer_data VALUES (?,?,?,?)', (bx0 + bx, by0 + by, 0, e.tobytes()))
            n_e += 1
    db.commit()
    db.close()
    print(f'{out}: tsdf·freespace 블록 {n_t} · esdf 블록 {n_e} (지도 {w}x{h} · 벽 칸 {int(occ.sum())})')


if __name__ == '__main__':
    main()
