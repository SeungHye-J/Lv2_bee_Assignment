"""움직임 정답표(답지)를 실측 도면으로 계산 (11-16) — 사람이 몰지 않고 plan.yaml 벽·상자·금지 구역에서 구간마다 경로를 구해
skill_navigate가 읽는 <이름>_NN_<구간>.motion.csv(+ .txt)로 씀. 녹화한 답지와 같은 형식이라 그대로 바꿔 끼움.

  구간 = 미션 주행 순서(출발 → survey_order 상자 → build_location_order 장소 → 출발). 구간 시작 자세 = 앞 상자 다가설 자리에서
  상자를 정면으로 봄 (AUTO가 approach exit으로 물러난 자세) · 끝 = 다음 상자 다가설 자리에서 상자를 봄 (mission_manager go_near와 같은 계산)
  경로 = 5 cm 격자 A*: 벽·상자·금지 구역에서 R(로봇 반폭 0.151 + 여유) 안은 못 감 · 벽에서 PREF m 안은 비용을 더해 통로·문 가운데로
  → 꺾이는 점만 (Douglas-Peucker) → 회전·직진 표. 실행: python3 route_tables.py [출력 폴더] (기본 ~/.ros/routes) — numpy · PyYAML
"""
import heapq, math, os, sys
from pathlib import Path

import numpy as np
import yaml

PLAN = Path(__file__).resolve().parents[2] / 'ros2_ws' / 'src' / 'plan' / 'config' / 'plan.yaml'
P = yaml.safe_load(PLAN.read_text())['mission_manager']['ros__parameters']
RES = 0.05
R = 0.151 + 0.06      # [m] 못 가는 거리 (로봇 반폭 + 여유)
PREF = 0.45           # [m] 벽에서 이 안은 비용을 더함 (통로·문 가운데로) — 11-16: 0.35면 벽 끝을 안쪽으로 끼고 돌다 collision_monitor 정지
W = P['arena_walls']; SEG = [W[i:i + 4] for i in range(0, len(W), 4)]
KO = P.get('keepout_zones', []); HALF = P['box_half']; TURN = P['nav_turn_m']; STAND = P['standoff']
SB = [(P['start_boxes'][i], P['start_boxes'][i + 1]) for i in range(0, len(P['start_boxes']), 2)]
LOC = [(P['locations'][i], P['locations'][i + 1]) for i in range(0, len(P['locations']), 2)]
START = tuple(P['start_pose_arena'])
NRM = {'+x': (1, 0), '-x': (-1, 0), '+y': (0, 1), '-y': (0, -1)}


def segd(px, py, s):
    ax, ay, bx, by = s; dx, dy = bx - ax, by - ay; L = dx * dx + dy * dy
    t = np.clip(((px - ax) * dx + (py - ay) * dy) / L, 0, 1) if L else 0
    return np.hypot(px - ax - t * dx, py - ay - t * dy)


def wall_dist(x, y):
    return min(float(segd(np.array(x), np.array(y), s)) for s in SEG)


def in_keepout(x, y):
    return any(min(KO[i], KO[i + 2]) <= x <= max(KO[i], KO[i + 2]) and min(KO[i + 1], KO[i + 3]) <= y <= max(KO[i + 1], KO[i + 3])
               for i in range(0, len(KO), 4))


def stand_pose(c, sides):
    """go_near와 같게: 첫 면 · '@d'면 그 거리 · nav_turn_m까지 물림(벽 여유·금지 구역에서 멈춤) → (x, y, 상자를 보는 yaw)"""
    first = sides.split()[0]; nx, ny = NRM[first[:2]]
    d = float(first[3:]) if '@' in first else HALF + STAND
    clear = min(0.32, wall_dist(c[0] + nx * d, c[1] + ny * d)); dd = d + 0.02
    while dd <= TURN + 1e-6:
        x, y = c[0] + nx * dd, c[1] + ny * dd
        if wall_dist(x, y) < clear or in_keepout(x, y): break
        d = dd; dd += 0.02
    return (c[0] + nx * d, c[1] + ny * d, math.atan2(-ny, -nx))


# ---------- 격자: 장애물까지 거리 D
nx_, ny_ = int(4.404 / RES) + 1, int(6.304 / RES) + 1
X, Y = np.meshgrid(np.arange(nx_) * RES, np.arange(ny_) * RES, indexing='ij')
D = np.min([segd(X, Y, s) for s in SEG], axis=0)
for cx, cy in SB + LOC:
    D = np.minimum(D, np.hypot(np.maximum(abs(X - cx) - HALF, 0), np.maximum(abs(Y - cy) - HALF, 0)))
for i in range(0, len(KO), 4):
    x0, x1 = sorted((KO[i], KO[i + 2])); y0, y1 = sorted((KO[i + 1], KO[i + 3]))
    D[(X >= x0) & (X <= x1) & (Y >= y0) & (Y <= y1)] = 0.0
COST = 1.0 + 4.0 * np.clip((PREF - D) / PREF, 0, 1) ** 2   # 벽 가까울수록 비쌈


def cell(p):
    return (int(round(p[0] / RES)), int(round(p[1] / RES)))


def astar(a, b):
    s, g = cell(a), cell(b)
    free = D > R
    free[s] = free[g] = True  # 다가설 자리는 상자 앞이라 R 안일 수 있음 — 시작·끝 칸은 허용
    pq = [(0.0, 0.0, s)]; best = {s: 0.0}; prev = {}
    nb = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]
    while pq:
        _, c, u = heapq.heappop(pq)
        if u == g: break
        if c > best.get(u, 1e18): continue
        for dx, dy in nb:
            v = (u[0] + dx, u[1] + dy)
            if not (0 <= v[0] < nx_ and 0 <= v[1] < ny_) or not free[v]: continue
            nc = c + math.hypot(dx, dy) * 0.5 * (COST[u] + COST[v])
            if nc < best.get(v, 1e18):
                best[v] = nc; prev[v] = u
                heapq.heappush(pq, (nc + math.hypot(v[0] - g[0], v[1] - g[1]), nc, v))
    if g not in prev and g != s: return None
    path = [g]
    while path[-1] != s: path.append(prev[path[-1]])
    pts = [(c[0] * RES, c[1] * RES) for c in reversed(path)]
    pts[0], pts[-1] = a[:2], b[:2]
    return pts


def simplify(p, tol):
    if len(p) < 3: return list(p)
    a, b = np.array(p[0]), np.array(p[-1]); d = b - a; L = np.hypot(*d) or 1e-9
    dist = [abs(d[0] * (q[1] - a[1]) - d[1] * (q[0] - a[0])) / L for q in p]
    k = int(np.argmax(dist))
    return simplify(p[:k + 1], tol)[:-1] + simplify(p[k:], tol) if dist[k] > tol else [p[0], p[-1]]


def push_corners(c, r_search=0.15):
    """꺾이는 점마다 둘레 r_search 안에서 벽까지 거리(D)가 가장 큰 칸으로 옮김 — 앞뒤 직선도 R 밖일 때만.
    벽 끝을 왼쪽에 끼고 돌 때 몸 앞 왼쪽이 벽 끝 1~2 cm까지 붙어 정지했음 (시뮬 정지 71번이 로봇 기준 앞 0.12 · 왼쪽 0.08~0.16)"""
    out = list(c)
    k_r = int(round(r_search / RES))
    def clear_line(a, b):
        n = max(2, int(math.hypot(b[0] - a[0], b[1] - a[1]) / (RES / 2)))
        return all(D[cell((a[0] + (b[0] - a[0]) * t / n, a[1] + (b[1] - a[1]) * t / n))] > R for t in range(n + 1))
    for i in range(1, len(out) - 1):
        ci, cj = cell(out[i]); best = (D[ci, cj], out[i])
        for di in range(-k_r, k_r + 1):
            for dj in range(-k_r, k_r + 1):
                ni, nj = ci + di, cj + dj
                if not (0 <= ni < nx_ and 0 <= nj < ny_) or math.hypot(di, dj) > k_r: continue
                q = (ni * RES, nj * RES)
                if D[ni, nj] > best[0] + 0.01 and clear_line(out[i - 1], q) and clear_line(q, out[i + 1]):
                    best = (D[ni, nj], q)
        out[i] = best[1]
    return out


def moves(pts, yaw0, yaw1):
    out, head = [], yaw0
    def turn(a, b):
        t = math.degrees(math.remainder(b - a, 2 * math.pi))
        if abs(t) >= 3: out.append(('turn', round(t, 1)))
    for a, b in zip(pts, pts[1:]):
        L = math.hypot(b[0] - a[0], b[1] - a[1])
        if L < 0.03: continue
        h = math.atan2(b[1] - a[1], b[0] - a[0]); turn(head, h); out.append(('forward', round(L, 3))); head = h
    turn(head, yaw1)
    return out


def main(out_dir):
    stops = [('start', (START[0], START[1], START[2]))]
    stops += [(f'S{i}', stand_pose(SB[i - 1], P['start_box_sides'][i - 1])) for i in P['survey_order']]
    stops += [(f'L{i}', stand_pose(LOC[i - 1], P['location_box_sides'][i - 1])) for i in P['build_location_order']]
    stops += [('start', (START[0], START[1], START[2]))]
    out_dir.mkdir(parents=True, exist_ok=True)
    legs = list(zip(stops, stops[1:]))
    # 상자마다 출발로 돌아가는 구간도 (상자로 가기 시험 · 중간 실패 뒤 복귀 — 11-16 실기 S4_start 없어 Nav2로 가다 걸림)
    home = stops[0]
    legs += [(s, home) for s in stops[1:-1] if (s, home) not in legs]
    for k, ((na, a), (nb, b)) in enumerate(legs, 1):
        pts = astar(a, b)
        if pts is None: print(f'{k:2d} {na}_{nb}: 경로 없음'); continue
        corners = push_corners(simplify(pts, 0.05))
        mv = moves(corners, a[2], b[2])
        stem = out_dir / f'auto_{k:02d}_{na}_{nb}'
        L = sum(v for t, v in mv if t == 'forward')
        Path(f'{stem}.motion.csv').write_text(f'# auto {na}_{nb} · plan.yaml 실측 벽 A* (R {R} · PREF {PREF}) · 직진 {L:.2f} m · turn = 도 (왼쪽 +) · forward = m\n'
                                             'action,value\n' + ''.join(f'{t},{v}\n' for t, v in mv))
        Path(f'{stem}.txt').write_text(f'# auto {na}_{nb} · arena x y (꺾이는 점)\n' + ''.join(f'{x:.3f} {y:.3f}\n' for x, y in corners))
        dmin = min(float(D[cell(q)]) for q in corners[1:-1]) if len(corners) > 2 else 0
        print(f'{k:2d} {na}_{nb}: 직진 {L:.2f} m · 꺾임 {len(corners) - 2} · 벽까지 최소 {dmin:.2f} m · '
              + ' → '.join(f'{"직" if t == "forward" else "회"}{v:+g}' for t, v in mv))


def stops():
    st = [('start', (START[0], START[1], START[2]))]
    st += [(f'S{i}', stand_pose(SB[i - 1], P['start_box_sides'][i - 1])) for i in P['survey_order']]
    st += [(f'L{i}', stand_pose(LOC[i - 1], P['location_box_sides'][i - 1])) for i in P['build_location_order']]
    return st


if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == '--stops':  # 구간 시험용: 이름 x y yaw (다가설 자리 · 상자를 봄)
        for n, (x, y, t) in stops(): print(n, f'{x:.3f}', f'{y:.3f}', f'{t:.5f}')
        sys.exit(0)
    main(Path(sys.argv[1]) if len(sys.argv) > 1 else Path(os.path.expanduser('~/.ros/routes')))
