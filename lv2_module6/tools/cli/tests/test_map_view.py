#!/usr/bin/env python3
"""map_view 단위 시험 — ROS 없이: python3 tools/cli/tests/test_map_view.py"""
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))   # tools/cli
from ros.fake.FakeRos import FakeRos  # noqa: E402

FakeRos.install()
from op.views.map import FREE, OCC, OUT, MapGrid, MapView, Viewport  # noqa: E402

# 8×8 빈 지도에 벽 한 칸 — 아무리 축소해도 벽이 안 사라져야 한다 (벽 우선 피라미드)
data = [0] * 64
data[3 * 8 + 5] = 100
g = MapGrid(8, 8, 0.1, -0.4, -0.4, data)
assert g.code(0, -0.4 + 5.5 * 0.1, -0.4 + 3.5 * 0.1) == OCC
assert g.code(0, 0.0, 0.0) == FREE and g.code(0, 9.0, 9.0) == OUT
assert all(g.code(k, -0.4 + 5.5 * 0.1, -0.4 + 3.5 * 0.1) == OCC for k in (1, 2, 3)), '축소해도 벽이 남아야 함'
assert g.level(3)[:2] == (1, 1)

# 전체 보기 — 지도가 화면에 들어옴 · 확대는 중심(화면 한가운데)이 그대로
vp = Viewport(g)
vp.fit(40, 20)
assert vp.fits(40, 20)
c0 = (vp.cx, vp.cy)
assert vp.zoom(+1) and (vp.cx, vp.cy) == c0
vp.k = -2
assert not vp.zoom(+1), '확대 한계'
vp.pan(2, 0)
assert abs(vp.cx - (c0[0] + 2 * vp.scale)) < 1e-9

# 그리기 — 글자 모드: 줄 수·폭이 맞고, goal 번호·십자선·로봇이 보임
view = MapView(g, color=False)
view.vp.fit(30, 20)
view.robot = (0.2, 0.2)
view.goals = [{'x': -0.3, 'y': -0.3, 'yaw_deg': 0}]
lines = view.render(30, 10)
assert len(lines) == 10 and all(len(l) == 30 for l in lines), (len(lines), [len(l) for l in lines])
txt = ''.join(lines)
assert '1' in txt and '+' in txt and '@' in txt and '#' in txt

# 로봇 따라가기 — 한 화면에 다 보이면 지도 고정, 확대해서 안 보이면 로봇이 중심
view.following = True
view.render(30, 10)
assert (view.vp.cx, view.vp.cy) == (0.0, 0.0), '지도가 다 보이면 지도 가운데'
view.vp.k = -2
view.robot = (0.3, -0.25)
view.render(30, 10)
assert (view.vp.cx, view.vp.cy) == (0.3, -0.25), '확대하면 로봇이 중심'
# 방향키로 밀면 따라가기가 풀리고 · r이면 다시 로봇 중심
assert view.on_key('left') and not view.following
cx = view.vp.cx
view.robot = (0.35, -0.25)
view.render(30, 10)
assert view.vp.cx == cx, '자유 이동 중엔 로봇을 안 따라감'
assert view.on_key('r') and view.following
view.render(30, 10)
assert (view.vp.cx, view.vp.cy) == (0.35, -0.25)
assert not view.on_key('x')   # 모르는 키는 안 먹음

# 방향선 — 찍는 모드(show_yaw)에서만 십자선에서 yaw 방향으로 선이 나감 (글자 모드는 ':')
view2 = MapView(g, color=False)
view2.vp.fit(30, 20)
view2.yaw = 0.0
assert ':' not in ''.join(view2.render(30, 10))
view2.show_yaw = True
assert ':' in ''.join(view2.render(30, 10))
rows0 = view2.render(30, 10)
view2.yaw = 90.0
assert view2.render(30, 10) != rows0, '방향이 바뀌면 선도 바뀜'

# 색 모드 — 방향선은 점자(⠀~⣿)로 가늘게 · 십자선은 분홍 바탕 흰 +
import re  # noqa: E402
cv = MapView(g, color=True)
cv.vp.fit(30, 20)
cv.show_yaw = True
cv.yaw = 0.0
cell_txt = re.sub(r'\x1b\[[0-9;]*m', '', ''.join(cv.render(30, 10)))
assert any(0x2800 < ord(ch) <= 0x28FF for ch in cell_txt), '방향선(점자)이 있어야 함'
assert '+' in cell_txt


# M · O · 로봇의 방향 화살표 — 색 모드에서 점자(U+2800~28FF)로 그려지고, 방향이 바뀌면 다른 칸에 그려진다
def braille_cells(view, cols=40, rows=20):
    out = view.render(cols, rows)
    return {(c, r) for r, line in enumerate(out) for c, ch in enumerate(_strip(line)) if 0x2800 <= ord(ch) <= 0x28FF}


def _strip(line):
    import re
    return re.sub(r'\x1b\[[0-9;]*m', '', line)


big = MapGrid(40, 40, 0.1, -2.0, -2.0, [0] * 1600)
cv = MapView(big, color=True)
cv.vp.fit(40, 40)
cv.show_frames = True
base = braille_cells(cv)                       # M 의 방향선(0°)만 있을 때
cv.odom, cv.odom_yaw = (0.8, 0.5), 0.0
east = braille_cells(cv)
cv.odom_yaw = 90.0
north = braille_cells(cv)
assert base and len(east) > len(base), 'O 방향선이 그려져야 함'
assert east != north, 'odom 방향이 바뀌면 화살표가 다른 칸에 그려져야 함'
cv.odom_yaw = None
assert braille_cells(cv) == base, 'odom 방향을 모르면 O 화살표는 안 그림'
cv.robot, cv.robot_yaw = (-0.6, -0.4), 180.0
assert len(braille_cells(cv)) > len(base), '로봇 진행 방향 화살표'
cv.show_frames = False
assert not braille_cells(cv), 'show_frames 가 꺼지면 방향 화살표도 없음'
print('PASS test_map_view')


# Nav2 경로 겹침(p) — 점선(:)이 늘고 끝점 E가 보임, 경로가 없으면 그대로
view = MapView(g, color=False)
view.vp.fit(30, 20)
base = ''.join(view.render(30, 10))
view.plans = [([(-0.3, -0.3), (0.0, 0.0), (0.3, 0.3)], (60, 230, 200))]
with_plan = ''.join(view.render(30, 10))
assert 'G' in with_plan and with_plan.count(':') > base.count(':'), '경로가 그려져야 함'
view.plans = []
assert ''.join(view.render(30, 10)) == base
print('map_view plan overlay ok')

# 십자선·방향 화살표가 지도 밖에 있어도 검은 네모(배경 48;2;0;0;0)를 칠하지 않는다 — 터미널 바탕 그대로
view = MapView(g, color=True)
view.vp.fit(30, 20)
view.vp.cy += 5.0        # 십자선을 지도 위쪽 밖으로
view.following = False
view.show_yaw = True
out = '\n'.join(view.render(30, 10))
assert '\033[48;2;0;0;0m' not in out, '지도 밖에 검은 배경이 칠해짐'
print('map_view off-map ok')

# 화살표·방향선·글자 밑 배경 — 위·아래 칸 중 한쪽이라도 벽이면 벽 색 (회색으로 지워지지 않음)
from op.views.map.map_view import RGB, _cell_bg  # noqa: E402
assert _cell_bg(OCC, FREE, RGB[OCC], RGB[FREE]) == RGB[OCC]
assert _cell_bg(FREE, OCC, RGB[FREE], RGB[OCC]) == RGB[OCC]
assert _cell_bg(FREE, FREE, RGB[FREE], RGB[FREE]) == RGB[FREE]
assert _cell_bg(OUT, OUT, None, None) is None, '지도 밖은 칠하지 않음'
print('map_view wall under arrow ok')

# 경유점 — 주황 번호로 찍힘 · 경로선 색은 밝은 회색 지도(205,205,210)와 구분되는 색 (흰색이면 안 보임)
from op.views.map.map_view import LOCAL_PLAN_RGB, PLAN_RGB, WAYPOINT_RGB  # noqa: E402
for rgb in (PLAN_RGB, LOCAL_PLAN_RGB, WAYPOINT_RGB):
    assert max(abs(a - b) for a, b in zip(rgb, (205, 205, 210))) > 80, rgb
view = MapView(g, color=True)
view.vp.fit(30, 20)
view.waypoints = [(0.2, 0.2)]   # (0,0)은 M 글자가 덮음
assert '\033[1;38;2;255;110;0m' in '\n'.join(view.render(30, 10)), '경유점이 주황으로 찍혀야 함'
print('map_view waypoint ok')

# 1/4 블록 지도(기본) — 빈틈 없는 블록 · 벽이 한 칸이라도 있으면 앞색은 벽 · costmap을 켜면 반블록으로 자동 전환
from op.views.map.map_view import QUAD, QUAD_BIT, _quad_cell  # noqa: E402
sub4 = [((dx, dy), FREE) for dy in (0, 1) for dx in (0, 1)]
assert _quad_cell(sub4)[0] == 0 and _quad_cell(sub4)[2] == RGB[FREE], '한 종류면 배경만'
w4 = [((dx, dy), OCC if (dx, dy) == (1, 0) else FREE) for dy in (0, 1) for dx in (0, 1)]
bits, fg, bg, wall = _quad_cell(w4)
assert QUAD[bits] == '▝' and fg == RGB[OCC] and bg == RGB[FREE] and wall, '벽 한 칸이 블록으로 남아야 함'
col = [((dx, dy), OCC if dx == 0 else FREE) for dy in (0, 1) for dx in (0, 1)]
assert QUAD[_quad_cell(col)[0]] == '▌', '세로 벽 = 왼쪽 반 블록 (이어진 선)'
edge = [((dx, dy), FREE if dx == 0 else OUT) for dy in (0, 1) for dx in (0, 1)]
bits, fg, bg, wall = _quad_cell(edge)
assert fg == RGB[FREE] and bg is None and not wall, '지도 가장자리는 바탕 그대로'
from ros.fake.FakeRos import fake_map  # noqa: E402
view = MapView(MapGrid.from_msg(fake_map()), color=True)   # 실제 미로 지도 — 벽이 이어진 선으로 나오는지
view.vp.fit(70, 60)
hi = '\n'.join(view.render(70, 30))
assert any(ch in QUAD[1:] for ch in hi), '블록으로 그려져야 함'
view.show_cost = True
lo = '\n'.join(view.render(70, 30))
assert not any(ch in '▘▝▖▗▌▐▚▞▛▜▙▟' for ch in lo), 'costmap 모드는 반블록(▀▄)만'
assert any(ch in '▘▝▖▗▌▐▚▞▛▜▙▟' for ch in hi), '기본은 1/4 블록'
print('map_view quad base ok')

# 경로 끝점 E — 흰 글자 + 경로 색 바탕 (경로선과 같은 색 글자라 안 보이던 것)
view = MapView(g, color=True)
view.vp.fit(30, 20)
view.plans = [([(-0.3, -0.3), (0.3, 0.3)], (20, 90, 255))]
out = '\n'.join(view.render(30, 10))
assert '\033[1;38;2;255;255;255m\033[48;2;20;90;255mG' in out, '끝점 G는 흰 글자 + 경로 색 바탕'
view.plans = [([(-0.3, -0.3), (0.3, 0.3)], (170, 0, 255))]
assert 'L' in ''.join(view.render(30, 10)).replace('\033', ''), '지역 경로 끝은 L'
print('map_view plan end marker ok')
