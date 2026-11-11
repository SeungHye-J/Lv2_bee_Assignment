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
assert 'E' in with_plan and with_plan.count(':') > base.count(':'), '경로가 그려져야 함'
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

# 화살표·방향선 밑의 벽 — 글자 한 칸의 위·아래 중 한쪽이라도 벽이면 배경은 벽 색 (회색으로 지워지지 않음)
data = [0] * 64
for c in range(8):
    data[4 * 8 + c] = 100        # 가로 벽 한 줄 (한 칸 두께)
gw = MapGrid(8, 8, 0.1, -0.4, -0.4, data)
view = MapView(gw, color=True)
view.vp.fit(40, 20)
view.show_yaw = True
view.yaw = 90.0
view.vp.cy = -0.4 + 4.5 * 0.1   # 십자선을 벽 근처로
out = '\n'.join(view.render(40, 12))
wall = '\033[48;2;18;18;24m'
import re as _re
braille = [m for m in _re.finditer(r'(\033\[[0-9;]*m)+[⠁-⣿]', out)]
assert braille and any(wall in b.group(0) for b in braille), '벽 위를 지나는 방향선 칸의 배경이 벽 색이어야 함'
print('map_view wall under arrow ok')
