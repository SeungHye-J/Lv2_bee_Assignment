#!/usr/bin/env python3
"""cuvslam_slam MapGate — 큰 점프는 거부·잠금, 작은 보정은 초당 rate까지만, reset으로 풀림 (rclpy·cuvslam 없이 클래스만)
   시나리오는 11-10 시뮬 T0(벽 앞 4.8 m 점프)와 정상 주행"""
import math
import pathlib

SRC = (pathlib.Path(__file__).resolve().parents[1] / 'scripts' / 'cuvslam_slam.py').read_text()
ns = {'math': math}
exec(SRC[SRC.index('class MapGate'):SRC.index('def image(')], ns)  # noqa: S102 — 같은 저장소 소스
G = ns['MapGate']
MG_compose, MG_inverse = G._compose, G._inverse

g = G(rate_m=0.02, rate_deg=2.0)   # 예전 한계로 — 숫자 그대로 확인
assert g.update((0.0, 0.0, 0.0), 0.0) == ((0.0, 0.0, 0.0), True)
out, ok = g.update((0.001, 0.0, 0.0), 0.1)                   # 0.1 s에 1 mm — 초당 2 cm 안이라 그대로
assert ok and abs(out[0] - 0.001) < 1e-9
out, ok = g.update((0.05, 0.0, 0.0), 0.2)                     # 0.1 s에 5 cm — 2 mm만 따라감
assert not ok and abs(out[0] - 0.003) < 1e-9 and not g.locked
out, ok = g.update((-4.1, -1.0, 0.5), 0.3)                    # T0: 4.8 m 점프 → 거부 · 잠금
assert not ok and g.locked and abs(out[0] - 0.003) < 1e-9
for i in range(50):
    out, ok = g.update((-4.1, -1.0, 0.5), 0.4 + 0.1 * i)       # 잘못된 자리가 계속 안정돼도 안 받음 (예전 MapGate는 10프레임 뒤 받았음)
assert not ok and abs(out[0] - 0.003) < 1e-9
g.reset()                                                     # /initialpose
assert g.update((1.0, 2.0, 0.1), 6.0) == ((1.0, 2.0, 0.1), True) and not g.locked

# 출발 정착 (11-14 실기): 서 있는데 bringup 2.4 s 뒤 EKF odom이 +0.75 m 튐 (바퀴 누적 엔코더) → 보정 후보 x −0.75
g = G(rate_m=0.02, rate_deg=2.0, settle_s=15.0)
g.update((0.0, 0.0, 0.0), 100.0)
out, ok = g.update((-0.75, 0.0, 0.0), 102.4)                  # 예전: 잠금 · 로봇이 map (0.75, 0)에 있다고 믿음
assert ok and not g.locked and out == (-0.75, 0.0, 0.0) and g.resettled[0] == 0.75
for i in range(100):                                          # 정착 끝난 뒤 정상 — 로봇 map 자세 = 보정∘odom = (0, 0)
    out, ok = g.update((-0.75, 0.0, 0.0), 103.0 + 0.1 * i)
assert ok and not g.locked
out, ok = g.update((-4.1, -1.0, 0.5), 120.0)                  # 정착 뒤 점프는 여전히 잠금
assert not ok and g.locked and out == (-0.75, 0.0, 0.0)
g.reset()                                                     # /initialpose 뒤엔 정착 다시 안 함 (t0 유지)
g.update((0.0, 0.0, 0.0), 121.0)
assert not g.update((1.0, 0.0, 0.0), 121.1)[1] and g.locked

# 도면 보정 → 로봇 위치 (11-15): 조각 c를 받으면 모든 map 자세가 c⁻¹만큼 옮겨짐 → compose(c, 도면)에 보이던 물체가 도면 자리로
ns2 = {'math': math}
exec(SRC[SRC.index('def compose('):SRC.index('def body_twist(')], ns2)  # noqa: S102
exec(SRC[SRC.index('def apply_map_correction('):SRC.index('class MapGate')], ns2)  # noqa: S102
cmp, inv, amc = ns2['compose'], ns2['inverse'], ns2['apply_map_correction']
off0, raw, c = (0.3, -0.2, 0.4), (2.0, 1.0, -0.5), (0.05, -0.03, math.radians(1.0))
base0 = cmp(off0, raw)
base1 = cmp(amc(off0, c), raw)
want = cmp(inv(c), base0)
assert all(abs(a - b) < 1e-9 for a, b in zip(base1, want)), (base1, want)
obj_nominal = (1.5, 2.5, 0.0)
seen_before = cmp(c, obj_nominal)                              # 지금 map에서 물체가 보이는 자리
seen_after = cmp(inv(c), seen_before)                          # 같은 물체가 새 map에서 보이는 자리 = 도면 자리
assert all(abs(a - b) < 1e-9 for a, b in zip(seen_after[:2], obj_nominal[:2]))

# 11-15 실기: odom 원점에서 4.7 m · EKF와 cuVSLAM yaw가 몇 도 다름 → 로봇 자리에서 도는 보정인데 map→odom 이동은 크게 보임
c5 = math.radians(5.0)
odom = (4.7, 0.0, 0.0)                                         # 로봇 odom 자세
rot_at_robot = MG_compose((4.7, 0.0, c5), MG_inverse(odom))    # 로봇 자리는 그대로, 방향만 5° — map→odom 이동 ≈ 0.41 m
assert math.hypot(rot_at_robot[0], rot_at_robot[1]) > 0.30     # 예전 기준이면 0.30 m '점프' → 잠금
g = G()
g.update((0.0, 0.0, 0.0), 0.0, odom)
out, ok = g.update(rot_at_robot, 0.1, odom)
assert not g.locked and not ok                                 # 잠기지 않고 초당 5°로 따라감
for i in range(2, 40):
    out, ok = g.update(rot_at_robot, 0.1 * i, odom)
assert ok and not g.locked and all(abs(a - b) < 1e-9 for a, b in zip(out, rot_at_robot))   # 1 s 안에 다 붙음
p = MG_compose(out, odom)
assert abs(p[0] - 4.7) < 1e-9 and abs(p[1]) < 1e-9             # 따라가는 동안에도 로봇 위치는 안 휘둘림
g = G()                                                        # 같은 자리에서 진짜 큰 점프(로봇이 1 m 옮겨짐)는 여전히 잠금
g.update((0.0, 0.0, 0.0), 0.0, odom)
assert not g.update((1.0, 0.0, 0.0), 0.1, odom)[1] and g.locked
g = G()                                                        # 한계: 로봇 자리에서 초당 5 cm — 0.1 s에 2 cm 이동이면 5 mm만
g.update((0.0, 0.0, 0.0), 0.0, odom)
out, ok = g.update((0.02, 0.0, 0.0), 0.1, odom)
assert not ok and abs(MG_compose(out, odom)[0] - 4.705) < 1e-9

# 영상 → numpy (복사 없이) · 가까운 depth 마스크가 예전 식과 같은지
import array  # noqa: E402
import numpy as np  # noqa: E402

ns = {'np': np}
exec(SRC[SRC.index('def image('):SRC.index('class CuvslamSlam')], ns)  # noqa: S102


class Img:
    pass


d = np.array([[0, 1, 498, 499, 500, 501, 65535, 0]], np.uint16)
m = Img()
m.height, m.width, m.step, m.data = 1, 7, 16, array.array('B', d.tobytes())  # 마지막 칸 = step 패딩
assert (ns['image'](m, np.uint16, 1) == d[:, :7]).all()
for min_depth, scale in ((0.5, 1000.0), (0.4999, 1000.0), (0.0, 1000.0), (0.3, 1000.0)):
    lim = max(math.ceil(min_depth * scale) - 1, 0)
    old = ((d > 0) & (d < min_depth * scale)).astype(np.uint8)
    new = ns['near_mask'](d, lim)
    assert new.dtype == np.uint8 and (new == old).all(), (min_depth, new, old)
print('test_map_gate OK')
