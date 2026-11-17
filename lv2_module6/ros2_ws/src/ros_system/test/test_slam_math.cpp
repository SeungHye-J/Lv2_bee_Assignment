// cuvslam_slam C++판 계산 (include/ros_system/slam_math.hpp) — test_map_gate.py와 같은 시나리오 · 같은 숫자
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "ros_system/slam_math.hpp"

using ros_system::MapGate;
using ros_system::Pose2;

namespace {
Pose2 upd(MapGate & g, Pose2 c, double t, bool & ok, Pose2 odom = {}) { return g.update(c, t, odom, ok); }
}  // namespace

TEST(MapGate, JumpLocksRateLimitsReset) {
  MapGate g(0.30, 15.0, 0.02, 2.0);  // 예전 한계로 — 숫자 그대로 확인
  bool ok = false;
  auto out = upd(g, {0, 0, 0}, 0.0, ok);
  EXPECT_TRUE(ok);
  out = upd(g, {0.001, 0, 0}, 0.1, ok);  // 0.1 s에 1 mm — 초당 2 cm 안이라 그대로
  EXPECT_TRUE(ok);
  EXPECT_NEAR(out.x, 0.001, 1e-9);
  out = upd(g, {0.05, 0, 0}, 0.2, ok);  // 0.1 s에 5 cm — 2 mm만 따라감
  EXPECT_FALSE(ok);
  EXPECT_NEAR(out.x, 0.003, 1e-9);
  EXPECT_FALSE(g.locked);
  out = upd(g, {-4.1, -1.0, 0.5}, 0.3, ok);  // T0: 4.8 m 점프 → 거부 · 잠금
  EXPECT_FALSE(ok);
  EXPECT_TRUE(g.locked);
  EXPECT_NEAR(out.x, 0.003, 1e-9);
  for (int i = 0; i < 50; ++i) out = upd(g, {-4.1, -1.0, 0.5}, 0.4 + 0.1 * i, ok);  // 잘못된 자리가 안정돼도 안 받음
  EXPECT_FALSE(ok);
  EXPECT_NEAR(out.x, 0.003, 1e-9);
  g.reset();  // /initialpose
  out = upd(g, {1.0, 2.0, 0.1}, 6.0, ok);
  EXPECT_TRUE(ok);
  EXPECT_FALSE(g.locked);
  EXPECT_DOUBLE_EQ(out.x, 1.0);
}

TEST(MapGate, SettleAcceptsStartJumpThenLocks) {
  MapGate g(0.30, 15.0, 0.02, 2.0, 15.0);
  bool ok = false;
  upd(g, {0, 0, 0}, 100.0, ok);
  auto out = upd(g, {-0.75, 0, 0}, 102.4, ok);  // 서 있는데 EKF odom이 +0.75 m 튐
  EXPECT_TRUE(ok);
  EXPECT_FALSE(g.locked);
  EXPECT_DOUBLE_EQ(out.x, -0.75);
  ASSERT_TRUE(g.resettled.has_value());
  EXPECT_NEAR(g.resettled->x, 0.75, 1e-9);
  for (int i = 0; i < 100; ++i) out = upd(g, {-0.75, 0, 0}, 103.0 + 0.1 * i, ok);
  EXPECT_TRUE(ok);
  EXPECT_FALSE(g.locked);
  out = upd(g, {-4.1, -1.0, 0.5}, 120.0, ok);  // 정착 끝난 뒤 점프는 잠금
  EXPECT_FALSE(ok);
  EXPECT_TRUE(g.locked);
  EXPECT_DOUBLE_EQ(out.x, -0.75);
  g.reset();  // /initialpose 뒤엔 정착 다시 안 함 (t0 유지)
  upd(g, {0, 0, 0}, 121.0, ok);
  upd(g, {1.0, 0, 0}, 121.1, ok);
  EXPECT_FALSE(ok);
  EXPECT_TRUE(g.locked);
}

TEST(MapGate, RotationAtRobotFarFromOdomOrigin) {
  const double c5 = 5.0 * M_PI / 180.0;
  const Pose2 odom{4.7, 0.0, 0.0};
  const Pose2 rot = MapGate::comp({4.7, 0.0, c5}, MapGate::inv(odom));  // 로봇 자리는 그대로, 방향만 5°
  EXPECT_GT(std::hypot(rot.x, rot.y), 0.30);
  MapGate g;
  bool ok = false;
  g.update({0, 0, 0}, 0.0, odom, ok);
  auto out = g.update(rot, 0.1, odom, ok);
  EXPECT_FALSE(g.locked);
  EXPECT_FALSE(ok);
  for (int i = 2; i < 40; ++i) out = g.update(rot, 0.1 * i, odom, ok);
  EXPECT_TRUE(ok);
  EXPECT_FALSE(g.locked);
  EXPECT_NEAR(out.x, rot.x, 1e-9);
  EXPECT_NEAR(out.y, rot.y, 1e-9);
  EXPECT_NEAR(out.yaw, rot.yaw, 1e-9);
  const auto p = MapGate::comp(out, odom);
  EXPECT_NEAR(p.x, 4.7, 1e-9);
  EXPECT_NEAR(p.y, 0.0, 1e-9);
  MapGate g2;  // 같은 자리에서 진짜 1 m 점프는 잠금
  g2.update({0, 0, 0}, 0.0, odom, ok);
  g2.update({1.0, 0, 0}, 0.1, odom, ok);
  EXPECT_FALSE(ok);
  EXPECT_TRUE(g2.locked);
  MapGate g3;  // 초당 5 cm — 0.1 s에 2 cm 이동이면 5 mm만
  g3.update({0, 0, 0}, 0.0, odom, ok);
  out = g3.update({0.02, 0, 0}, 0.1, odom, ok);
  EXPECT_FALSE(ok);
  EXPECT_NEAR(MapGate::comp(out, odom).x, 4.705, 1e-9);
}

TEST(SlamMath, MapCorrectionMovesObjectsBack) {
  using ros_system::compose;
  using ros_system::inverse;
  const Pose2 off0{0.3, -0.2, 0.4}, raw{2.0, 1.0, -0.5}, c{0.05, -0.03, 1.0 * M_PI / 180.0};
  const auto base1 = compose(ros_system::apply_map_correction(off0, c), raw);
  const auto want = compose(inverse(c), compose(off0, raw));
  EXPECT_NEAR(base1.x, want.x, 1e-9);
  EXPECT_NEAR(base1.y, want.y, 1e-9);
  EXPECT_NEAR(base1.yaw, want.yaw, 1e-9);
  const Pose2 obj{1.5, 2.5, 0.0};
  const auto after = compose(inverse(c), compose(c, obj));
  EXPECT_NEAR(after.x, obj.x, 1e-9);
  EXPECT_NEAR(after.y, obj.y, 1e-9);
}

TEST(SlamMath, NearMaskMatchesOldFormula) {
  const std::vector<uint16_t> d{0, 1, 498, 499, 500, 501, 65535};
  for (const auto [min_depth, scale] : {std::pair{0.5, 1000.0}, {0.4999, 1000.0}, {0.0, 1000.0}, {0.3, 1000.0}}) {
    std::vector<uint8_t> m(d.size());
    ros_system::near_mask(d.data(), m.data(), d.size(), ros_system::near_limit(min_depth, scale));
    for (size_t i = 0; i < d.size(); ++i)
      EXPECT_EQ(m[i], (d[i] > 0 && d[i] < min_depth * scale) ? 1 : 0) << min_depth << " depth " << d[i];
  }
}

TEST(SlamMath, BaseAndOdomGeometry) {
  // base_from_cam = 카메라가 base 앞 0.1 m, OpenCV 축(z 앞 · x 오른쪽 · y 아래) → world에서 카메라가 z로 1 m = base가 x로 1 m
  Eigen::Matrix4d B = Eigen::Matrix4d::Identity();
  B.block<3, 3>(0, 0) << 0, 0, 1, -1, 0, 0, 0, -1, 0;
  B(0, 3) = 0.1;
  Eigen::Matrix4d W = Eigen::Matrix4d::Identity();
  W(2, 3) = 1.0;
  const auto b = ros_system::base_pose(B, W);
  EXPECT_NEAR(b.x, 1.0, 1e-9);
  EXPECT_NEAR(b.y, 0.0, 1e-9);
  EXPECT_NEAR(b.yaw, 0.0, 1e-9);
  // map_to_odom ∘ odom = base
  const Pose2 base{2.0, -1.0, 0.7}, odom{0.5, 0.3, -0.2};
  const auto mo = ros_system::map_to_odom(base, odom), back = ros_system::compose(mo, odom);
  EXPECT_NEAR(back.x, base.x, 1e-9);
  EXPECT_NEAR(back.y, base.y, 1e-9);
  EXPECT_NEAR(back.yaw, base.yaw, 1e-9);
  // body_twist: yaw π/2에서 앞(+y map)으로 0.1 m · 0.1 s → vx 1 m/s
  const auto v = ros_system::body_twist({0, 0, M_PI / 2}, {0, 0.1, M_PI / 2 + 0.05}, 0.1);
  EXPECT_NEAR(v.x, 1.0, 1e-9);
  EXPECT_NEAR(v.y, 0.0, 1e-9);
  EXPECT_NEAR(v.yaw, 0.5, 1e-9);
}

TEST(MapGate, HoldsWhileRotating) {
  MapGate g;
  bool ok = false;
  g.update({0, 0, 0}, 0.0, {}, ok);
  auto out = g.update({0.01, 0, 0.02}, 0.1, {}, ok, true);  // 회전 중 — 작은 보정도 안 넣음
  EXPECT_FALSE(ok);
  EXPECT_DOUBLE_EQ(out.x, 0.0);
  EXPECT_EQ(g.held_rot, 1);
  EXPECT_EQ(g.rejected, 0);
  EXPECT_FALSE(g.locked);
  out = g.update({1.0, 0, 0}, 0.2, {}, ok, true);  // 회전 중엔 큰 차이도 잠그지 않음 (멈춘 뒤 판단)
  EXPECT_FALSE(g.locked);
  out = g.update({0.001, 0, 0}, 0.3, {}, ok, false);  // 멈춘 뒤 다시 따라감
  EXPECT_TRUE(ok);
  EXPECT_DOUBLE_EQ(out.x, 0.001);
}
