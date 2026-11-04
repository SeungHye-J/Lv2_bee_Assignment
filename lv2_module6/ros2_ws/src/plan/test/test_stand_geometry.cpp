// 세우기 경로 테스트 (plan/stand_geometry.hpp) — ROS 통신 없이 식만: 먼 끝 위쪽 절반 잡기 · 가까운 끝 모서리 축 호 · 내려놓기
#include <gtest/gtest.h>

#include <cmath>

#include "plan/stand_geometry.hpp"

using namespace plan;

namespace {

constexpr double DEG = M_PI / 180.0;

perception::msg::TrackedObject object(const std::string & cls, double x, double y, double z, double sx, double sy, double sz,
                                      double yaw_deg = NAN, int period_deg = 0) {
  perception::msg::TrackedObject o;
  o.class_name = cls;
  o.pose.pose.position.x = x;
  o.pose.pose.position.y = y;
  o.pose.pose.position.z = z;
  o.pose.pose.orientation.w = 1.0;
  o.size.x = sx;
  o.size.y = sy;
  o.size.z = sz;
  if (!std::isnan(yaw_deg)) {
    o.pose.pose.orientation.z = std::sin(yaw_deg * DEG / 2);
    o.pose.pose.orientation.w = std::cos(yaw_deg * DEG / 2);
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = "yaw_period";
    kv.value = std::to_string(period_deg);
    o.details.push_back(kv);
  }
  return o;
}

// 퍽 (0.3, 0) · 받침 윗면 0.07 · 긴 축 +x (먼 끝 = +x) — 시작 상자 위 누운 퍽
const StandParams P;
StandPath box_path(double axis = 0.0) { return stand_path(0.3, 0.0, 0.07, axis, true, P); }

}  // namespace

TEST(StandGeometry, TopDownRoundTrip) {
  double psi = 0.0;
  ASSERT_TRUE(top_down_yaw(top_down_q(0.7), psi));
  EXPECT_NEAR(psi, 0.7, 1e-12);
  ASSERT_TRUE(top_down_yaw(top_down_q(-2.5), psi));
  EXPECT_NEAR(psi, -2.5, 1e-12);
  geometry_msgs::msg::Quaternion q;  // 단위 (w = 1) — 손 자세 아님 (적재함 칸 목표)
  q.w = 1.0;
  EXPECT_FALSE(top_down_yaw(q, psi));
  q.w = std::cos(0.3);  // 물체 yaw 쿼터니언 (inspect 목표에 퍽 pose를 그대로 넣음)
  q.z = std::sin(0.3);
  EXPECT_FALSE(top_down_yaw(q, psi));
  q.w = 0.0;  // yaw 180° 물체 (0, 0, 1, 0)
  q.z = 1.0;
  EXPECT_FALSE(top_down_yaw(q, psi));
}

TEST(StandGeometry, FarAxisPointsAwayFromRobot) {
  EXPECT_NEAR(far_axis_yaw(M_PI, 0.3, 0.0, 0.0, 0.0), 0.0, 1e-12);       // 뒤집어 먼 쪽으로
  EXPECT_NEAR(far_axis_yaw(0.0, 0.3, 0.0, 0.0, 0.0), 0.0, 1e-12);        // 이미 먼 쪽
  EXPECT_NEAR(std::cos(far_axis_yaw(0.0, 0.3, 0.0, 1.0, 0.0)), -1.0, 1e-12);  // 로봇이 +x 쪽이면 −x가 먼 쪽 (±π)
  EXPECT_NEAR(far_axis_yaw(M_PI / 2, 0.3, 0.0, 0.0, 0.0), M_PI / 2, 1e-12);  // 로봇 선에 직각 — 그대로
}

TEST(StandGeometry, SupportTopFromBoxElseHalfHeight) {
  const auto puck = object("puck_lying", 0.3, 0.0, 0.087, 0.06, 0.03, 0.03);
  const auto box = object("start_box", 0.3, 0.03, 0.071, 0.205, 0.205, 0.07);
  EXPECT_DOUBLE_EQ(support_top_z(puck, {puck, box}, 0.03), 0.071);       // 상자 윗면
  const auto away = object("start_box", 1.0, 0.0, 0.071, 0.205, 0.205, 0.07);
  EXPECT_NEAR(support_top_z(puck, {puck, away}, 0.03), 0.072, 1e-12);    // 상자 밖 → 퍽 중심 − 잰 높이/2
  auto unsized = puck;
  unsized.size.z = 0.0;
  EXPECT_NEAR(support_top_z(unsized, {unsized}, 0.03), 0.072, 1e-12);    // 높이를 못 쟀으면 퍽 폭/2
  // 돌아간 상자 (45°) — 모서리 근처 점이 사각형 밖이면 상자 아님
  const auto rot = object("start_box", 0.0, 0.0, 0.071, 0.205, 0.205, 0.07, 45.0, 90);
  const auto corner = object("puck_lying", 0.1, 0.1, 0.087, 0.06, 0.03, 0.03);
  EXPECT_NEAR(support_top_z(corner, {corner, rot}, 0.03), 0.072, 1e-12);
  const auto inside = object("puck_lying", 0.0, 0.1, 0.087, 0.06, 0.03, 0.03);
  EXPECT_DOUBLE_EQ(support_top_z(inside, {inside, rot}, 0.03), 0.071);
}

TEST(StandGeometry, PinchFarEndUpperHalf) {
  const auto p = box_path();
  // 손가락 중심선 = 먼 끝면 (중심 + L/2), 높이 = 받침 + 누운 퍽 윗면(0.030) + 손끝 오프셋
  EXPECT_NEAR(p.pinch.position.x, 0.33, 1e-12);
  EXPECT_NEAR(p.pinch.position.y, 0.0, 1e-12);
  EXPECT_NEAR(p.pinch.position.z, 0.07 + 0.030 + P.ee_up, 1e-12);
  EXPECT_NEAR(p.above.position.z - p.pinch.position.z, P.above_m, 1e-12);
  // 원기둥은 축 높이
  const auto c = stand_path(0.3, 0.0, 0.07, 0.0, false, P);
  EXPECT_NEAR(c.pinch.position.z, 0.07 + 0.015 + P.ee_up, 1e-12);
  // 닫힘축(ψ − 90°)이 긴 축에 직각 · 손은 위→아래
  double psi = 0.0;
  ASSERT_TRUE(top_down_yaw(p.pinch.orientation, psi));
  EXPECT_NEAR(std::abs(std::cos(psi - M_PI / 2 - 0.0)), 0.0, 1e-9);
}

TEST(StandGeometry, ArcAboutNearBottomEdge) {
  const auto p = box_path();
  const double a = P.puck_l, b = P.contact_h_box, r = std::hypot(a, b);
  const double pivot_x = 0.3 - P.puck_l / 2, end = P.arc_end_deg * DEG;
  const size_t n = p.arc_up.size();
  ASSERT_EQ(n, static_cast<size_t>(std::round(P.arc_end_deg / P.step_deg)));
  for (size_t k = 0; k < n; ++k) {
    const double f = end * (k + 1) / n;
    const auto & q = p.arc_up[k].position;
    const double s = q.x - pivot_x, h = q.z - 0.07 - P.ee_up - P.tilt_over_m * f / end;
    EXPECT_NEAR(std::hypot(s, h), r, 1e-9) << k;              // 축에서 같은 거리 (잡는 점이 퍽과 함께 돎)
    EXPECT_NEAR(std::atan2(h, s), f + std::atan2(b, a), 1e-9) << k;  // f만큼 돈 자리
    EXPECT_NEAR(q.y, 0.0, 1e-12);
  }
  EXPECT_NEAR(p.arc_up.back().position.z - 0.07 - P.ee_up, a * std::sin(end) + b * std::cos(end) + P.tilt_over_m, 1e-9);
}

TEST(StandGeometry, BackToUprightAndSetDown) {
  const auto p = box_path();
  // 112.5 · 105 · 97.5 · 90° (호 끝 높이 유지) + 내려놓기
  ASSERT_EQ(p.arc_back.size(), 5u);
  const auto & set = p.arc_back.back().position;
  EXPECT_NEAR(set.x, 0.3 - P.puck_l / 2 - P.contact_h_box, 1e-9);  // 선 퍽 윗면 위 — 축에서 b만큼 안쪽
  EXPECT_NEAR(set.z, 0.07 + P.puck_l + P.ee_up + P.set_dz_m, 1e-9);
  const auto & up90 = p.arc_back[3].position;
  EXPECT_NEAR(up90.z - set.z, P.tilt_over_m - P.set_dz_m, 1e-9);
  EXPECT_NEAR(p.lift.position.z - set.z, P.lift_away_m, 1e-12);
  EXPECT_NEAR(p.lift.position.x, set.x, 1e-12);
}

TEST(StandGeometry, RotatedAxis) {
  const auto p = box_path(M_PI / 2);  // 긴 축 +y
  EXPECT_NEAR(p.pinch.position.x, 0.3, 1e-12);
  EXPECT_NEAR(p.pinch.position.y, 0.03, 1e-12);
  EXPECT_NEAR(p.arc_back.back().position.y, -P.puck_l / 2 - P.contact_h_box, 1e-9);
  double psi = 0.0;
  ASSERT_TRUE(top_down_yaw(p.arc_up[3].orientation, psi));  // 호 내내 같은 손 방향
  EXPECT_NEAR(psi, M_PI / 2, 1e-12);
}

TEST(StandGeometry, LyingAndYawHelpers) {
  auto o = object("puck", 0, 0, 0, 0, 0, 0, 30.0, 180);
  EXPECT_FALSE(is_lying(o));
  diagnostic_msgs::msg::KeyValue kv;
  kv.key = "posture";
  kv.value = "lying";
  o.details.push_back(kv);
  EXPECT_TRUE(is_lying(o));
  double yaw = 0.0;
  ASSERT_TRUE(object_yaw(o, yaw));
  EXPECT_NEAR(yaw, 30 * DEG, 1e-12);
  EXPECT_FALSE(object_yaw(object("puck_lying", 0, 0, 0, 0, 0, 0), yaw));  // yaw_period 없음 = 방향 모름
}

TEST(StandGeometry, SnapSupportToKnownLevels) {
  const std::vector<double> lv{0.0, 0.07};
  EXPECT_DOUBLE_EQ(snap_support(0.098, lv, 0.05), 0.07);   // 실측: 상자 위 누운 퍽 (보이는 윗면 쪽 z)
  EXPECT_DOUBLE_EQ(snap_support(0.012, lv, 0.05), 0.0);
  EXPECT_DOUBLE_EQ(snap_support(0.04, lv, 0.05), 0.07);    // 둘 다 가까우면 더 가까운 쪽
  EXPECT_DOUBLE_EQ(snap_support(0.20, lv, 0.05), 0.20);    // 어느 것과도 멀면 잰 값 그대로
}
