// hand_geom C++판 계산 확인 (ROS 없음) — test/test_hand_relative.py(파이썬 hand_geom)와 같은 시나리오 (11-16)
#include <gtest/gtest.h>

#include <cmath>

#include "perception/hand_geom.hpp"

namespace hg = perception::hand_geom;
using hg::M3;
using hg::V3;

static const M3 DOWN{{{1.0, 0.0, 0.0}, {0.0, -1.0, 0.0}, {0.0, 0.0, -1.0}}};  // 손끝 z 아래 (bee_common.DOWN = x축 180°)

static M3 rz(double a) {
  const double c = std::cos(a), s = std::sin(a);
  return {{{c, -s, 0.0}, {s, c, 0.0}, {0.0, 0.0, 1.0}}};
}

static M3 matmul(const M3 & A, const M3 & B) {
  M3 C{};
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      for (int k = 0; k < 3; ++k) C[i][j] += A[i][k] * B[k][j];
  return C;
}

static void expect_near(const V3 & a, const V3 & b, double tol = 1e-8) {
  for (int i = 0; i < 3; ++i) EXPECT_NEAR(a[i], b[i], tol) << "성분 " << i;
}

static double rad(double deg) { return deg * M_PI / 180.0; }

TEST(HandGeom, BottomInHandBelowTool) {
  const V3 ee_t{0.2, 0.0, 0.25};
  expect_near(hg::bottom_in_hand(ee_t, DOWN), {0.2, 0.0, 0.25 - hg::GRASP_Z});
}

TEST(HandGeom, ToolYawAndFace) {
  const M3 R = matmul(rz(0.3), DOWN);
  EXPECT_NEAR(hg::tool_yaw(R), 0.3, 1e-9);
  EXPECT_NEAR(hg::face_yaw(0.3 + rad(10), 0.3, true), rad(10), 1e-9);
  EXPECT_NEAR(hg::face_yaw(0.3 + rad(100), 0.3, true), rad(10), 1e-9);  // 사각 퍽은 90° 대칭
  EXPECT_EQ(hg::face_yaw(1.0, 0.3, false), 0.0);
  EXPECT_EQ(hg::face_yaw(std::nullopt, 0.3, true), 0.0);
}

TEST(HandGeom, WrapLikePythonModulo) {
  EXPECT_NEAR(hg::wrap(-M_PI - 0.1, 2 * M_PI), M_PI - 0.1, 1e-12);
  EXPECT_NEAR(hg::wrap(M_PI + 0.1, 2 * M_PI), -M_PI + 0.1, 1e-12);
  EXPECT_EQ(hg::wrap(M_PI, 2 * M_PI), -M_PI);  // 파이썬: (pi + pi) % 2pi − pi = −pi
}

TEST(HandGeom, ToBaseAndCellShift) {
  const V3 base_t{1.0, 2.0, 0.0};
  const M3 base_R = rz(M_PI / 2);
  expect_near(hg::to_base({1.0, 3.0, 0.0}, base_t, base_R), {1.0, 0.0, 0.0});  // 로봇 앞 1 m
  expect_near(hg::cell_now({0.5, 0.5, 0.07}, V3{0.5, 0.5, 0.07}, V3{0.51, 0.49, 0.07}), {0.51, 0.49, 0.07});  // 상자가 밀린 만큼 칸도
  expect_near(hg::cell_now({0.5, 0.5, 0.07}, V3{0.5, 0.5, 0.07}, V3{0.7, 0.5, 0.07}), {0.5, 0.5, 0.07});  // 너무 크면 무시
  expect_near(hg::cell_now({0.5, 0.5, 0.07}, std::nullopt, V3{0.51, 0.49, 0.07}), {0.5, 0.5, 0.07});  // 상자 보정 안 함
}

TEST(HandGeom, PuckPickAndNeighbour) {
  const std::vector<hg::Puck> pucks{{3, {0.18, 0.04, 0.10}}, {7, {0.18, -0.04, 0.10}}};
  EXPECT_EQ(hg::pick_target(pucks, {0.18, -0.03, 0.2}), 1u);  // 손에 가까운 것
  EXPECT_EQ(hg::pick_target(pucks, {0.18, -0.03, 0.2}, 3), 0u);
  EXPECT_FALSE(hg::pick_target(pucks, {0.0, 0.0, 0.0}, 5));
  EXPECT_FALSE(hg::pick_target({}, {0.0, 0.0, 0.0}));
  auto nb = hg::neighbour(pucks, 0);
  ASSERT_TRUE(nb);
  expect_near(*nb, {0.0, -0.08, 0.0});
  EXPECT_FALSE(hg::neighbour(pucks, 0, 0.05));
  expect_near(hg::puck_centre({0, 0, 0.13}, true), {0, 0, 0.10});
  expect_near(hg::puck_centre({0, 0, 0.13}, false), {0, 0, 0.13});
}

TEST(HandGeom, LyingAxisPointsAway) {
  expect_near(hg::puck_axis(false, M_PI, {0.2, 0.0, 0.08}), {1.0, 0.0, 0.0}, 1e-9);
  expect_near(hg::puck_axis(true, 0.3, {0.2, 0.0, 0.08}), {0.0, 0.0, 1.0});
}

// 마지막 검출(map)은 그 자리 · 손이 내려가면 rel이 그만큼 줄어듦 (시뮬 pick_env: last_m − 지금 손)
TEST(HandGeom, HeldRelativeFollowsTheHand) {
  const V3 base_t{1.0, 2.0, 0.0};
  const M3 base_R = rz(M_PI / 2);
  const V3 c_map = hg::add(base_t, hg::mul(base_R, {0.20, 0.01, 0.10}));  // base_link (0.20, 0.01, 0.10)의 퍽 가운데
  hg::Held held{"puck", c_map, true, M_PI / 2 + 0.1, true};
  const auto r0 = hg::held_relative(held, base_t, base_R, {0.20, 0.0, 0.20}, DOWN);
  const auto r1 = hg::held_relative(held, base_t, base_R, {0.20, 0.0, 0.15}, DOWN);
  expect_near(r0.rel, {0.0, 0.01, -0.10});
  expect_near(r1.rel, {0.0, 0.01, -0.05});
  EXPECT_NEAR(r0.face_yaw, 0.1, 1e-9);
  expect_near(r0.axis, {0, 0, 1});
  const auto cell = hg::held_relative(hg::Held{"cell", c_map}, base_t, base_R, {0.20, 0.0, 0.20}, DOWN);
  expect_near(cell.rel, hg::sub({0.20, 0.01, 0.10}, hg::bottom_in_hand({0.20, 0.0, 0.20}, DOWN)));
}

// pick 손끝 목표 → 겨냥한 퍽 가운데 (시뮬 tgt_insp_b) · unload 칸 안 퍽 − 손 (시뮬 UnloadEnv slot_tf)
TEST(HandGeom, AimedCentreAndSlotPuck) {
  const V3 bottom{0.3, 0.1, 0.07};
  expect_near(hg::aimed_centre(hg::add(bottom, {0, 0, hg::GRASP_Z})), hg::add(bottom, {0, 0, hg::PUCK_H / 2}));
  const V3 floor{-0.10484, 0.0325, 0.09125};
  expect_near(hg::slot_puck_rel(floor, {-0.1, 0.03, 0.2}), {-0.00484, 0.0025, 0.09125 + 0.03 - 0.2});
}

int main(int argc, char ** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
