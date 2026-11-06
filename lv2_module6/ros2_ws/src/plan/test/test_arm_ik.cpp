// 팔 전용 IK (plan/arm_ik.hpp) — 정기구학이 실기 TF와 맞고(11/03 실측), 위→아래 목표를 손 방향까지 푸는지
#include <gtest/gtest.h>

#include <cmath>

#include "plan/arm_ik.hpp"
#include "plan/rl_segment.hpp"

using namespace plan;

TEST(ArmIk, FkMatchesRobotTf) {
  // 실기: joint 0.53076 0.54150 −0.80074 1.81777 1.84845 → TF arm_base_link ← end_effector_link (0.1802, 0.1054, 0.0397)
  const auto T = arm_fk({0.53076, 0.54150, -0.80074, 1.81777, 1.84845});
  EXPECT_NEAR(T[3], 0.1802, 0.001);
  EXPECT_NEAR(T[7], 0.1054, 0.001);
  EXPECT_NEAR(T[11], 0.0397, 0.001);
}

TEST(ArmIk, SolvesTopDownWithYaw) {
  const std::vector<double> stow{0.0, -1.9024, 1.344, 0.9023, 0.0};
  for (const auto & [x, y, z, psi] : std::vector<std::array<double, 4>>{
         {0.20, 0.00, 0.05, 0.5}, {0.18, 0.06, 0.06, 1.2}, {0.15, -0.05, 0.08, -0.7}, {0.22, 0.02, 0.03, 0.0}}) {
    std::vector<double> q;
    ASSERT_TRUE(arm_ik_down({x, y, z}, psi, stow, q)) << x << " " << y << " " << z;
    const auto T = arm_fk(q);
    EXPECT_NEAR(T[3], x, 0.002);
    EXPECT_NEAR(T[7], y, 0.002);
    EXPECT_NEAR(T[11], z, 0.002);
    // 손이 아래 · 닫힘축(손 y) 방향이 ψ − 90° 또는 그 반대 (손이 대칭 — ψ+π 해도 같은 파지)
    EXPECT_LT(std::min(down_rot_err(T, psi), down_rot_err(T, psi + M_PI)), 0.03);
  }
}

TEST(ArmIk, RejectsUnreachableDown) {
  // 25 cm 앞 · 10 cm 위는 손을 아래로 향한 채로는 못 감 (joint4 한계 — 손끝 최대 높이가 거리에 따라 급히 낮아짐)
  std::vector<double> q;
  EXPECT_FALSE(arm_ik_down({0.25, 0.05, 0.10}, 1.2, {0.0, -1.9024, 1.344, 0.9023, 0.0}, q));
}

TEST(ArmIk, JointPathPassesWaypoints) {
  const std::vector<std::vector<double>> pts{{0, 0, 0, 0, 0}, {0.2, 0, 0, 0, 0}, {0.2, 0.1, 0, 0, 0}};
  const auto path = joint_path(pts, 0.3);
  ASSERT_FALSE(path.empty());
  EXPECT_NEAR(path.back().second[0], 0.2, 1e-9);
  EXPECT_NEAR(path.back().second[1], 0.1, 1e-9);
  // 경로 길이 0.3 rad → 사다리꼴 시간 (0.5 s 최소)
  EXPECT_NEAR(path.back().first, std::max(trapezoid_duration(0.3, 0.3), 0.5), 0.02);
  // 가운데쯤엔 첫 구간을 다 가고 두 번째 구간 진행 중이거나 첫 구간 위
  for (const auto & [t, q] : path) EXPECT_TRUE(q[1] < 1e-9 || std::abs(q[0] - 0.2) < 1e-9) << t;
}

#include <cstdio>

#include "plan/stand_geometry.hpp"

// 세우기 경로 전체가 이 팔로 손을 아래로 향한 채 풀리는지 (퍽 = 로봇 중심 앞 d, 받침 높이 · 축 방향별)
//   arm_base_link = base_footprint + (0.048, 0, 0.140) (urdf arm_mount_joint + base_joint)
static int stand_fail_at(double d, double support, double axis_deg) {
  StandParams P;
  const double far = far_axis_yaw(axis_deg * M_PI / 180, d, 0.0, 0.0, 0.0);
  const auto path = stand_path(d, 0.0, support, far, true, P);
  std::vector<geometry_msgs::msg::Pose> ps{path.pinch};
  ps.insert(ps.end(), path.arc_up.begin(), path.arc_up.end());
  ps.insert(ps.end(), path.arc_back.begin(), path.arc_back.end());  // 들어 올리기는 coordinator가 MoveIt으로
  std::vector<double> seed{0.0, -1.9024, 1.3788, 0.9023, 0.0}, q;  // 11/03 stow
  // 먼 끝 위 대기 자리 — coordinator처럼 4 → 3 → 2 → 1 cm 중 닿는 높이 (하나도 안 되면 0번째 점 실패)
  bool above = false;
  for (double h : {P.above_m, 0.03, 0.02, 0.01}) {
    const auto & p0 = path.pinch.position;
    if (arm_ik_down({p0.x - 0.047985, p0.y + 0.00023, p0.z + h - 0.140175}, path.tool_yaw, seed, q)) { above = true; seed = q; break; }
  }
  if (!above) return 0;
  for (size_t k = 0; k < ps.size(); ++k) {
    double psi = 0.0;
    top_down_yaw(ps[k].orientation, psi);
    if (!arm_ik_down({ps[k].position.x - 0.047985, ps[k].position.y + 0.00023, ps[k].position.z - 0.140175}, psi, seed, q))
      return static_cast<int>(k) + 1;
    seed = q;
  }
  return -1;
}

TEST(ArmIk, StandPathReach) {
  for (double d : {0.22, 0.25, 0.276, 0.28})
    for (double sz : {0.0, 0.07, 0.098})
      for (double ax : {99.0, 0.0, 45.0}) {
        const int bad = stand_fail_at(d, sz, ax);
        std::printf("[reach] d %.2f support %.3f axis %3.0f : %s %d\n", d, sz, ax, bad < 0 ? "OK" : "FAIL at", bad);
      }
  // 바닥 · 시작 상자(0.07) 위, 로봇 중심 앞 0.22~0.25 m — 풀려야 함 (stand-test 안내 거리)
  for (double d : {0.22, 0.25})
    for (double sz : {0.0, 0.07})
      for (double ax : {99.0, 0.0, 45.0}) EXPECT_EQ(stand_fail_at(d, sz, ax), -1) << d << " " << sz << " " << ax;
}
