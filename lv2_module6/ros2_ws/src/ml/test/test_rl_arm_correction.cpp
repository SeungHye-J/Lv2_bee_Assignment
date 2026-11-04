// rl_arm_correction 처짐 보정 적분 (sag_step) 확인 — colcon test --packages-select ml
#include <gtest/gtest.h>

#include "ml/rl_arm_correction.hpp"

// 10 mrad 처짐을 100 Hz로 0.3 s (학습의 hold) 보정: 시뮬(85 ms마다 30 % 제거, 0.3 s에 약 72 %)과 같게 줄어야 한다
TEST(SagStep, RemovesSagLikeTraining) {
  const double sag = 0.010, dt = 0.01, gain = 4.2;
  double corr = 0.0;
  for (int k = 0; k < 30; ++k) {
    const double q = 0.5 + corr - sag;  // 정상상태 근사: 실제각 = 명령 − 처짐
    corr = ml::sag_step(corr, 0.5, 0.0, q, dt, gain, 0.2);
  }
  EXPECT_NEAR(corr / sag, 0.72, 0.02);
}

// 정책 Δ는 오차에 포함: q가 ref + Δ에 있으면 corr은 변하지 않는다 (Δ를 지우지 않음)
TEST(SagStep, KeepsResidual) {
  EXPECT_DOUBLE_EQ(ml::sag_step(0.0, 0.5, 0.03, 0.53, 0.01, 4.2, 0.2), 0.0);
}

TEST(SagStep, Clamps) {
  EXPECT_DOUBLE_EQ(ml::sag_step(0.19, 0.0, 0.0, -10.0, 0.01, 4.2, 0.2), 0.2);
}

// 정책 Δ: 받은 지 timeout 안이면 ±max로 잘라 쓰고, 낡았거나 받은 적 없으면 0
TEST(FreshDelta, UsesRecentClampsAndDropsStale) {
  EXPECT_DOUBLE_EQ(ml::fresh_delta(0.03, 0.01, 0.2, 0.1), 0.03);
  EXPECT_DOUBLE_EQ(ml::fresh_delta(0.5, 0.01, 0.2, 0.1), 0.1);
  EXPECT_DOUBLE_EQ(ml::fresh_delta(-0.5, 0.01, 0.2, 0.1), -0.1);
  EXPECT_DOUBLE_EQ(ml::fresh_delta(0.03, 0.25, 0.2, 0.1), 0.0);   // 정책 노드가 멈춤
  EXPECT_DOUBLE_EQ(ml::fresh_delta(0.03, -1.0, 0.2, 0.1), 0.0);   // 받은 적 없음
  EXPECT_DOUBLE_EQ(ml::fresh_delta(NAN, 0.01, 0.2, 0.1), 0.0);
}

// 중력 토크 (팔 평면 점질량) — isaac/sim/R2S/arm_model.py(URDF 전체 FK)와 같은 값. grasp 자세 [deg] q2 29.78 q3 −11.31 q4 71.52
TEST(Gravity, TorquesMatchUrdfModel) {
  const double d2r = M_PI / 180.0;
  const auto t = ml::gravity::torques(29.78 * d2r, -11.31 * d2r, 71.52 * d2r);
  EXPECT_NEAR(t[0], 1.252, 0.01);
  EXPECT_NEAR(t[1], 0.698, 0.01);
  EXPECT_NEAR(t[2], 0.063, 0.005);
  const auto n = ml::gravity::torques(-22.68 * d2r, 46.72 * d2r, 65.96 * d2r);  // 가까이 접은 자세
  EXPECT_NEAR(n[0], 0.478, 0.01);
  EXPECT_NEAR(n[1], 0.674, 0.01);
}

// 고정 높이 오차를 올리는 관절 변화: 손가락 끝면 중심이 dz만큼 올라가고 x·손 피치는 그대로
TEST(Gravity, LiftMovesFaceCentreUpOnly) {
  using namespace ml::gravity;
  const double q2 = 0.3, q3 = 0.1, q4 = 1.1, dz = 0.005;  // 손이 거의 아래를 향한 자세
  std::array<double, 3> dq{};
  ASSERT_TRUE(lift(q2, q3, q4, dz, dq));
  const auto mark = [](double a, double b, double c) {
    const V2 p3 = rot(a, J3_X, J3_Z), p4 = rot(a + b, J4_X, 0.0), m = rot(a + b + c, HAND_X + MARK_X, 0.0);
    return V2{p3.x + p4.x + m.x, p3.z + p4.z + m.z};
  };
  const V2 before = mark(q2, q3, q4), after = mark(q2 + dq[0], q3 + dq[1], q4 + dq[2]);
  EXPECT_NEAR(after.z - before.z, dz, 1e-4);
  EXPECT_NEAR(after.x - before.x, 0.0, 1e-4);
  EXPECT_NEAR(dq[0] + dq[1] + dq[2], 0.0, 1e-9);  // 손 피치 그대로
}
