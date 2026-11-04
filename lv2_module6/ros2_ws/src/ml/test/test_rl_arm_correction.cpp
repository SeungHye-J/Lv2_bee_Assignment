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
