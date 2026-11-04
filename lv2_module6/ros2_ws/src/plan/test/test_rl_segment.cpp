// RL 정책 구간 시간표 (plan/rl_segment.hpp) — 학습 pick_env · demo_cycle pick()과 같은 단계·기준 궤적
#include <gtest/gtest.h>

#include <cmath>

#include "plan/rl_segment.hpp"

using namespace plan;

TEST(RlSegment, TrapezoidMatchesBeeCommon) {
  // bee_common.trapezoid_duration: vmax 0.9 · amax 1.5 (× 0.3) → t_acc 0.6, 삼각형 한계 0.54 rad
  EXPECT_NEAR(trapezoid_duration(0.3, 0.3), 2 * std::sqrt(0.3 / 1.5), 1e-12);
  EXPECT_NEAR(trapezoid_duration(1.0, 0.3), 1.0 / 0.9 + 0.6, 1e-12);
  EXPECT_DOUBLE_EQ(trapezoid_duration(0.0, 0.3), 0.1);
  EXPECT_DOUBLE_EQ(move_s(-1.0, 1.0), 0.0);
  EXPECT_DOUBLE_EQ(move_s(0.5, 1.0), 0.5);
  EXPECT_DOUBLE_EQ(move_s(2.0, 1.0), 1.0);
}

TEST(RlSegment, PhasesAndReference) {
  const std::vector<double> q0{0, 0, 0, 0, 0}, qa{0.3, 0, 0, 0, 0}, qd{0.3, 0.2, 0, 0, 0};
  RlTimes times;
  const auto tl = rl_timeline(q0, qa, qd, times);
  ASSERT_EQ(tl.t_ph.size(), 6u);
  EXPECT_NEAR(tl.t_ph[0], tl.t_tr + times.hold, 1e-12);
  EXPECT_NEAR(tl.t_ph[1] - tl.t_ph[0], tl.t_mv, 1e-12);
  EXPECT_NEAR(tl.t_ph[3] - tl.t_ph[2], times.act, 1e-12);
  EXPECT_EQ(rl_phase(0.0, tl), 0);
  EXPECT_EQ(rl_phase(tl.t_ph[2] + 1e-6, tl), 3);
  EXPECT_EQ(rl_phase(tl.t_ph.back(), tl), 6);
  // 이동 끝 = 위 · 멈춤 동안 위 · 내려감 끝 = 아래 · 닫는 동안 아래 · 끝 = 다시 위
  EXPECT_NEAR(rl_ref(tl.t_tr, q0, qa, qd, tl)[0], 0.3, 1e-12);
  EXPECT_NEAR(rl_ref(tl.t_ph[0] - 1e-3, q0, qa, qd, tl)[1], 0.0, 1e-12);
  EXPECT_NEAR(rl_ref(tl.t_ph[1], q0, qa, qd, tl)[1], 0.2, 1e-12);
  EXPECT_NEAR(rl_ref(0.5 * (tl.t_ph[2] + tl.t_ph[3]), q0, qa, qd, tl)[1], 0.2, 1e-12);
  EXPECT_NEAR(rl_ref(tl.t_ph[4], q0, qa, qd, tl)[1], 0.0, 1e-12);
  EXPECT_NEAR(rl_ref(0.5 * tl.t_tr, q0, qa, qd, tl)[0], 0.15, 1e-12);  // smooth step 가운데
}

TEST(RlSegment, ContextJson) {
  RlTimes times;
  const auto tl = rl_timeline({0, 0, 0, 0, 0}, {0.1, 0, 0, 0, 0}, {0.1, 0.1, 0, 0, 0}, times);
  const auto j = rl_context_json("pick", 12.5, tl, {0.1, 0, 0, 0, 0}, {0.1, 0.1, 0, 0, 0}, 7, true, 0.25, {});
  EXPECT_NE(j.find("\"skill\": \"pick\""), std::string::npos) << j;
  EXPECT_NE(j.find("\"target_id\": 7"), std::string::npos) << j;
  EXPECT_NE(j.find("\"is_box\": true"), std::string::npos) << j;
  EXPECT_NE(j.find("\"cell\": []"), std::string::npos) << j;
  EXPECT_EQ(rl_context_json("", 0, tl, {}, {}, 0, false, 0, {}), "{\"skill\": \"\"}");
}
