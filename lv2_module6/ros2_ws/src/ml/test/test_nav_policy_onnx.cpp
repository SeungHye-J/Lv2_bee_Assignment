// Nav2 주행 정책 ONNX 로드 테스트 — rl_nav_controller가 기대하는 입출력과 맞는지
// (기본 lookahead=5 · lidar_rays=36 → 입력 2*5+36+5=51, 출력 2 이상)
#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>

#include "ml/onnx_policy.hpp"

namespace fs = std::filesystem;

TEST(NavPolicyOnnx, LoadsWithExpectedDims) {
#ifndef ML_HAS_ONNX
  GTEST_SKIP() << "onnxruntime_vendor 없음 — ONNX 없이 빌드됨";
#else
  const fs::path path = fs::path(MODEL_DIR) / "nav_controller" / "policy.onnx";
  ASSERT_TRUE(fs::exists(path)) << "모델 없음: " << path;
  ml::OnnxPolicy p;
  EXPECT_NO_THROW(p.load(path.string()));
  ASSERT_TRUE(p.loaded());
  // rl_nav_controller 기본값(lookahead_=5, lidar_rays_=36)과 동기화 — 바꾸면 여기 숫자도 함께
  EXPECT_EQ(p.inputs(), 2 * 5 + 36 + 5);
  EXPECT_GE(p.outputs(), 2);
#endif
}

TEST(NavPolicyOnnx, RunsZeroObs) {
#ifndef ML_HAS_ONNX
  GTEST_SKIP() << "onnxruntime_vendor 없음 — ONNX 없이 빌드됨";
#else
  const fs::path path = fs::path(MODEL_DIR) / "nav_controller" / "policy.onnx";
  ml::OnnxPolicy p;
  p.load(path.string());
  ASSERT_TRUE(p.loaded());
  const std::vector<float> out = p.run(std::vector<float>(static_cast<size_t>(p.inputs()), 0.0f));
  ASSERT_EQ(static_cast<int64_t>(out.size()), p.outputs());
  for (float v : out) EXPECT_TRUE(std::isfinite(v));
#endif
}
