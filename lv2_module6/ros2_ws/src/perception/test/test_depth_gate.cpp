// depth_gate — scan_cam_gate.py near_zeroed와 같은 결과인지 (16UC1 mm · 32FC1 m · 그 밖 인코딩 · 행 끝 여백)
#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <limits>

#include "perception/depth_gate.hpp"

using perception::zero_near_depth;
using sensor_msgs::msg::Image;

namespace {
Image make(const std::string & enc, uint32_t w, uint32_t h, uint32_t step) {
  Image m;
  m.encoding = enc;
  m.width = w;
  m.height = h;
  m.step = step;
  m.data.assign(size_t(step) * h, 0xAB);  // 행 끝 여백은 0xAB 그대로 남아야 함
  return m;
}
}  // namespace

TEST(DepthGate, Mm16) {
  Image m = make("16UC1", 4, 2, 10);  // 한 행 8 B + 여백 2 B
  const uint16_t v[2][4] = {{0, 100, 499, 500}, {501, 3000, 1, 65535}};
  for (int r = 0; r < 2; ++r) std::memcpy(m.data.data() + r * 10, v[r], 8);
  EXPECT_EQ(zero_near_depth(m, 0.5), 3);  // 100 · 499 · 1 (0은 이미 측정 없음 · 500은 경계 — 파이썬 d < limit)
  const uint16_t want[2][4] = {{0, 0, 0, 500}, {501, 3000, 0, 65535}};
  for (int r = 0; r < 2; ++r) {
    EXPECT_EQ(std::memcmp(m.data.data() + r * 10, want[r], 8), 0);
    EXPECT_EQ(m.data[r * 10 + 8], 0xAB);
  }
}

TEST(DepthGate, Meters32) {
  Image m = make("32FC1", 3, 1, 12);
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float v[3] = {0.2f, nan, 0.6f};
  std::memcpy(m.data.data(), v, 12);
  EXPECT_EQ(zero_near_depth(m, 0.5), 1);
  float got[3];
  std::memcpy(got, m.data.data(), 12);
  EXPECT_EQ(got[0], 0.0f);
  EXPECT_TRUE(std::isnan(got[1]));
  EXPECT_FLOAT_EQ(got[2], 0.6f);
}

TEST(DepthGate, OtherEncodingUntouched) {
  Image m = make("rgb8", 2, 2, 6);
  const auto before = m.data;
  EXPECT_EQ(zero_near_depth(m, 0.5), -1);
  EXPECT_EQ(m.data, before);
  Image short_data = make("16UC1", 4, 2, 8);
  short_data.data.resize(8);  // 크기가 모자란 메시지 — 건드리지 않음
  EXPECT_EQ(zero_near_depth(short_data, 0.5), -1);
}
