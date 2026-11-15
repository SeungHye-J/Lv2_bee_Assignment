#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>

#include "skills/route_plan.hpp"

using skills::Pt;

// ㄱ자 경로 (0,0) → (1,0) → (1,1), 5 cm 간격 + 2 cm 흔들림
static std::vector<Pt> ell() {
  std::vector<Pt> p;
  for (int i = 0; i <= 20; ++i) p.emplace_back(i * 0.05, (i % 2) * 0.02);
  for (int i = 1; i <= 20; ++i) p.emplace_back(1.0 + (i % 2) * 0.02, i * 0.05);
  return p;
}

TEST(RoutePlan, CornerOnly) {
  const auto legs = skills::pick_route({ell()}, {0.0, 0.0}, {1.0, 1.0}, 0.5, 0.4, 0.08);
  ASSERT_EQ(legs.size(), 1u);  // 꺾이는 점 하나 (끝은 목표라 뺌)
  EXPECT_NEAR(legs[0].first, 1.0, 0.05);
  EXPECT_NEAR(legs[0].second, 0.0, 0.05);
}

TEST(RoutePlan, ReverseForReturn) {
  const auto legs = skills::pick_route({ell()}, {1.0, 1.0}, {0.0, 0.0}, 0.5, 0.4, 0.08);
  ASSERT_EQ(legs.size(), 1u);
  EXPECT_NEAR(legs[0].first, 1.0, 0.05);
}

TEST(RoutePlan, NoMatch) {
  EXPECT_TRUE(skills::pick_route({ell()}, {3.0, 3.0}, {1.0, 1.0}, 0.5, 0.4, 0.08).empty());   // 로봇이 멂
  EXPECT_TRUE(skills::pick_route({ell()}, {0.0, 0.0}, {2.0, 2.0}, 0.5, 0.4, 0.08).empty());   // 목표가 다름
}

TEST(RoutePlan, StartsFromNearestPoint) {  // 경로 중간에서 출발 — 지난 꺾임은 안 감
  const auto legs = skills::pick_route({ell()}, {1.0, 0.3}, {1.0, 1.0}, 0.5, 0.4, 0.08);
  EXPECT_TRUE(legs.empty());
}

TEST(RoutePlan, LoadFile) {
  const char * f = "/tmp/test_route_plan.txt";
  std::ofstream(f) << "# 설명\n0 0\n1 0\n\n1 1\n";
  const auto p = skills::load_route(f);
  std::remove(f);
  ASSERT_EQ(p.size(), 3u);
  EXPECT_DOUBLE_EQ(p[2].second, 1.0);
}
