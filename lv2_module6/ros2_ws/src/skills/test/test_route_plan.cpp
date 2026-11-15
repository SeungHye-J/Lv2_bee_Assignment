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

TEST(RoutePlan, MotionTable) {
  const char * f = "/tmp/test_route_motion.csv";
  std::ofstream(f) << "# 설명\naction,value\nforward,1.0\nturn,90\nforward,0.5\n";
  const auto m = skills::load_motion(f);
  std::remove(f);
  ASSERT_EQ(m.size(), 3u);
  EXPECT_NEAR(m[1].value, M_PI / 2, 1e-9);
  skills::Pose2d p{0.0, 0.0, 0.0};
  for (const auto & mv : m) p = skills::after(p, mv);
  EXPECT_NEAR(p.x, 1.0, 1e-9);
  EXPECT_NEAR(p.y, 0.5, 1e-9);
  const auto d = skills::deviation(p, {1.0, 0.8, M_PI / 2 + 0.1});
  EXPECT_NEAR(d.first, 0.3, 1e-9);
  EXPECT_NEAR(d.second, 0.1, 1e-9);
}

TEST(RoutePlan, MedianSpeed) {
  const char * f = "/tmp/test_route_cmd.csv";
  std::ofstream(f) << "# 설명\nt,linear_mps,angular_radps\n0,0.0,0.8\n1,0.15,0\n2,0.2,0\n3,0.15,0\n4,0,0\n";
  EXPECT_DOUBLE_EQ(skills::median_speed(f, 0.1), 0.15);
  std::remove(f);
  EXPECT_DOUBLE_EQ(skills::median_speed("/nonexistent", 0.1), 0.1);
}

TEST(RoutePlan, PathPoints) {
  // 1 m 직진 → 왼쪽 90° → 0.5 m 직진 → 왼쪽 90° (끝 방향)
  const std::vector<skills::Move> m{{false, 1.0}, {true, M_PI / 2}, {false, 0.5}, {true, M_PI / 2}};
  const auto p = skills::path_points({0.0, 0.0, 0.0}, m, 0.1);
  ASSERT_EQ(p.size(), 16u);  // 시작 + 10 + 5
  EXPECT_NEAR(p[10].x, 1.0, 1e-9);
  EXPECT_NEAR(p[10].yaw, M_PI / 2, 1e-9);  // 꺾이는 점은 새 방향
  EXPECT_NEAR(p.back().y, 0.5, 1e-9);
  EXPECT_NEAR(p.back().yaw, M_PI, 1e-9);   // 끝 방향
}

TEST(RoutePlan, AnchorEnd) {
  // 직진 2 m 경로 끝이 (2, 0) — 목표 (2, 0.4) · 방향 90°로 맞춤: 가운데 점은 0.2 m 옮김, 끝은 목표 그대로
  std::vector<skills::Pose2d> p;
  for (int k = 0; k <= 20; ++k) p.push_back({k * 0.1, 0.0, 0.0});
  skills::anchor_end(p, {2.0, 0.4, M_PI / 2});
  EXPECT_NEAR(p.front().y, 0.0, 1e-9);
  EXPECT_NEAR(p[10].y, 0.2, 1e-9);
  EXPECT_NEAR(p.back().x, 2.0, 1e-9);
  EXPECT_NEAR(p.back().y, 0.4, 1e-9);
  EXPECT_NEAR(p.back().yaw, M_PI / 2, 1e-9);
}
