#include <gtest/gtest.h>

#include "skills/via_graph.hpp"

// 벽 x = 1 (y −1 ~ 1) · 점 A(0, 1.5) M(1, 1.5) B(2, 1.5) C(2, 0) · 화살표 B → C
static skills::ViaGraph graph() {
  skills::ViaGraph g;
  g.nodes = {{0, 1.5}, {1, 1.5}, {2, 1.5}, {2, 0}};
  g.arrows = {{2, 3}};
  g.obstacles = {{{1, -1}, {1, 1}}};
  g.clearance = 0.3;
  return g;
}

TEST(ViaGraph, AroundWallSkipsStraightPoint) {
  const auto r = graph().route({0, 0}, {2, -0.5});
  ASSERT_TRUE(r);
  ASSERT_EQ(r->size(), 2u);  // A → B (M은 같은 직선이라 빠짐) — C는 화살표라 B에서만 들어가니 안 거침
  EXPECT_DOUBLE_EQ((*r)[0].first, 0);
  EXPECT_DOUBLE_EQ((*r)[1].first, 2);
}

TEST(ViaGraph, ArrowForcesEntry) {
  const auto r = graph().route({3, 0}, {2, 0});  // C가 바로 보여도 B를 거침
  ASSERT_TRUE(r);
  ASSERT_EQ(r->size(), 1u);
  EXPECT_DOUBLE_EQ((*r)[0].second, 1.5);
}

TEST(ViaGraph, BackAgainstArrowIsFree) {
  const auto r = graph().route({2, 0}, {3, 0});  // C에서 나갈 땐 제한 없음
  ASSERT_TRUE(r);
  EXPECT_TRUE(r->empty());
}

TEST(ViaGraph, NoRoute) {
  auto g = graph();
  g.obstacles.push_back({{-5, 3}, {5, 3}});
  g.obstacles.push_back({{-5, -3}, {5, -3}});
  g.obstacles.push_back({{1, 1}, {1, 3}});
  g.obstacles.push_back({{1, -1}, {1, -3}});
  EXPECT_FALSE(g.route({0, 0}, {2, -0.5}));
}
