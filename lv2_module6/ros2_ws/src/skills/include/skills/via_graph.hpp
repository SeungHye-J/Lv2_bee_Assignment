#pragma once
// 경유점 지도 (11-15 사용자 지정) — 사람이 찍은 점(approach 시작 자리 + 갈림·통로 점)과 화살표로 navigate 경유점을 고름.
//   점끼리는 벽·상자에서 clearance 넘게 떨어진 직선이면 이어짐 (보이는 점끼리). 최단 경로라 같은 방향으로 이어지는 점은
//   저절로 빠짐 (B6app → L2top → L2right → W_mid 대신 B6app → W_mid).
//   화살표 a → b (한쪽만): b로 들어가는 길은 a에서만 — 그 방향으로 갈 때는 꼭 a를 거침. b에서 a로 되돌아가는 건 자유.
//   양쪽 화살표(a ↔ b)는 그냥 이어진 길 (들어가는 제한 없음). 로봇 · 목표가 점에서 snap_m 안이면 그 점으로 봄
#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <queue>
#include <set>
#include <utility>
#include <vector>

namespace skills {

struct ViaGraph {
  using P = std::pair<double, double>;
  std::vector<P> nodes;
  std::vector<std::pair<int, int>> arrows;     // 노드 번호 a → b
  std::vector<std::pair<P, P>> obstacles;      // 벽 · 상자 변 선분
  double clearance = 0.18;                     // [m] 이어지는 직선이 장애물에서 이만큼은 떨어져야
  double snap_m = 0.15;

  static double pt_seg(P p, P a, P b) {
    const double ax = b.first - a.first, ay = b.second - a.second, l = ax * ax + ay * ay;
    const double t = l == 0 ? 0 : std::max(0.0, std::min(1.0, ((p.first - a.first) * ax + (p.second - a.second) * ay) / l));
    return std::hypot(p.first - a.first - t * ax, p.second - a.second - t * ay);
  }
  static double cross(P o, P a, P b) { return (a.first - o.first) * (b.second - o.second) - (a.second - o.second) * (b.first - o.first); }
  bool clear(P p, P q) const {
    for (const auto & [a, b] : obstacles) {
      if ((cross(p, q, a) > 0) != (cross(p, q, b) > 0) && (cross(a, b, p) > 0) != (cross(a, b, q) > 0)) return false;
      if (std::min({pt_seg(p, a, b), pt_seg(q, a, b), pt_seg(a, p, q), pt_seg(b, p, q)}) < clearance) return false;
    }
    return true;
  }
  void add_box(double x, double y, double h) {
    const P c[4] = {{x - h, y - h}, {x + h, y - h}, {x + h, y + h}, {x - h, y + h}};
    for (int i = 0; i < 4; ++i) obstacles.push_back({c[i], c[(i + 1) % 4]});
  }

  // 로봇 r → 목표 g 사이 경유점 (r · g 자신은 뺌). 이어진 길이 없으면 nullopt (쓰는 쪽이 예전 방식으로)
  std::optional<std::vector<P>> route(P r, P g) const {
    const int n = static_cast<int>(nodes.size()), S = n, G = n + 1;
    auto near = [&](P p) {
      for (int i = 0; i < n; ++i)
        if (std::hypot(nodes[i].first - p.first, nodes[i].second - p.second) <= snap_m) return i;
      return -1;
    };
    const int s = near(r) >= 0 ? near(r) : S, t = near(g) >= 0 ? near(g) : G;
    std::vector<P> pos(nodes);
    pos.push_back(r);
    pos.push_back(g);
    std::set<std::pair<int, int>> arrow(arrows.begin(), arrows.end());
    std::vector<std::set<int>> forced(n + 2);  // 들어갈 수 있는 앞 점 (비면 제한 없음)
    for (const auto & [a, b] : arrows)
      if (!arrow.count({b, a})) forced[b].insert(a);
    for (const auto & [a, b] : arrows)
      if (!forced[a].empty()) forced[a].insert(b);  // 화살표 거꾸로 되돌아가기는 자유
    std::vector<double> dist(n + 2, std::numeric_limits<double>::infinity());
    std::vector<int> prev(n + 2, -1);
    using Q = std::pair<double, int>;
    std::priority_queue<Q, std::vector<Q>, std::greater<Q>> pq;
    dist[s] = 0;
    pq.push({0, s});
    while (!pq.empty()) {
      const auto [d, u] = pq.top();
      pq.pop();
      if (u == t) break;
      if (d > dist[u]) continue;
      for (int v = 0; v < n + 2; ++v) {
        if (v == u || v == S || (v == G && t != G)) continue;
        if (!arrow.count({u, v}) && (!(forced[v].empty() || forced[v].count(u)) || !clear(pos[u], pos[v]))) continue;
        const double nd = d + std::hypot(pos[u].first - pos[v].first, pos[u].second - pos[v].second);
        if (nd < dist[v]) { dist[v] = nd; prev[v] = u; pq.push({nd, v}); }
      }
    }
    if (s != t && prev[t] < 0) return std::nullopt;
    std::vector<P> out;
    for (int k = prev[t]; k >= 0 && k != s; k = prev[k]) out.insert(out.begin(), pos[k]);
    return out;
  }
};

}  // namespace skills
