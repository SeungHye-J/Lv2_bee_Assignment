#pragma once
// 가르친 경로 (teach & repeat, 11-15) — 사람이 수동 조작으로 몬 경로(tools/field/route_record.py가 arena 좌표로 저장)를
//   navigate가 경유점으로 씀. 플래너가 costmap 값에 따라 벽에 붙거나 대각선으로 질러 멈추던 것 대신 사람이 고른 길로.
//   경로 파일 = 한 줄에 "x y" (arena, m) · '#' 줄은 설명. 시작·끝이 로봇·목표 근처인 경로를 고름 (거꾸로도 — 복귀)
//   쓰는 쪽: 꺾이는 점만 남김(Douglas-Peucker) → 점마다 다음 점을 보는 방향으로 NavigateToPose (문 경유점과 같은 방식)
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace skills {

using Pt = std::pair<double, double>;

// "x y" 줄 → 점들 (못 읽으면 빈 목록)
inline std::vector<Pt> load_route(const std::string & file) {
  std::vector<Pt> pts;
  std::ifstream f(file);
  for (std::string line; std::getline(f, line);) {
    if (line.empty() || line[0] == '#') continue;
    std::istringstream in(line);
    double x, y;
    if (in >> x >> y) pts.emplace_back(x, y);
  }
  return pts;
}

// 꺾이는 점만 남김 — 선분에서 tol보다 먼 점이 없으면 양 끝만
inline void simplify_into(const std::vector<Pt> & p, size_t a, size_t b, double tol, std::vector<Pt> & out) {
  double best = -1.0;
  size_t k = a;
  const double dx = p[b].first - p[a].first, dy = p[b].second - p[a].second, len = std::hypot(dx, dy);
  for (size_t i = a + 1; i < b; ++i) {
    const double ex = p[i].first - p[a].first, ey = p[i].second - p[a].second;
    const double d = len < 1e-9 ? std::hypot(ex, ey) : std::abs(dx * ey - dy * ex) / len;
    if (d > best) { best = d; k = i; }
  }
  if (best > tol) {
    simplify_into(p, a, k, tol, out);
    simplify_into(p, k, b, tol, out);
  } else {
    out.push_back(p[b]);
  }
}

inline std::vector<Pt> simplify(const std::vector<Pt> & p, double tol) {
  if (p.size() < 3) return p;
  std::vector<Pt> out{p.front()};
  simplify_into(p, 0, p.size() - 1, tol, out);
  return out;
}

// 로봇(r)에서 목표(g)로 가는 경유점 — 경로 시작이 r에서 start_m 안 · 끝이 g에서 goal_m 안인 것 (거꾸로도).
//   여러 개면 두 거리 합이 가장 작은 것. 로봇에 가장 가까운 점부터 쓰고, 목표에서 goal_m 안의 점은 뺌 (목표는 쓰는 쪽이 붙임).
//   없으면 빈 목록. routes와 r · g는 같은 좌표계(map)여야 함
inline std::vector<Pt> pick_route(const std::vector<std::vector<Pt>> & routes, Pt r, Pt g,
                                  double start_m, double goal_m, double tol) {
  auto dist = [](Pt a, Pt b) { return std::hypot(a.first - b.first, a.second - b.second); };
  const std::vector<Pt> * best = nullptr;
  bool rev = false;
  double score = 1e9;
  for (const auto & rt : routes) {
    if (rt.size() < 2) continue;
    for (bool back : {false, true}) {
      const Pt s = back ? rt.back() : rt.front(), e = back ? rt.front() : rt.back();
      const double ds = dist(s, r), de = dist(e, g);
      if (ds <= start_m && de <= goal_m && ds + de < score) { score = ds + de; best = &rt; rev = back; }
    }
  }
  if (!best) return {};
  std::vector<Pt> p(*best);
  if (rev) p.assign(best->rbegin(), best->rend());
  size_t k0 = 0;  // 로봇에 가장 가까운 점부터
  for (size_t i = 1; i < p.size(); ++i)
    if (dist(p[i], r) < dist(p[k0], r)) k0 = i;
  p.erase(p.begin(), p.begin() + static_cast<long>(k0));
  std::vector<Pt> legs;
  const auto s = simplify(p, tol);
  for (size_t i = 1; i < s.size(); ++i)  // 첫 점 = 로봇 자리라 뺌
    if (dist(s[i], g) > goal_m && dist(s[i], r) > 0.15) legs.push_back(s[i]);
  return legs;
}

}  // namespace skills
