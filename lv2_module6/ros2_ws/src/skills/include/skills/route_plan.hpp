#pragma once
// 가르친 경로 (teach & repeat, 11-15) — 사람이 수동 조작으로 몬 경로(tools/field/route_record.py가 arena 좌표로 저장)를
//   navigate가 경유점으로 씀. 플래너가 costmap 값에 따라 벽에 붙거나 대각선으로 질러 멈추던 것 대신 사람이 고른 길로.
//   경로 파일 = 한 줄에 "x y" (arena, m) · '#' 줄은 설명. 시작·끝이 로봇·목표 근처인 경로를 고름 (거꾸로도 — 복귀)
//   쓰는 쪽: 꺾이는 점만 남김(Douglas-Peucker) → 점마다 다음 점을 보는 방향으로 NavigateToPose (문 경유점과 같은 방식)
#include <algorithm>
#include <cmath>
#include <cstdlib>
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

namespace skills {

// ---------- 움직임 정답표 재생 (11-15) — tools/cli/op/routerec.py가 바퀴+IMU odom으로 남긴 <이름>_NN_<구간>.motion.csv
//   한 줄 = action,value: turn = 도 (왼쪽 +) · forward = m. 재생은 skill_navigate가 Nav2 Spin · DriveOnHeading으로
struct Move { bool turn; double value; };  // turn: rad · forward: m

inline std::vector<Move> load_motion(const std::string & file) {
  std::vector<Move> out;
  std::ifstream f(file);
  for (std::string line; std::getline(f, line);) {
    const auto c = line.find(',');
    if (line.empty() || line[0] == '#' || c == std::string::npos) continue;
    const std::string a = line.substr(0, c);
    char * end = nullptr;
    const double v = std::strtod(line.c_str() + c + 1, &end);
    if (end == line.c_str() + c + 1) continue;  // 머리줄 "action,value"
    if (a == "turn") out.push_back({true, v * M_PI / 180.0});
    else if (a == "forward") out.push_back({false, v});
  }
  return out;
}

// 사람이 준 직진 속도의 중앙값 (.cmd.csv: t,linear,angular) — 0.02 m/s 넘는 전진만. 없으면 fallback
inline double median_speed(const std::string & cmd_file, double fallback) {
  std::vector<double> v;
  std::ifstream f(cmd_file);
  for (std::string line; std::getline(f, line);) {
    if (line.empty() || line[0] == '#' || line[0] == 't') continue;
    std::istringstream in(line);
    std::string t, lin;
    if (std::getline(in, t, ',') && std::getline(in, lin, ',')) {
      const double x = std::strtod(lin.c_str(), nullptr);
      if (x > 0.02) v.push_back(x);
    }
  }
  if (v.empty()) return fallback;
  std::sort(v.begin(), v.end());
  return v[v.size() / 2];
}

struct Pose2d { double x, y, yaw; };

// 한 동작 뒤 기대 자세 (odom)
inline Pose2d after(const Pose2d & p, const Move & m) {
  if (m.turn) return {p.x, p.y, p.yaw + m.value};
  return {p.x + m.value * std::cos(p.yaw), p.y + m.value * std::sin(p.yaw), p.yaw};
}

// 기대 자세에서 얼마나 벗어났나 — 위치 [m] · 방향 [rad] (절댓값)
inline std::pair<double, double> deviation(const Pose2d & expect, const Pose2d & actual) {
  return {std::hypot(actual.x - expect.x, actual.y - expect.y),
          std::abs(std::remainder(actual.yaw - expect.yaw, 2 * M_PI))};
}

// 표 → 경로 점들 (구간 출발 자세 p에서, step 간격) — 회전은 그 자리 점의 방향만 바꿈, 마지막 점 방향 = 끝 방향.
//   follow 모드: 이 경로를 Nav2 FollowPath로 — 컨트롤러가 local costmap(벽·장애물)을 보며 따라감 (11-15)
inline std::vector<Pose2d> path_points(Pose2d p, const std::vector<Move> & moves, double step = 0.05) {
  std::vector<Pose2d> out{p};
  for (const auto & m : moves) {
    if (m.turn) {
      p.yaw += m.value;
      out.back().yaw = p.yaw;
      continue;
    }
    const int n = std::max(1, static_cast<int>(std::ceil(m.value / step - 1e-9)));
    for (int k = 0; k < n; ++k) {
      p.x += m.value / n * std::cos(p.yaw);
      p.y += m.value / n * std::sin(p.yaw);
      out.push_back(p);
    }
  }
  return out;
}

// 경로 끝을 목표(goal)에 맞춤 — 시작은 그대로, 끝 오차(목표 − 끝)를 걸어온 길이 비율만큼 나눠 더함 · 마지막 방향 = 목표 방향.
//   녹화 odom 오차가 끝으로 쌓여 6 m 구간 끝이 0.37 m 벗어나 벽 모서리에 걸렸음 (시뮬 11-15) — 목표는 mission_manager가 벽 여유를 두고 정한 자리
inline void anchor_end(std::vector<Pose2d> & p, const Pose2d & goal) {
  if (p.size() < 2) return;
  double total = 0.0;
  for (size_t k = 1; k < p.size(); ++k) total += std::hypot(p[k].x - p[k - 1].x, p[k].y - p[k - 1].y);
  if (total < 1e-6) return;
  const double ex = goal.x - p.back().x, ey = goal.y - p.back().y;
  double run = 0.0;
  std::vector<Pose2d> q(p);
  for (size_t k = 1; k < p.size(); ++k) {
    run += std::hypot(q[k].x - q[k - 1].x, q[k].y - q[k - 1].y);
    p[k].x += ex * run / total;
    p[k].y += ey * run / total;
  }
  for (size_t k = 1; k + 1 < p.size(); ++k) {  // 방향 = 다음 점 쪽 (옮긴 뒤 다시)
    const double dx = p[k + 1].x - p[k].x, dy = p[k + 1].y - p[k].y;
    if (std::hypot(dx, dy) > 1e-6) p[k].yaw = std::atan2(dy, dx);
  }
  p.back().yaw = goal.yaw;
}

}  // namespace skills
