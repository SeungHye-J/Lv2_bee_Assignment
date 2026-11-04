// RL 정책 구간 시간표 — 설명은 include/plan/rl_segment.hpp
#include "plan/rl_segment.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace plan {

namespace {
constexpr double VMAX = 3.0, AMAX = 5.0;  // bee_common VMAX · AMAX (팀 관절 한계)

double max_abs_diff(const std::vector<double> & a, const std::vector<double> & b) {
  double m = 0.0;
  for (size_t i = 0; i < a.size() && i < b.size(); ++i) m = std::max(m, std::abs(a[i] - b[i]));
  return m;
}

std::string list(const std::vector<double> & v) {
  std::string s = "[";
  char buf[32];
  for (size_t i = 0; i < v.size(); ++i) {
    std::snprintf(buf, sizeof(buf), "%s%.6f", i ? "," : "", v[i]);
    s += buf;
  }
  return s + "]";
}
}  // namespace

double trapezoid_duration(double dist, double vel_scale) {
  const double vmax = VMAX * vel_scale, amax = AMAX * vel_scale, t_acc = vmax / amax;
  if (dist <= vmax * t_acc) return std::max(2.0 * std::sqrt(dist / amax), 0.1);
  return dist / vmax + t_acc;
}

double move_s(double t, double dur) {
  const double x = std::clamp(dur > 0 ? t / dur : 1.0, 0.0, 1.0);
  return x * x * (3 - 2 * x);
}

RlTimeline rl_timeline(const std::vector<double> & q0, const std::vector<double> & q_above,
                       const std::vector<double> & q_down, const RlTimes & times) {
  RlTimeline tl;
  tl.t_tr = trapezoid_duration(max_abs_diff(q0, q_above), times.vel_scale);
  tl.t_mv = trapezoid_duration(max_abs_diff(q_above, q_down), times.vel_scale);
  const double seg[] = {tl.t_tr + times.hold, tl.t_mv, times.settle, times.act, tl.t_mv, times.wait};
  double acc = 0.0;
  for (double s : seg) tl.t_ph.push_back(acc += s);
  return tl;
}

int rl_phase(double t, const RlTimeline & tl) {
  return static_cast<int>(std::count_if(tl.t_ph.begin(), tl.t_ph.end(), [t](double e) { return t >= e; }));
}

std::vector<double> rl_ref(double t, const std::vector<double> & q0, const std::vector<double> & q_above,
                           const std::vector<double> & q_down, const RlTimeline & tl) {
  std::vector<double> q(q_above.size());
  const int ph = rl_phase(t, tl);
  for (size_t i = 0; i < q.size(); ++i) {
    if (t < tl.t_tr) q[i] = q0[i] + move_s(t, tl.t_tr) * (q_above[i] - q0[i]);                         // 이동
    else if (ph >= 3) q[i] = q_down[i] + move_s(t - tl.t_ph[3], tl.t_mv) * (q_above[i] - q_down[i]);  // 닫기·놓기 뒤 올라감
    else q[i] = q_above[i] + move_s(t - tl.t_ph[0], tl.t_mv) * (q_down[i] - q_above[i]);              // 멈춤 · 내려감 · 멈춤
  }
  return q;
}

std::vector<std::pair<double, std::vector<double>>> joint_path(const std::vector<std::vector<double>> & pts, double vel_scale,
                                                               double dt, double min_s) {
  std::vector<std::pair<double, std::vector<double>>> out;
  if (pts.size() < 2) return out;
  std::vector<double> cum{0.0};
  for (size_t k = 1; k < pts.size(); ++k) cum.push_back(cum.back() + max_abs_diff(pts[k - 1], pts[k]));
  const double total = cum.back();
  const double dur = std::max(trapezoid_duration(total, vel_scale), min_s);  // 길이 0 = 멈춤 (min_s 동안 그 자리)
  for (double t = dt; t < dur + dt / 2; t += dt) {
    const double tt = std::min(t, dur), d = move_s(tt, dur) * total;
    size_t k = 1;
    while (k + 1 < pts.size() && cum[k] < d) ++k;
    const double seg = cum[k] - cum[k - 1], fr = seg > 1e-12 ? std::clamp((d - cum[k - 1]) / seg, 0.0, 1.0) : 1.0;
    std::vector<double> q(pts[k].size());
    for (size_t i = 0; i < q.size(); ++i) q[i] = pts[k - 1][i] + fr * (pts[k][i] - pts[k - 1][i]);
    out.push_back({tt, q});
  }
  return out;
}

std::string rl_context_json(const std::string & skill, double t0, const RlTimeline & tl, const std::vector<double> & q_above,
                            const std::vector<double> & q_down, int target_id, bool is_box, double tool_yaw,
                            const std::vector<double> & cell) {
  if (skill.empty()) return "{\"skill\": \"\"}";
  char head[256];
  std::snprintf(head, sizeof(head), "{\"skill\": \"%s\", \"t0\": %.6f, \"t_tr\": %.6f, \"target_id\": %d, \"is_box\": %s, \"tool_yaw\": %.6f",
                skill.c_str(), t0, tl.t_tr, target_id, is_box ? "true" : "false", tool_yaw);
  return std::string(head) + ", \"t_ph\": " + list(tl.t_ph) + ", \"q_above\": " + list(q_above) +
         ", \"q_down\": " + list(q_down) + ", \"cell\": " + list(cell) + "}";
}

}  // namespace plan
