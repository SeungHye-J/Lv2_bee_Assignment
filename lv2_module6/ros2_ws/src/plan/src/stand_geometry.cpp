// 세우기 경로 — 설명은 include/plan/stand_geometry.hpp (식은 isaac/lab/right_env.py plan_waypoints mode "tilt")
#include "plan/stand_geometry.hpp"

#include <algorithm>
#include <cmath>

namespace plan {

geometry_msgs::msg::Quaternion top_down_q(double psi) {
  geometry_msgs::msg::Quaternion q;
  q.x = std::cos(psi / 2);
  q.y = std::sin(psi / 2);
  q.z = 0.0;
  q.w = 0.0;
  return q;
}

bool top_down_yaw(const geometry_msgs::msg::Quaternion & q, double & psi) {
  if (std::abs(q.z) > 1e-6 || std::abs(q.w) > 1e-6 || std::hypot(q.x, q.y) < 0.5) return false;
  psi = 2.0 * std::atan2(q.y, q.x);
  return true;
}

bool object_yaw(const perception::msg::TrackedObject & o, double & yaw) {
  if (detail(o, "yaw_period").empty()) return false;  // perception은 방향을 알 때만 이 칸을 넣는다
  const auto & q = o.pose.pose.orientation;
  yaw = 2.0 * std::atan2(q.z, q.w);
  return true;
}

bool is_lying(const perception::msg::TrackedObject & o) {
  return o.class_name == "puck_lying" || detail(o, "posture") == "lying";
}

double far_axis_yaw(double axis_yaw, double cx, double cy, double rx, double ry) {
  const double ux = std::cos(axis_yaw), uy = std::sin(axis_yaw);
  return (ux * (cx - rx) + uy * (cy - ry)) >= 0 ? axis_yaw : std::atan2(-uy, -ux);
}

double support_top_z(const perception::msg::TrackedObject & puck, const std::vector<perception::msg::TrackedObject> & objects,
                     double puck_w) {
  const auto & p = puck.pose.pose.position;
  for (const auto & b : objects) {
    if (b.class_name != "start_box" && b.class_name != "location_box") continue;
    // 상자 윗면 사각형(가운데 · size · yaw) 안인가 — perception 상자 위치 = 윗면 가운데, z = 윗면 높이
    double yaw = 0.0;
    object_yaw(b, yaw);
    const double dx = p.x - b.pose.pose.position.x, dy = p.y - b.pose.pose.position.y;
    const double lx = dx * std::cos(yaw) + dy * std::sin(yaw), ly = -dx * std::sin(yaw) + dy * std::cos(yaw);
    if (std::abs(lx) <= b.size.x / 2 && std::abs(ly) <= b.size.y / 2) return b.pose.pose.position.z;
  }
  const double h = (puck.size.z > 0.01 && puck.size.z < 2 * puck_w) ? puck.size.z : puck_w;  // 잰 높이가 그럴듯하면
  return p.z - h / 2;
}

double snap_support(double z, const std::vector<double> & levels, double max_snap) {
  double best = z, d = max_snap;
  for (double l : levels) if (std::abs(z - l) <= d) { d = std::abs(z - l); best = l; }
  return best;
}

StandPath stand_path(double cx, double cy, double support_z, double axis_yaw, bool is_box, const StandParams & p) {
  StandPath out;
  const double ux = std::cos(axis_yaw), uy = std::sin(axis_yaw);
  // 닫힘축(ψ − 90°)이 긴 축에 직각 → ψ = axis_yaw (손이 대칭이라 ψ + π도 같은 파지 — coordinator가 닿는 쪽을 고름)
  out.tool_yaw = std::atan2(uy, ux);
  const auto q = top_down_q(out.tool_yaw);
  const double a = p.puck_l - p.pinch_from_end;    // 축(가까운 끝 아래 모서리) ~ 잡는 점, 긴 축 방향
  const double b = is_box ? p.contact_h_box : p.contact_h_cyl;  // 잡는 점 높이
  const double px = cx - ux * p.puck_l / 2, py = cy - uy * p.puck_l / 2;  // 축 = 가까운 끝 아래 모서리
  auto on_arc = [&](double f, double h) {
    geometry_msgs::msg::Pose o;
    const double s = a * std::cos(f) - b * std::sin(f);
    o.position.x = px + ux * s;
    o.position.y = py + uy * s;
    o.position.z = support_z + a * std::sin(f) + b * std::cos(f) + p.ee_up + h;
    o.orientation = q;
    return o;
  };
  if (p.drag_m > 0.0) {  // 호 없이 곧게 끌기: 먼 끝 위에서 잡은 채 가까운 쪽(−u)으로 drag_m 수평 이동 → 눌러 놓기 → 들어 올림
    out.pinch = on_arc(0.0, 0.0);
    out.above = out.pinch;
    out.above.position.z += p.above_m;
    const int nd = std::max(1, static_cast<int>(std::ceil(p.drag_m / 0.01)));  // 1 cm마다 점
    for (int k = 1; k <= nd; ++k) {
      const double f = static_cast<double>(k) / nd;
      geometry_msgs::msg::Pose o = out.pinch;
      o.position.x -= ux * p.drag_m * f;
      o.position.y -= uy * p.drag_m * f;
      o.position.z += p.drag_lift_m * f;
      out.arc_up.push_back(o);
    }
    auto set = out.arc_up.back();
    set.position.z += p.set_dz_m;  // 눌러 놓기
    out.arc_back.push_back(set);
    out.lift = set;
    out.lift.position.z += p.lift_away_m;
    return out;
  }
  const double end = p.arc_end_deg * M_PI / 180.0, step = p.step_deg * M_PI / 180.0;
  const int n = static_cast<int>(std::round(end / step));
  auto rise = [&](double f) { return end > 0 ? p.tilt_over_m * f / end : 0.0; };
  out.pinch = on_arc(0.0, 0.0);
  out.above = out.pinch;
  out.above.position.z += p.above_m;
  for (int k = 1; k <= n; ++k) {
    const double f = end * k / n;
    out.arc_up.push_back(on_arc(f, rise(f)));
  }
  for (int k = n - 1; k >= 0; --k) {  // 호 끝에서 90°까지 되돌아옴 (높이는 호 끝 그대로)
    const double f = end * k / n;
    if (f < M_PI / 2 - 1e-9) break;
    out.arc_back.push_back(on_arc(f, rise(end)));
  }
  out.arc_back.push_back(on_arc(M_PI / 2, p.set_dz_m));  // 내려놓기
  out.lift = out.arc_back.back();
  out.lift.position.z += p.lift_away_m;
  return out;
}

}  // namespace plan
