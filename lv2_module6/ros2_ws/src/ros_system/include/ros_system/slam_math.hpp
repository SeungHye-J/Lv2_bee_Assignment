// cuvslam_slam C++판의 계산 — 평면 자세 · map→odom · MapGate · 가까운 depth 마스크 (11-16, scripts/cuvslam_slam.py와 같은 식)
//   cuVSLAM·ROS 없이 시험 (test/test_slam_math.cpp — test_map_gate.py와 같은 시나리오)
#pragma once

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>

namespace ros_system {

struct Pose2 {
  double x = 0.0, y = 0.0, yaw = 0.0;
};

// 쿼터니언(x, y, z, w) · 이동 → 4x4
inline Eigen::Matrix4d mat(double qx, double qy, double qz, double qw, double tx, double ty, double tz) {
  Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
  m.block<3, 3>(0, 0) = Eigen::Quaterniond(qw, qx, qy, qz).normalized().toRotationMatrix();
  m(0, 3) = tx;
  m(1, 3) = ty;
  m(2, 3) = tz;
  return m;
}

// 4x4 → (x, y, yaw) — z·roll·pitch는 버림
inline Pose2 planar(const Eigen::Matrix4d & m) { return {m(0, 3), m(1, 3), std::atan2(m(1, 0), m(0, 0))}; }

// cuVSLAM world = 출발 때 카메라(OpenCV 축) → 출발 때 base_footprint 기준 지금 base_footprint. map_from_base = B·W·B⁻¹
inline Pose2 base_pose(const Eigen::Matrix4d & base_from_cam, const Eigen::Matrix4d & world_from_cam) {
  return planar(base_from_cam * world_from_cam * base_from_cam.inverse());
}

// 평면 자세 a·b (yaw는 −π~π로)
inline Pose2 compose(const Pose2 & a, const Pose2 & b) {
  const double c = std::cos(a.yaw), s = std::sin(a.yaw), yaw = a.yaw + b.yaw;
  return {a.x + c * b.x - s * b.y, a.y + s * b.x + c * b.y, std::atan2(std::sin(yaw), std::cos(yaw))};
}

inline Pose2 inverse(const Pose2 & a) {
  const double c = std::cos(a.yaw), s = std::sin(a.yaw);
  return {-c * a.x - s * a.y, s * a.x - c * a.y, -a.yaw};
}

// map_from_odom = map_from_base · odom_from_base⁻¹ (평면)
inline Pose2 map_to_odom(const Pose2 & base, const Pose2 & odom) {
  const double a = base.yaw - odom.yaw, c = std::cos(a), s = std::sin(a);
  return {base.x - (c * odom.x - s * odom.y), base.y - (s * odom.x + c * odom.y), a};
}

// 두 평면 자세 사이 속도를 base_footprint 기준으로 (vx, vy, vyaw) — EKF에 넣는 값
inline Pose2 body_twist(const Pose2 & prev, const Pose2 & cur, double dt) {
  const double dx = cur.x - prev.x, dy = cur.y - prev.y, c = std::cos(prev.yaw), s = std::sin(prev.yaw);
  double dyaw = std::fmod(cur.yaw - prev.yaw + M_PI, 2 * M_PI);
  if (dyaw < 0) dyaw += 2 * M_PI;
  dyaw -= M_PI;
  return {(c * dx + s * dy) / dt, (-s * dx + c * dy) / dt, dyaw / dt};
}

// 도면 보정 조각 corr → 새 map_offset = corr⁻¹ ∘ map_offset (모든 map 자세가 corr⁻¹만큼 옮겨져 도면 물체가 제자리에)
inline Pose2 apply_map_correction(const Pose2 & map_offset, const Pose2 & corr) { return compose(inverse(corr), map_offset); }

// map→odom 보정 거르기 — 설명은 scripts/cuvslam_slam.py MapGate (같은 동작)
class MapGate {
public:
  MapGate(double jump_m = 0.30, double jump_deg = 15.0, double rate_m = 0.05, double rate_deg = 5.0, double settle_s = 0.0)
  : jump_m_(jump_m), jump_rad_(jump_deg * M_PI / 180.0), rate_m_(rate_m), rate_rad_(rate_deg * M_PI / 180.0), settle_s_(settle_s) {}

  void reset() {
    held_.reset();
    t_.reset();
    locked = false;
  }

  // 후보 map→odom, 시각 t [s], 지금 odom 자세 → 낼 map→odom (accepted = 후보를 그대로 받았는지)
  //   rotating (11-16): 제자리·빠른 회전 중이면 지금 값을 그대로 둠 — 회전 중 cuVSLAM·EKF yaw가 서로 늦게 따라와 생기는
  //   일시적 차이를 보정으로 넣지 않음 (멈춘 뒤 다시 따라감, 잠금·거름 수는 안 셈)
  Pose2 update(const Pose2 & cand, double t, const Pose2 & odom, bool & accepted, bool rotating = false) {
    if (!t0_) t0_ = t;
    if (rotating && held_) {
      t_ = t;
      ++held_rot;
      accepted = false;
      return *held_;
    }
    if (!held_) {
      held_ = cand;
      t_ = t;
      accepted = true;
      return *held_;
    }
    const double dt = std::max(t - *t_, 1e-3);
    t_ = t;
    const Pose2 ph = comp(*held_, odom), pc = comp(cand, odom);  // 지금 보정 · 후보 보정에서의 로봇 map 자세
    const double d = std::hypot(pc.x - ph.x, pc.y - ph.y), r = std::abs(std::remainder(pc.yaw - ph.yaw, 2 * M_PI));
    if (t - *t0_ < settle_s_) {  // 출발 정착 중 — 그대로 받음
      if (d > jump_m_ || r > jump_rad_) resettled = Pose2{d, r * 180.0 / M_PI, 0.0};
      held_ = cand;
      accepted = true;
      return *held_;
    }
    if (locked || d > jump_m_ || r > jump_rad_) {
      locked = true;
      ++rejected;
      accepted = false;
      return *held_;
    }
    const double k = std::min({1.0, d > 1e-9 ? rate_m_ * dt / d : 1.0, r > 1e-9 ? rate_rad_ * dt / r : 1.0});
    if (k >= 1.0) {
      held_ = cand;
      accepted = true;
      return *held_;
    }
    // 로봇 자세를 k만큼 옮기고, 그 자세가 되게 하는 map→odom으로 (회전은 로봇 자리 기준)
    const Pose2 p{ph.x + k * (pc.x - ph.x), ph.y + k * (pc.y - ph.y), ph.yaw + k * std::remainder(pc.yaw - ph.yaw, 2 * M_PI)};
    held_ = comp(p, inv(odom));
    ++rejected;
    accepted = false;
    return *held_;
  }

  // 2D a∘b — yaw를 감싸지 않음 (Python MapGate._compose와 같게)
  static Pose2 comp(const Pose2 & a, const Pose2 & b) {
    const double c = std::cos(a.yaw), s = std::sin(a.yaw);
    return {a.x + c * b.x - s * b.y, a.y + s * b.x + c * b.y, a.yaw + b.yaw};
  }
  static Pose2 inv(const Pose2 & a) {
    const double c = std::cos(a.yaw), s = std::sin(a.yaw);
    return {-c * a.x - s * a.y, s * a.x - c * a.y, -a.yaw};
  }

  bool locked = false;
  long rejected = 0;
  long held_rot = 0;  // 회전 중이라 그대로 둔 프레임 수
  std::optional<Pose2> resettled;  // 정착 중 다시 맞춘 크기 (x = 거리 m, y = 각 °) — 노드가 로그하고 지움

private:
  double jump_m_, jump_rad_, rate_m_, rate_rad_, settle_s_;
  std::optional<Pose2> held_;
  std::optional<double> t_, t0_;
};

// depth 원값 < min_depth·scale ⇔ ≤ 이 값 (정수)
inline uint16_t near_limit(double min_depth, double depth_scale) {
  return static_cast<uint16_t>(std::max(std::ceil(min_depth * depth_scale) - 1.0, 0.0));
}

// uint16 depth → uint8 마스크 (1 = 0 < depth ≤ lim, 추적에서 뺌). depth − 1을 uint16으로 빼면 0은 65535로 넘어가 한 번 비교로 끝남
inline void near_mask(const uint16_t * depth, uint8_t * mask, std::size_t n, uint16_t lim) {
  for (std::size_t i = 0; i < n; ++i) mask[i] = static_cast<uint16_t>(depth[i] - 1u) < lim;
}

}  // namespace ros_system
