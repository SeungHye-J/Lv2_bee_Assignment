// rl_nav_correction 구현 — 노드 설명·입출력·멤버 선언은 include/ml/rl_nav_correction.hpp, main은 rl_nav_correction_main.cpp
#include "ml/rl_nav_correction.hpp"

#include <filesystem>

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace ml {

RlNavCorrection::RlNavCorrection()
: rclcpp::Node("rl_nav_correction") {
  declare_parameter<bool>("enabled", true);
  max_v_ = declare_parameter<double>("max_vel_x", 0.26);        // Waffle Pi 최대 선속도
  max_w_ = declare_parameter<double>("max_vel_theta", 1.82);
  timeout_s_ = declare_parameter<double>("timeout_s", 0.2);
  limit_v_ = max_v_;
  cmd_sub_ = create_subscription<TwistStamped>(
    "/cmd_vel_nav", 10, [this](TwistStamped::ConstSharedPtr m) { on_cmd(*m); });
  plan_sub_ = create_subscription<nav_msgs::msg::Path>(
    "/plan", 1, [this](nav_msgs::msg::Path::ConstSharedPtr m) { plan_ = m; });
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10, [this](nav_msgs::msg::Odometry::ConstSharedPtr m) { odom_ = m; last_sensor_ = now(); });
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    "/imu", rclcpp::SensorDataQoS().keep_last(1), [this](sensor_msgs::msg::Imu::ConstSharedPtr m) { imu_ = m; });
  limit_sub_ = create_subscription<nav2_msgs::msg::SpeedLimit>(
    "/speed_limit", rclcpp::QoS(1).reliable(), [this](nav2_msgs::msg::SpeedLimit::ConstSharedPtr m) {
      const double v = m->percentage ? max_v_ * m->speed_limit / 100.0 : m->speed_limit;
      limit_v_ = (m->speed_limit <= 0.0) ? max_v_ : std::min(max_v_, v);  // 0 = 제한 없음
    });
  pub_ = create_publisher<TwistStamped>("/cmd_vel_corrected", rclcpp::QoS(1).reliable());
  // 도킹 모드 정책 — skill 이름 · 시뮬 제한 시간 [s] (drive_env TASKS) · 파일 model_dir/<skill>.policy (없으면 그 skill은 Δ = 0)
  res_v_ = declare_parameter<double>("residual_v", 0.02);
  res_w_ = declare_parameter<double>("residual_w", 0.1);
  std::string dir = declare_parameter<std::string>("model_dir", "");
  if (dir.empty()) dir = ament_index_cpp::get_package_share_directory("ml") + "/model/nav_correction";
  const auto names = declare_parameter<std::vector<std::string>>("dock_skills", {"dock_box", "drive_straight", "park_home", "drive_arc"});
  const auto limits = declare_parameter<std::vector<double>>("dock_limits_s", {25.0, 20.0, 25.0, 25.0});
  for (size_t i = 0; i < names.size(); ++i) {
    const std::string path = dir + "/" + names[i] + ".policy";
    if (!std::filesystem::exists(path)) continue;
    NavSkill k;
    std::string why;
    if (!k.policy.load(path, why) || k.policy.obs_dim() != NAV_OBS || k.policy.act_dim() != 2) {
      RCLCPP_ERROR(get_logger(), "%s: 정책을 못 씀 — %s", names[i].c_str(), why.c_str());
      continue;
    }
    k.limit_s = i < limits.size() ? limits[i] : 25.0;
    RCLCPP_INFO(get_logger(), "%s: 정책 %s", names[i].c_str(), path.c_str());
    skills_[names[i]] = std::move(k);
  }
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  ctx_sub_ = create_subscription<ml::msg::SkillContext>("/skill_context", rclcpp::QoS(10).reliable(),
    [this](ml::msg::SkillContext::ConstSharedPtr m) {
      if (m->run_id != ctx_.run_id || m->skill != ctx_.skill) { ctx_start_ = now(); act0_ = act1_ = 0.0; }
      ctx_ = *m;
    });
  diag_.setHardwareID("beebot");
  diag_.add("상태", [this](diagnostic_updater::DiagnosticStatusWrapper & s) {
    const bool fresh = (now() - last_sensor_).seconds() <= timeout_s_;
    if (!fresh) s.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "/odom 끊김 — Δ=0 (통과만)");
    else s.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, get_parameter("enabled").as_bool() ? "보정 중" : "Δ=0 (enabled=false)");
    s.add("enabled", get_parameter("enabled").as_bool());
    s.add("속도 상한 [m/s]", limit_v_);
    s.add("보정한 명령 수", corrected_);
  });
}

void RlNavCorrection::on_cmd(const TwistStamped & in) {
  double dv = 0.0, dw = 0.0;
  const bool fresh = (now() - last_sensor_).seconds() <= timeout_s_;
  if (get_parameter("enabled").as_bool() && fresh && dock_delta(dv, dw)) {
    dv = std::clamp(dv, -0.05, 0.05);
    dw = std::clamp(dw, -0.10, 0.10);
  }
  TwistStamped out = in;
  out.twist.linear.x = std::clamp(in.twist.linear.x + dv, -limit_v_, limit_v_);
  out.twist.angular.z = std::clamp(in.twist.angular.z + dw, -max_w_, max_w_);
  pub_->publish(out);
  last_v_ = out.twist.linear.x;
  last_w_ = out.twist.angular.z;
  ++corrected_;
}

bool RlNavCorrection::dock_delta(double & dv, double & dw) {
  const auto it = skills_.find(ctx_.skill);
  if (it == skills_.end() || !odom_) return false;
  NavObsIn in;
  try {  // 로봇 자세 (map) — coordinator approach와 같은 base_footprint
    const auto t = tf_buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero).transform;
    in.x = t.translation.x;
    in.y = t.translation.y;
    in.yaw = std::atan2(2 * (t.rotation.w * t.rotation.z + t.rotation.x * t.rotation.y),
                        1 - 2 * (t.rotation.y * t.rotation.y + t.rotation.z * t.rotation.z));
  } catch (const std::exception &) {
    return false;
  }
  in.gx = ctx_.goal_x;
  in.gy = ctx_.goal_y;
  in.gyaw = ctx_.goal_yaw;
  in.vx = odom_->twist.twist.linear.x;
  in.vy = odom_->twist.twist.linear.y;
  in.wz = odom_->twist.twist.angular.z;
  in.cmd_v = last_v_;
  in.cmd_w = last_w_;
  in.act0 = act0_;
  in.act1 = act1_;
  in.elapsed_s = (now() - ctx_start_).seconds();
  in.limit_s = it->second.limit_s;
  if (imu_) in.tilt = tilt_of(imu_->orientation.x, imu_->orientation.y, imu_->orientation.z, imu_->orientation.w);
  double obs[NAV_OBS], act[2];
  nav_obs(in, obs);
  it->second.policy.act_into(obs, act);
  act0_ = std::clamp(act[0], -1.0, 1.0);
  act1_ = std::clamp(act[1], -1.0, 1.0);
  dv = res_v_ * act0_;
  dw = res_w_ * act1_;
  return true;
}

}  // namespace ml
