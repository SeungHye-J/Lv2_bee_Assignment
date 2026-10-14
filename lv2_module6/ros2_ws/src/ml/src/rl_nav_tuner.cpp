// rl_nav_tuner 구현 — 노드 설명·입출력·멤버 선언은 include/ml/rl_nav_tuner.hpp, main은 rl_nav_tuner_main.cpp
#include "ml/rl_nav_tuner.hpp"

namespace ml {

namespace {

struct Preset { double speed_pct, max_vel_theta, acc_lim_x, decel_lim_x; };
static const std::map<char, Preset> PRESETS = {
  {'A', {100.0, 1.82, 2.5, -2.5}},
  {'B', {60.0, 1.0, 1.5, -1.5}},
  {'C', {40.0, 0.8, 1.0, -1.0}},
};

}  // namespace

RlNavTuner::RlNavTuner()
: rclcpp::Node("rl_nav_tuner") {
  enabled_ = declare_parameter<bool>("enabled", true);
  costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/local_costmap/costmap", 1, [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m) { costmap_ = m; });
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/odometry/filtered", 10, [this](nav_msgs::msg::Odometry::ConstSharedPtr m) { odom_ = m; });
  plan_sub_ = create_subscription<nav_msgs::msg::Path>(
    "/plan", 1, [this](nav_msgs::msg::Path::ConstSharedPtr m) { plan_ = m; });
  limit_pub_ = create_publisher<nav2_msgs::msg::SpeedLimit>("/speed_limit", rclcpp::QoS(1).reliable());
  params_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "/controller_server");
  timer_ = create_wall_timer(std::chrono::seconds(1), [this]() { tick(); });
  diag_.setHardwareID("beebot");
  diag_.add("상태", [this](diagnostic_updater::DiagnosticStatusWrapper & s) {
    if (!params_->service_is_ready()) s.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "/controller_server 대기");
    else s.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, std::string("프리셋 ") + (current_ ? current_ : '-'));
    s.add("enabled", get_parameter("enabled").as_bool());
    s.add("프리셋", std::string(1, current_ ? current_ : '-'));
  });
}

char RlNavTuner::choose() const {
  if (!get_parameter("enabled").as_bool()) return 'A';
  // TODO: costmap에서 통로 폭·최근접 장애물, plan 끝까지 남은 거리로 판단
  if (plan_ && !plan_->poses.empty() && odom_) {
    const auto & g = plan_->poses.back().pose.position;
    const auto & p = odom_->pose.pose.position;
    if (std::hypot(g.x - p.x, g.y - p.y) < 0.5) return 'C';
  }
  return 'A';
}

void RlNavTuner::tick() {
  const char next = choose();
  if (next == current_) return;
  if (!params_->service_is_ready()) {  // controller_server가 뜰 때까지 다음 주기에 다시
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "/controller_server 파라미터 서비스 대기 중");
    return;
  }
  current_ = next;
  const auto & p = PRESETS.at(next);
  nav2_msgs::msg::SpeedLimit limit;
  limit.header.stamp = now();
  limit.percentage = true;
  limit.speed_limit = p.speed_pct;
  limit_pub_->publish(limit);
  params_->set_parameters({
    rclcpp::Parameter("FollowPath.max_vel_theta", p.max_vel_theta),
    rclcpp::Parameter("FollowPath.acc_lim_x", p.acc_lim_x),
    rclcpp::Parameter("FollowPath.decel_lim_x", p.decel_lim_x)});
  RCLCPP_INFO(get_logger(), "preset %c (speed %.0f%%)", next, p.speed_pct);
}

}  // namespace ml
