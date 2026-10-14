// fake_nav2 구현 — 설명·선언은 include/faker/fake_nav2.hpp, main은 fake_nav2_main.cpp
#include "faker/fake_nav2.hpp"

namespace faker {

FakeNav2::FakeNav2()
: rclcpp::Node("controller_server") {
  nav_time_s_ = declare_parameter<double>("nav_time_s", 2.0);
  fail_count_ = declare_parameter<int>("fail_count", 0);
  // DWB kinematic 파라미터 (rl_nav_tuner 프리셋이 바꾸는 값)
  declare_parameter<double>("FollowPath.max_vel_theta", 1.82);
  declare_parameter<double>("FollowPath.acc_lim_x", 2.5);
  declare_parameter<double>("FollowPath.decel_lim_x", -2.5);
  param_cb_ = add_post_set_parameters_callback([this](const std::vector<rclcpp::Parameter> & ps) {
    for (const auto & p : ps) RCLCPP_INFO(get_logger(), "set_parameters %s = %s", p.get_name().c_str(), p.value_to_string().c_str());
  });
  plan_pub_ = create_publisher<nav_msgs::msg::Path>("/plan", 1);
  cmd_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel_nav", 10);
  costmap_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/local_costmap/costmap", 1);
  limit_sub_ = create_subscription<nav2_msgs::msg::SpeedLimit>("/speed_limit", rclcpp::QoS(1).reliable(),
    [this](nav2_msgs::msg::SpeedLimit::ConstSharedPtr m) {
      RCLCPP_INFO(get_logger(), "/speed_limit %.1f%s", m->speed_limit, m->percentage ? "%" : " m/s");
    });
  vel_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", rclcpp::QoS(1).reliable());
  vel_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>("/cmd_vel_mux", rclcpp::QoS(1).reliable(),
    [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr m) { vel_pub_->publish(*m); });
  costmap_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() { publish_costmap(); });
  server_ = rclcpp_action::create_server<NavigateToPose>(this, "/navigate_to_pose",
    [](const rclcpp_action::GoalUUID &, std::shared_ptr<const NavigateToPose::Goal>) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [](std::shared_ptr<GoalHandle>) { return rclcpp_action::CancelResponse::ACCEPT; },
    [this](std::shared_ptr<GoalHandle> gh) { std::thread([this, gh]() { execute(gh); }).detach(); });
}

void FakeNav2::execute(const std::shared_ptr<GoalHandle> & gh) {
  const auto & target = gh->get_goal()->pose;
  RCLCPP_INFO(get_logger(), "navigate_to_pose (%.2f, %.2f)", target.pose.position.x, target.pose.position.y);
  nav_msgs::msg::Path path;
  path.header.stamp = now();
  path.header.frame_id = "map";
  geometry_msgs::msg::PoseStamped start;
  start.header = path.header;
  start.pose.orientation.w = 1.0;
  path.poses = {start, target};
  plan_pub_->publish(path);

  rclcpp::Rate rate(20);
  const auto end = now() + rclcpp::Duration::from_seconds(nav_time_s_);
  while (rclcpp::ok() && now() < end) {
    if (gh->is_canceling()) { gh->canceled(std::make_shared<NavigateToPose::Result>()); return; }
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = now();
    cmd.header.frame_id = "base_footprint";
    cmd.twist.linear.x = 0.12;
    cmd.twist.angular.z = 0.30;
    cmd_pub_->publish(cmd);
    rate.sleep();
  }
  geometry_msgs::msg::TwistStamped stop;
  stop.header.stamp = now();
  stop.header.frame_id = "base_footprint";
  cmd_pub_->publish(stop);
  auto result = std::make_shared<NavigateToPose::Result>();
  if (fail_count_ > 0) {
    --fail_count_;
    result->error_code = NavigateToPose::Result::UNKNOWN;
    gh->abort(result);
    RCLCPP_WARN(get_logger(), "navigate_to_pose ABORTED (fail_count 남은 %d)", fail_count_);
  } else {
    gh->succeed(result);
  }
}

void FakeNav2::publish_costmap() {
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.stamp = now();
  grid.header.frame_id = "odom";
  grid.info.resolution = 0.05f;
  grid.info.width = 60;
  grid.info.height = 60;
  grid.info.origin.position.x = -1.5;
  grid.info.origin.position.y = -1.5;
  grid.info.origin.orientation.w = 1.0;
  grid.data.assign(grid.info.width * grid.info.height, 0);
  costmap_pub_->publish(grid);
}

}  // namespace faker
