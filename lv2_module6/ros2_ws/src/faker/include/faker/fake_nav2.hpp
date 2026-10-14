#pragma once

// 더미 Nav2 (bt_navigator + planner_server + controller_server + local_costmap).
// 노드 이름을 controller_server로 띄워 rl_nav_tuner의 📞 /controller_server/set_parameters를 그대로 받는다.
//   🎬 /navigate_to_pose 서버: nav_time_s 뒤 성공 (fail_count 파라미터만큼은 ABORTED — 이동 실패 시나리오)
//   ✉️ /plan (목표까지 직선), /cmd_vel_nav (이동 중 20Hz, TwistStamped), /local_costmap/costmap (빈 지도 1Hz)
//   ✉️ /speed_limit 구독 — 받은 값을 로그로
//   ✉️ /cmd_vel_mux(twist_mux 출력) → /cmd_vel 그대로 중계 — velocity_smoother + collision_monitor 자리 (바퀴가 실제로 돈다)
#include <chrono>
#include <memory>
#include <thread>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/msg/speed_limit.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace faker {

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandle = rclcpp_action::ServerGoalHandle<NavigateToPose>;

class FakeNav2 : public rclcpp::Node {
public:
  FakeNav2();

private:
  void execute(const std::shared_ptr<GoalHandle> & gh);

  void publish_costmap();

  double nav_time_s_;
  int fail_count_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr param_cb_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_, vel_pub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
  rclcpp::Subscription<nav2_msgs::msg::SpeedLimit>::SharedPtr limit_sub_;
  rclcpp::TimerBase::SharedPtr costmap_timer_;
  rclcpp_action::Server<NavigateToPose>::SharedPtr server_;
};

}  // namespace faker
