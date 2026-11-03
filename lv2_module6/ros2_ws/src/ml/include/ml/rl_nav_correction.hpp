#pragma once

// rl_nav_correction (🛠 직접 작성, 담당: 송정혁) — 설계: documents/project/ros/06-base.md, 11-learn.md
//   입력: /cmd_vel_nav (Nav2, 20Hz), /plan (경로 오차), /odom·/imu (실제 움직임), /speed_limit (재클리핑 상한)
//   출력: /cmd_vel_corrected (geometry_msgs/TwistStamped) → velocity_smoother
// 규칙: Δ는 선속도 ±0.05m/s · 각속도 ±0.10rad/s 로 제한, 더한 뒤 min(프리셋 상한, /speed_limit)으로 다시 자른다.
//       센서 입력이 timeout_s(0.2초) 넘게 끊기거나 enabled:=false 면 Δ=0 (통과만).
// 도킹 모드 (배포 결정 2026-11-03): /skill_context의 skill에 정책 파일(model/nav_correction/<skill>.policy)이 있으면
//   그동안만 nav_obs(…) → MlpPolicy → Δv = 0.02·a0, Δw = 0.1·a1 (시뮬 drive_env res_v·res_w) — coordinator approach(시뮬과 같은
//   돌기-직진-돌기)가 낸 명령에 더함. 그 밖(Nav2 주행)은 Δ = 0 (Nav2용 보정 정책은 아직 없음).
#include <algorithm>
#include <cmath>

#include <map>
#include <memory>
#include <string>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_msgs/msg/speed_limit.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "ml/mlp_policy.hpp"
#include "ml/msg/skill_context.hpp"
#include "ml/nav_obs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace ml {

using geometry_msgs::msg::TwistStamped;

class RlNavCorrection : public rclcpp::Node {
public:
  RlNavCorrection();

private:
  void on_cmd(const TwistStamped & in);
  bool dock_delta(double & dv, double & dw);  // 도킹 모드 정책 Δ (꺼져 있으면 false)

  struct NavSkill { MlpPolicy policy; double limit_s = 25.0; };
  std::map<std::string, NavSkill> skills_;
  ml::msg::SkillContext ctx_;
  rclcpp::Time ctx_start_{0, 0, RCL_ROS_TIME};
  double last_v_ = 0.0, last_w_ = 0.0, act0_ = 0.0, act1_ = 0.0, res_v_ = 0.02, res_w_ = 0.1;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<ml::msg::SkillContext>::SharedPtr ctx_sub_;

  double max_v_, max_w_, timeout_s_, limit_v_;
  rclcpp::Time last_sensor_{0, 0, RCL_ROS_TIME};
  nav_msgs::msg::Path::ConstSharedPtr plan_;
  nav_msgs::msg::Odometry::ConstSharedPtr odom_;
  sensor_msgs::msg::Imu::ConstSharedPtr imu_;
  rclcpp::Subscription<TwistStamped>::SharedPtr cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav2_msgs::msg::SpeedLimit>::SharedPtr limit_sub_;
  rclcpp::Publisher<TwistStamped>::SharedPtr pub_;
  size_t corrected_ = 0;
  diagnostic_updater::Updater diag_{this};
};

}  // namespace ml
