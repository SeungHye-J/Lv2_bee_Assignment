#pragma once

#include <array>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <string>

#include "ml/msg/arm_correction_debug.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace skills {

// One high-rate state/TF subscriber set per rclcpp context (in practice, per
// arm_skills/base_skills component container), shared by its ArmIO instances.
// Keep the 100 Hz source rate and the full TF history; only remove duplicate readers.
class ArmStateCache {
public:
  static std::shared_ptr<ArmStateCache> for_node(rclcpp::Node & node);

  bool position(const std::string & joint, double & value) const;
  bool joint(const std::string & joint, double & position, double & effort) const;
  double jam_mean(double window_s) const;
  bool delta_stable(double now, double fresh_s, double window_s, double max_spread, size_t min_samples) const;
  tf2_ros::Buffer & tf() const { return *tf_buffer_; }

private:
  explicit ArmStateCache(rclcpp::Node & node);
  void on_joint_state(const sensor_msgs::msg::JointState & msg);
  void on_debug(const ml::msg::ArmCorrectionDebug & msg);

  mutable std::mutex mutex_;
  std::map<std::string, double> pos_, effort_;
  std::deque<std::pair<double, double>> jam_hist_;
  std::deque<std::pair<double, std::array<double, 5>>> delta_hist_;
  rclcpp::Clock::SharedPtr clock_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  // Declared after callback-owned state so subscriptions are destroyed first.
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joints_sub_;
  rclcpp::Subscription<ml::msg::ArmCorrectionDebug>::SharedPtr debug_sub_;
};

}  // namespace skills
