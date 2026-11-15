#pragma once

// 🛠 hand_relative C++판 (11-16) — scripts/hand_relative.py와 노드 이름·파라미터·토픽·QoS·진단이 같음 · 계산은 hand_geom.hpp
//   입력: /hand_relative/target (perception/HandTarget, reliable + transient_local) · /perception/gripper_cam/detections (map 기준)
//         TF map → base_link, base_link → end_effector_link (영상 시각, 없으면 최신)
//   출력: /hand_relative (perception/HandRelative, base_link) — rl_arm_correction 정책 관측 · /diagnostics "hand_relative: 상태"
//   주기 rate_hz: slot·slot_puck은 TF만 · cell·puck은 마지막 검출(map)을 들고 발행 때마다 지금 손으로 다시 뺌 (stamp = 그 검출 영상 시각)
//   왜 C++: 파이썬 노드는 젯슨 CPU가 몰리면 ~1 s 멈춤 — 20 Hz 정책 입력이 끊김
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "builtin_interfaces/msg/time.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "perception/hand_geom.hpp"
#include "perception/msg/hand_relative.hpp"
#include "perception/msg/hand_target.hpp"
#include "perception/msg/tracked_object_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace perception {

class HandRelativeNode : public rclcpp::Node {
public:
  explicit HandRelativeNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using Pose = std::pair<hand_geom::V3, hand_geom::M3>;
  struct HeldDet {
    builtin_interfaces::msg::Time stamp;
    hand_geom::Held held;
    std::optional<hand_geom::V3> nb;  // 옆 퍽 − 목표
  };
  // (map에서 본 base_link, base_link에서 본 손끝) — stamp 시각(0.05 s 기다림), 안 되면 최신
  std::optional<std::pair<Pose, Pose>> frames(const builtin_interfaces::msg::Time * stamp);
  perception::msg::HandRelative base_msg(const builtin_interfaces::msg::Time * stamp, const std::string & kind) const;
  void publish(const perception::msg::HandRelative & m);
  void on_target(const perception::msg::HandTarget & msg);
  void on_timer();
  void publish_held();
  void on_dets(const perception::msg::TrackedObjectArray & msg);
  void on_diag(diagnostic_updater::DiagnosticStatusWrapper & s);

  std::string base_, ee_, map_;
  double max_age_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> tfl_;
  perception::msg::HandTarget target_;
  std::optional<HeldDet> held_;  // 마지막 cell·puck 검출
  unsigned long sent_ = 0;
  std::optional<rclcpp::Time> last_valid_;
  rclcpp::Subscription<perception::msg::HandTarget>::SharedPtr target_sub_;
  rclcpp::Subscription<perception::msg::TrackedObjectArray>::SharedPtr dets_sub_;
  rclcpp::Publisher<perception::msg::HandRelative>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<diagnostic_updater::Updater> diag_;
};

}  // namespace perception
