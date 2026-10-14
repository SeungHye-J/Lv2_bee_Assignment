#pragma once

// perception (🛠 직접 작성, 담당: 전승혜) — 설계: documents/ros2-design/02-sense.md
//   입력: pole_cam·gripper_cam 컬러·camera_info·정렬 depth (realsense2_camera 드라이버, best_effort)
//   출력: /tracked_objects (perception/TrackedObjectArray, 10Hz · reliable) → vlm_task_planner, coordinator, grasp_scorer
// 지금은 골격: 영상이 들어오면 고정 물체 목록(퍽·목표 구역·장애물)을 발행한다.
// TODO(전승혜): 검출 모델 추론 → depth로 3D 위치 → TF(header.stamp 시각)로 map 기준 변환 → ID 추적
#include <chrono>
#include <string>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "perception/msg/tracked_object_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace perception {

using perception::msg::TrackedObject;
using perception::msg::TrackedObjectArray;
using sensor_msgs::msg::CameraInfo;
using sensor_msgs::msg::Image;

class PerceptionNode : public rclcpp::Node {
public:
  PerceptionNode();

private:
  void on_color(const std::string & cam, const Image::ConstSharedPtr & msg);

  void publish();

  TrackedObject make(uint32_t id, const std::string & cls, float conf,
    double x, double y, double z, double sx, double sy, double sz);

  std::vector<rclcpp::Subscription<Image>::SharedPtr> image_subs_;
  std::vector<rclcpp::Subscription<CameraInfo>::SharedPtr> info_subs_;
  rclcpp::Publisher<TrackedObjectArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_pole_{0, 0, RCL_ROS_TIME};
  size_t published_ = 0;
  diagnostic_updater::Updater diag_{this};
};

}  // namespace perception
