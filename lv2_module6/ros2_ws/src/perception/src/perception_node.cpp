// perception_node 구현 — 노드 설명·입출력·멤버 선언은 include/perception/perception_node.hpp, main은 perception_node_main.cpp
#include "perception/perception_node.hpp"

namespace perception {

PerceptionNode::PerceptionNode()
: rclcpp::Node("perception") {
  const auto sensor_qos = rclcpp::SensorDataQoS();
  for (const std::string cam : {"pole_cam", "gripper_cam"}) {
    image_subs_.push_back(create_subscription<Image>(
      "/" + cam + "/color/image_raw", sensor_qos,
      [this, cam](Image::ConstSharedPtr msg) { on_color(cam, msg); }));
    image_subs_.push_back(create_subscription<Image>(
      "/" + cam + "/aligned_depth_to_color/image_raw", sensor_qos,
      [](Image::ConstSharedPtr) { /* TODO: 픽셀별 거리 */ }));
    info_subs_.push_back(create_subscription<CameraInfo>(
      "/" + cam + "/color/camera_info", sensor_qos,
      [](CameraInfo::ConstSharedPtr) { /* TODO: 내부 파라미터 → 3D 투영 */ }));
  }
  const double rate_hz = declare_parameter<double>("rate_hz", 10.0);
  pub_ = create_publisher<TrackedObjectArray>("/tracked_objects", rclcpp::QoS(5).reliable());
  timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate_hz), [this]() { publish(); });
  // 상태: /diagnostics (bringup의 diagnostic_aggregator가 /diagnostics_agg로 모음)
  diag_.setHardwareID("beebot");
  diag_.add("상태", [this](diagnostic_updater::DiagnosticStatusWrapper & s) {
    const double age = last_pole_.nanoseconds() ? (now() - last_pole_).seconds() : -1;
    if (age < 0) s.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "pole_cam 영상 대기");
    else if (age > 1.0) s.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "pole_cam 영상 끊김");
    else s.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "인지 중");
    s.add("pole_cam 영상 [s 전]", age);
    s.add("발행한 /tracked_objects", published_);
  });
}

void PerceptionNode::on_color(const std::string & cam, const Image::ConstSharedPtr & msg) {
  if (cam == "pole_cam") last_pole_ = msg->header.stamp;
}

void PerceptionNode::publish() {
  if (last_pole_.nanoseconds() == 0) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "pole_cam 영상 대기 중");
    return;
  }
  TrackedObjectArray out;
  out.header.stamp = now();
  out.header.frame_id = "map";
  // 더미 물체 — 미로 코스(waffle-lyrical-slam twin_closed_maze) 안 좌표, map 원점 = 로봇 시작 칸 (미로 -1.835, 2.757)
  //   미로는 y=0 벽으로 위·아래가 막혀 있어 시작 칸이 있는 위쪽 반에만 둔다
  //   퍽: 위쪽 반 오른쪽 아래 칸 (미로 1.835, 0.394) · 목표 구역: 왼쪽 칸 (미로 -1.835, 1.182) · 장애물: 그 사이 통로 (미로 -0.367, 1.182)
  //   class_name: puck(옮길 물체) · goal_zone(내려놓을 곳, 바닥 표시) · obstacle(피할 것)
  out.objects.push_back(make(3, "puck", 0.93f, 3.670, -2.363, 0.0, 0.06, 0.06, 0.04));
  out.objects.push_back(make(5, "goal_zone", 0.95f, 0.000, -1.575, 0.0, 0.30, 0.30, 0.01));
  out.objects.push_back(make(7, "obstacle", 0.90f, 1.468, -1.575, 0.0, 0.20, 0.20, 0.30));
  pub_->publish(out);
  ++published_;
}

TrackedObject PerceptionNode::make(uint32_t id, const std::string & cls, float conf,
  double x, double y, double z, double sx, double sy, double sz) {
  TrackedObject o;
  o.id = id;
  o.class_name = cls;
  o.confidence = conf;
  o.pose.pose.position.x = x;
  o.pose.pose.position.y = y;
  o.pose.pose.position.z = z;
  o.pose.pose.orientation.w = 1.0;
  o.size.x = sx;
  o.size.y = sy;
  o.size.z = sz;
  o.last_seen = last_pole_;
  return o;
}

}  // namespace perception
