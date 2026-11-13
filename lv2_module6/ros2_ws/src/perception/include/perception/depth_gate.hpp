#pragma once

// depth_gate — nvblox에 줄 depth에서 카메라 가까이(min_range 안) 픽셀을 0(측정 없음)으로: pole_front_cam에 찍힌 팔·폴대
//   ros_system/scripts/scan_cam_gate.py(gate_by_pose false · depth_min_range)의 C++판. 하는 일은 같음.
//   왜: 파이썬판은 15 Hz depth(640×360×2 B)마다 rclpy 변환 + bytes 복사 여러 번 — C++은 받은 메시지를 그 자리에서 고쳐 그대로 넘김.
//       젯슨 bag 재생 (11/14, 15 Hz): 파이썬 CPU 6.1 % → C++ 2.0 % (출력 같음)
//   입력: depth_in (정렬 depth, 16UC1 mm · 32FC1 m — 그 밖 인코딩은 그대로 넘김) · 출력: depth_out (같은 시각·프레임·크기)
//   파라미터 이름은 파이썬판과 같음 (ros_system.launch.py gate_params 그대로) — gate_by_pose·cloud_in은 C++판에 없음(쓰는 곳 없음)
#include <cstdint>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace perception {

// depth 영상 그 자리에서: 0 < 값 < min_range인 픽셀을 0으로 → 바꾼 픽셀 수 (인코딩이 16UC1·32FC1이 아니면 -1)
long zero_near_depth(sensor_msgs::msg::Image & img, double min_range_m);

class DepthGate : public rclcpp::Node {
public:
  explicit DepthGate(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void on_depth(sensor_msgs::msg::Image::UniquePtr msg);
  double min_range_;
  uint64_t passed_{0}, zeroed_px_{0};
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr log_timer_;
};

}  // namespace perception
