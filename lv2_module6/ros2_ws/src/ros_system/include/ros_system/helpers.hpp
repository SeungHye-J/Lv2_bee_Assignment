#pragma once

// ros_system 보조 노드 C++판 (11-16) — 파이썬 scripts/nvblox_map.py · rewind.py와 노드 이름·파라미터·토픽·QoS·서비스·로그가 같음
//   왜: 파이썬 노드는 젯슨 CPU가 몰리면 ~1 s 멈추고, 프로세스마다 ~120 MB PSS (py_helpers로 묶어도 한 덩어리 남음)
//   cpp_helpers <노드 이름…> 이 한 프로세스에 띄움 (src/cpp_helpers.cpp · py_helpers와 같은 방식)
#include <memory>

#include "rclcpp/rclcpp.hpp"

namespace ros_system {

// nvblox 2D 점유 격자 → /map (latched) + 저장 지도 불러오기 (설명: scripts/nvblox_map.py 머리말)
std::shared_ptr<rclcpp::Node> make_nvblox_map(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
// 실패한 주행의 odom 궤적을 0.4 m 후진 (설명: scripts/rewind.py 머리말)
std::shared_ptr<rclcpp::Node> make_rewind(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

}  // namespace ros_system
