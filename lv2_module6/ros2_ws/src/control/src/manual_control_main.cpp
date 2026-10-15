// manual_control 실행 파일 — 노드는 control::ManualControl (include/control/manual_control.hpp · src/manual_control.cpp)
#include "control/manual_control.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<control::ManualControl>());
  rclcpp::shutdown();
  return 0;
}
