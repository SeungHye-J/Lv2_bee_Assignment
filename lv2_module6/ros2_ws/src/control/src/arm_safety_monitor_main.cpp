// arm_safety_monitor 실행 파일 — 노드는 control::ArmSafetyMonitor (include/control/arm_safety_monitor.hpp · src/arm_safety_monitor.cpp)
#include "control/arm_safety_monitor.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<control::ArmSafetyMonitor>());
  rclcpp::shutdown();
  return 0;
}
