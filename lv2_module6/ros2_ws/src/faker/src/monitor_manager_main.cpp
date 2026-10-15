// monitor_manager 실행 파일 — 노드는 faker::MonitorManager (include/faker/monitor_manager.hpp · src/monitor_manager.cpp)
#include "faker/monitor_manager.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<faker::MonitorManager>());
  rclcpp::shutdown();
  return 0;
}
