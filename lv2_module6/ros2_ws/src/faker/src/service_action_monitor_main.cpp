// service_action_monitor 실행 파일 — main만 (구현은 src/service_action_monitor.cpp)
#include <memory>

#include "faker/service_action_monitor.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<faker::ServiceActionMonitor>());
  rclcpp::shutdown();
  return 0;
}
