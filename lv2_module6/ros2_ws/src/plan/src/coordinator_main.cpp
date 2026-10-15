// coordinator 실행 파일 — 노드는 plan::Coordinator (include/plan/coordinator.hpp · src/coordinator.cpp)
#include "plan/coordinator.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<plan::Coordinator>());
  rclcpp::shutdown();
  return 0;
}
