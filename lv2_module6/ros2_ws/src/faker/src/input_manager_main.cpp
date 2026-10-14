// input_manager 실행 파일 — 노드는 faker::InputManager (include/faker/input_manager.hpp · src/input_manager.cpp)
#include "faker/input_manager.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<faker::InputManager>());
  rclcpp::shutdown();
  return 0;
}
