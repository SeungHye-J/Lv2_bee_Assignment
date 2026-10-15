// fake_camera 실행 파일 — 노드는 faker::FakeCamera (include/faker/fake_camera.hpp · src/fake_camera.cpp)
#include "faker/fake_camera.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<faker::FakeCamera>());
  rclcpp::shutdown();
  return 0;
}
