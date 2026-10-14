// fake_moveit 실행 파일 — 노드는 faker::FakeMoveit (include/faker/fake_moveit.hpp · src/fake_moveit.cpp)
#include "faker/fake_moveit.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec;
  auto node = std::make_shared<faker::FakeMoveit>();  // executor는 weak_ptr만 들고 있으므로 여기서 붙잡아 둔다
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
