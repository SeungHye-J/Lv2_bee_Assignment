// depth_gate 실행 파일 — 노드는 perception::DepthGate (include/perception/depth_gate.hpp)
#include "perception/depth_gate.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<perception::DepthGate>());
  rclcpp::shutdown();
  return 0;
}
