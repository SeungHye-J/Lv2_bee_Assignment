// hand_relative 실행 파일 — 노드는 perception::HandRelativeNode (include/perception/hand_relative.hpp)
#include "perception/hand_relative.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<perception::HandRelativeNode>());
  rclcpp::shutdown();
  return 0;
}
