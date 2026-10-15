// perception_node 실행 파일 — 노드는 perception::PerceptionNode (include/perception/perception_node.hpp · src/perception_node.cpp)
#include "perception/perception_node.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<perception::PerceptionNode>());
  rclcpp::shutdown();
  return 0;
}
