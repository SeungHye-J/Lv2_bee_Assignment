// vlm_task_planner 실행 파일 — 노드는 plan::VlmTaskPlanner (include/plan/vlm_task_planner.hpp · src/vlm_task_planner.cpp)
#include "plan/vlm_task_planner.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<plan::VlmTaskPlanner>());
  rclcpp::shutdown();
  return 0;
}
