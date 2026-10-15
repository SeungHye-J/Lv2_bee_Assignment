// grasp_scorer 실행 파일 — 노드는 ml::GraspScorer (include/ml/grasp_scorer.hpp · src/grasp_scorer.cpp)
#include "ml/grasp_scorer.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ml::GraspScorer>());
  rclcpp::shutdown();
  return 0;
}
