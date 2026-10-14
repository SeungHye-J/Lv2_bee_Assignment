// rl_nav_tuner 실행 파일 — 노드는 ml::RlNavTuner (include/ml/rl_nav_tuner.hpp · src/rl_nav_tuner.cpp)
#include "ml/rl_nav_tuner.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ml::RlNavTuner>());
  rclcpp::shutdown();
  return 0;
}
