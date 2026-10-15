// rl_nav_correction 실행 파일 — 노드는 ml::RlNavCorrection (include/ml/rl_nav_correction.hpp · src/rl_nav_correction.cpp)
#include "ml/rl_nav_correction.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ml::RlNavCorrection>());
  rclcpp::shutdown();
  return 0;
}
