// user_input 실행 파일 — 노드는 plan::UserInput (include/plan/user_input.hpp · src/user_input.cpp)
#include "plan/user_input.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<plan::UserInput>());
  rclcpp::shutdown();
  return 0;
}
