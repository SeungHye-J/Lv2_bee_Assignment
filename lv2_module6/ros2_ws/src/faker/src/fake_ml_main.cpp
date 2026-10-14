// fake_ml 실행 파일 — ML 대역 노드 3개(grasp_scorer · rl_nav_tuner · rl_nav_correction)를 한 프로세스에 (include/faker/fake_ml.hpp)
#include "faker/fake_ml.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  auto scorer = std::make_shared<faker::FakeGraspScorer>();  // executor는 weak_ptr만 들고 있으므로 여기서 붙잡아 둔다
  auto tuner = std::make_shared<faker::FakeNavTuner>();
  auto correction = std::make_shared<faker::FakeNavCorrection>();
  exec.add_node(scorer);
  exec.add_node(tuner);
  exec.add_node(correction);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
