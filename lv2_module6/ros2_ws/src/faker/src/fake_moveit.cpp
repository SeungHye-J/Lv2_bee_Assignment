// fake_moveit 구현 — 설명·선언은 include/faker/fake_moveit.hpp, main은 fake_moveit_main.cpp
#include "faker/fake_moveit.hpp"

namespace faker {

FakeMoveit::FakeMoveit()
: rclcpp::Node("move_group") {
  exec_time_s_ = declare_parameter<double>("exec_time_s", 1.0);
  fail_count_ = declare_parameter<int>("fail_count", 0);
  clear_ = create_service<std_srvs::srv::Empty>("/clear_octomap",
    [this](const std_srvs::srv::Empty::Request::SharedPtr, std_srvs::srv::Empty::Response::SharedPtr) {
      RCLCPP_INFO(get_logger(), "/clear_octomap");
    });
  scene_ = create_service<ApplyPlanningScene>("/apply_planning_scene",
    [this](const ApplyPlanningScene::Request::SharedPtr, ApplyPlanningScene::Response::SharedPtr res) {
      RCLCPP_INFO(get_logger(), "/apply_planning_scene");
      res->success = true;
    });
  server_ = rclcpp_action::create_server<ExecuteTaskSolution>(this, "/execute_task_solution",
    [](const rclcpp_action::GoalUUID &, std::shared_ptr<const ExecuteTaskSolution::Goal>) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [](std::shared_ptr<GoalHandle>) { return rclcpp_action::CancelResponse::ACCEPT; },
    [this](std::shared_ptr<GoalHandle> gh) { std::thread([this, gh]() { execute(gh); }).detach(); });
}

void FakeMoveit::execute(const std::shared_ptr<GoalHandle> & gh) {
  const auto & task = gh->get_goal()->solution.task_id;
  RCLCPP_INFO(get_logger(), "execute_task_solution %s", task.c_str());
  std::this_thread::sleep_for(std::chrono::duration<double>(exec_time_s_));
  auto result = std::make_shared<ExecuteTaskSolution::Result>();
  if (fail_count_ > 0) {
    --fail_count_;
    result->error_code.val = MoveItErrorCodes::PLANNING_FAILED;
    RCLCPP_WARN(get_logger(), "%s 실패 (fail_count 남은 %d)", task.c_str(), fail_count_);
  } else {
    result->error_code.val = MoveItErrorCodes::SUCCESS;
  }
  gh->succeed(result);  // MTC는 실패도 result의 error_code로 알린다
}

}  // namespace faker
