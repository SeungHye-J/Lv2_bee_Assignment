#pragma once

// 더미 move_group (+ MTC 실행 capability).
//   🎬 /execute_task_solution 서버: exec_time_s 뒤 성공. fail_count 파라미터만큼은 실패
//      (error_code=PLANNING_FAILED — "그리퍼 빈손" 파지 실패 → coordinator 재시도 시나리오)
//   📞 /clear_octomap (std_srvs/Empty), /apply_planning_scene (moveit_msgs/ApplyPlanningScene) — 항상 성공
#include <chrono>
#include <memory>
#include <thread>

#include "moveit_msgs/srv/apply_planning_scene.hpp"
#include "moveit_task_constructor_msgs/action/execute_task_solution.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/empty.hpp"

namespace faker {

using ExecuteTaskSolution = moveit_task_constructor_msgs::action::ExecuteTaskSolution;
using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteTaskSolution>;
using moveit_msgs::msg::MoveItErrorCodes;
using moveit_msgs::srv::ApplyPlanningScene;

class FakeMoveit : public rclcpp::Node {
public:
  FakeMoveit();

private:
  void execute(const std::shared_ptr<GoalHandle> & gh);

  double exec_time_s_;
  int fail_count_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr clear_;
  rclcpp::Service<ApplyPlanningScene>::SharedPtr scene_;
  rclcpp_action::Server<ExecuteTaskSolution>::SharedPtr server_;
};

}  // namespace faker
