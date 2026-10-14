#pragma once

// vlm_task_planner (🛠 직접 작성, 담당: 권형중) — 설계: documents/ros2-design/03-plan.md
//   입력: /user_command (명령), /tracked_objects (지금 보이는 물체), /task_status (FAILED면 재계획)
//   출력: /sub_goals (plan/SubGoalArray, reliable + transient_local) → coordinator
// VLM은 "무엇을, 어떤 순서로"만 정한다. 어떻게 움직일지·파지 후보 재시도는 coordinator 몫.
// 지금은 골격: 명령과 상관없이 퍽 이송 템플릿 [navigate(퍽 앞) → pick(퍽) → navigate(목표 구역) → place(목표 구역)].
// 재계획은 명령 하나당 max_replans(기본 1)회 — coordinator의 "같은 단계 2회 연속 실패면 정지"와 짝.
// TODO(권형중): VLM 호출(프롬프트 + /tracked_objects + 이미지) → JSON 파싱 → SubGoalArray
#include <string>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "perception/msg/tracked_object_array.hpp"
#include "plan/msg/sub_goal_array.hpp"
#include "plan/msg/task_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace plan {

using perception::msg::TrackedObjectArray;
using plan::msg::SubGoal;
using plan::msg::SubGoalArray;
using plan::msg::TaskStatus;

class VlmTaskPlanner : public rclcpp::Node {
public:
  VlmTaskPlanner();

private:
  void on_status(const TaskStatus & s);

  std::string find(const std::string & cls) const;

  void make_plan();

  static SubGoal goal(uint32_t step, const std::string & skill, const std::string & target,
    const std::string & relation, float offset);

  int max_replans_, replans_ = 0;
  bool pending_ = false;  // 계획할 명령이 남아 있음
  uint32_t plan_id_ = 0;
  TrackedObjectArray::ConstSharedPtr objects_;
  rclcpp::Publisher<SubGoalArray>::SharedPtr goals_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;
  rclcpp::Subscription<TrackedObjectArray>::SharedPtr objects_sub_;
  rclcpp::Subscription<TaskStatus>::SharedPtr status_sub_;
  diagnostic_updater::Updater diag_{this};
};

}  // namespace plan
