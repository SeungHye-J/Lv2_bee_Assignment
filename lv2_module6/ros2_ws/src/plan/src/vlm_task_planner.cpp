// vlm_task_planner 구현 — 노드 설명·입출력·멤버 선언은 include/plan/vlm_task_planner.hpp, main은 vlm_task_planner_main.cpp
#include "plan/vlm_task_planner.hpp"

namespace plan {

VlmTaskPlanner::VlmTaskPlanner()
: rclcpp::Node("vlm_task_planner") {
  max_replans_ = declare_parameter<int>("max_replans", 1);
  goals_pub_ = create_publisher<SubGoalArray>("/sub_goals", rclcpp::QoS(10).reliable().transient_local());
  cmd_sub_ = create_subscription<std_msgs::msg::String>("/user_command", rclcpp::QoS(10).reliable(),
    [this](std_msgs::msg::String::ConstSharedPtr m) {
      RCLCPP_INFO(get_logger(), "명령: \"%s\"", m->data.c_str());
      replans_ = 0;
      pending_ = true;
      make_plan();
    });
  objects_sub_ = create_subscription<TrackedObjectArray>("/tracked_objects", rclcpp::QoS(5).reliable(),
    [this](TrackedObjectArray::ConstSharedPtr m) {
      objects_ = m;
      if (pending_) make_plan();  // 물체 목록보다 명령이 먼저 왔으면 지금 계획
    });
  status_sub_ = create_subscription<TaskStatus>("/task_status", rclcpp::QoS(10).reliable().transient_local(),
    [this](TaskStatus::ConstSharedPtr m) { on_status(*m); });
  diag_.setHardwareID("beebot");
  diag_.add("상태", [this](diagnostic_updater::DiagnosticStatusWrapper & s) {
    if (pending_) s.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "계획 보류 — 대상 물체 대기");
    else s.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, plan_id_ ? "계획 " + std::to_string(plan_id_) : "명령 대기");
    s.add("plan_id", plan_id_);
    s.add("재계획", std::to_string(replans_) + "/" + std::to_string(max_replans_));
    s.add("보이는 물체", objects_ ? objects_->objects.size() : 0);
  });
}

void VlmTaskPlanner::on_status(const TaskStatus & s) {
  if (s.state != TaskStatus::FAILED || s.plan_id != plan_id_) return;
  if (s.reason.rfind("manual", 0) == 0) return;  // 수동 개입으로 멈춘 것 — 사람이 다시 명령할 때까지
  if (replans_ >= max_replans_) return;  // coordinator가 알리고 정지한다
  ++replans_;
  RCLCPP_WARN(get_logger(), "step %u 실패(%s) → 재계획 %d/%d", s.step, s.reason.c_str(), replans_, max_replans_);
  pending_ = true;
  make_plan();  // TODO: 실패 이유를 프롬프트에 넣어 다른 접근 방향으로
}

std::string VlmTaskPlanner::find(const std::string & cls) const {
  if (objects_) for (const auto & o : objects_->objects) if (o.class_name == cls) return std::to_string(o.id);
  return "";
}

void VlmTaskPlanner::make_plan() {
  const auto puck = find("puck"), goal_zone = find("goal_zone");
  if (puck.empty() || goal_zone.empty()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "/tracked_objects에 퍽·목표 구역이 없음 — 계획 보류");
    return;
  }
  pending_ = false;
  SubGoalArray plan;
  plan.header.stamp = now();
  plan.plan_id = ++plan_id_;
  plan.goals.push_back(goal(1, "navigate", puck, "front", 0.20f));
  plan.goals.push_back(goal(2, "pick", puck, "", 0.0f));
  plan.goals.push_back(goal(3, "navigate", goal_zone, "front", 0.20f));
  plan.goals.push_back(goal(4, "place", goal_zone, "on", 0.0f));
  goals_pub_->publish(plan);
  RCLCPP_INFO(get_logger(), "/sub_goals plan=%u (%zu steps)", plan.plan_id, plan.goals.size());
}

SubGoal VlmTaskPlanner::goal(uint32_t step, const std::string & skill, const std::string & target,
  const std::string & relation, float offset) {
  SubGoal g;
  g.step = step;
  g.skill = skill;
  g.target = target;
  g.relation = relation;
  g.offset_m = offset;
  return g;
}

}  // namespace plan
