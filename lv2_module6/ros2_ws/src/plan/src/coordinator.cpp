// coordinator 구현 — 노드 설명·입출력·멤버 선언은 include/plan/coordinator.hpp, main은 coordinator_main.cpp
#include "plan/coordinator.hpp"

namespace plan {

Coordinator::Coordinator()
: rclcpp::Node("coordinator") {
  max_grasp_tries_ = declare_parameter<int>("max_grasp_tries", 2);
  max_step_failures_ = declare_parameter<int>("max_step_failures", 2);
  server_wait_s_ = declare_parameter<double>("server_wait_s", 30.0);
  const auto latched = rclcpp::QoS(10).reliable().transient_local();
  status_pub_ = create_publisher<TaskStatus>("/task_status", latched);
  say_pub_ = create_publisher<std_msgs::msg::String>("/robot_say", rclcpp::QoS(10).reliable());
  goals_sub_ = create_subscription<SubGoalArray>("/sub_goals", latched,
    [this](SubGoalArray::ConstSharedPtr m) { on_plan(*m); });
  objects_sub_ = create_subscription<TrackedObjectArray>("/tracked_objects", rclcpp::QoS(5).reliable(),
    [this](TrackedObjectArray::ConstSharedPtr m) {
      const bool first = !objects_;
      objects_ = m;
      if (first && waiting_plan_) {  // 물체 목록보다 계획이 먼저 왔으면 지금 검증
        auto plan = *waiting_plan_;
        waiting_plan_.reset();
        on_plan(plan);
      }
    });
  safety_sub_ = create_subscription<ArmSafetyEvent>("/arm_safety_event", rclcpp::QoS(10).reliable(),
    [this](ArmSafetyEvent::ConstSharedPtr m) {
      RCLCPP_WARN(get_logger(), "팔 접촉 %s %.2fA", m->joint_name.c_str(), m->current_a);
      arm_contact_ = true;  // 진행 중인 MTC 결과가 실패로 오면 reason을 arm_contact로
    });
  mode_sub_ = create_subscription<std_msgs::msg::String>("/operation_mode", rclcpp::QoS(1).reliable().transient_local(),
    [this](std_msgs::msg::String::ConstSharedPtr m) { on_mode(m->data); });
  nav_ = rclcpp_action::create_client<NavigateToPose>(this, "/navigate_to_pose");
  mtc_ = rclcpp_action::create_client<ExecuteTaskSolution>(this, "/execute_task_solution");
  score_ = create_client<ScoreGrasps>("/score_grasps");
  clear_octomap_ = create_client<std_srvs::srv::Empty>("/clear_octomap");
  apply_scene_ = create_client<ApplyPlanningScene>("/apply_planning_scene");
  diag_.setHardwareID("beebot");
  diag_.add("상태", [this](diagnostic_updater::DiagnosticStatusWrapper & s) {
    using diagnostic_msgs::msg::DiagnosticStatus;
    if (mode_ == "MANUAL") s.summary(DiagnosticStatus::WARN, "수동 모드 — 자동 실행 멈춤");
    else if (waited_s_ > 0) s.summary(DiagnosticStatus::WARN, "Nav2·MoveIt 서버 대기");
    else if (running_) s.summary(DiagnosticStatus::OK, "실행 중: " + step_text());
    else s.summary(DiagnosticStatus::OK, "대기");
    s.add("모드", mode_);
    s.add("plan_id", plan_.plan_id);
    s.add("단계", step_text());
    s.add("실행 중", running_);
    s.add("연속 실패", failures_.size());
  });
}

void Coordinator::on_mode(const std::string & mode) {
  if (mode == mode_) return;
  mode_ = mode;
  RCLCPP_WARN(get_logger(), "운영 모드: %s", mode.c_str());
  if (mode == "MANUAL" && running_) {
    nav_->async_cancel_all_goals();
    mtc_->async_cancel_all_goals();
    running_ = false;
    status(idx_ < plan_.goals.size() ? plan_.goals[idx_].step : 0, TaskStatus::FAILED, "manual_override");
  }
}

std::string Coordinator::step_text() const {
  if (idx_ >= plan_.goals.size()) return "-";
  const auto & g = plan_.goals[idx_];
  return std::to_string(g.step) + " " + g.skill + "(" + g.target + ")";
}

void Coordinator::on_plan(const SubGoalArray & plan) {
  if (plan.plan_id == plan_.plan_id && running_) return;  // transient_local 재수신
  if (mode_ == "MANUAL") {
    plan_ = plan;
    RCLCPP_WARN(get_logger(), "계획 %u 거부: 수동 모드", plan.plan_id);
    status(0, TaskStatus::FAILED, "manual_mode");
    return;
  }
  if (!objects_) {  // 대상 id를 검증할 수 없음 — 첫 /tracked_objects까지 미룬다
    RCLCPP_INFO(get_logger(), "계획 %u: /tracked_objects 대기 후 검증", plan.plan_id);
    waiting_plan_ = std::make_shared<SubGoalArray>(plan);
    return;
  }
  std::string why;
  if (!validate(plan, why)) {
    plan_ = plan;
    RCLCPP_ERROR(get_logger(), "계획 %u 거부: %s", plan.plan_id, why.c_str());
    status(0, TaskStatus::FAILED, "invalid_plan:" + why);
    return;
  }
  RCLCPP_INFO(get_logger(), "계획 %u 수락 (%zu steps)", plan.plan_id, plan.goals.size());
  plan_ = plan;
  idx_ = 0;
  running_ = true;
  run_step();
}

bool Coordinator::validate(const SubGoalArray & plan, std::string & why) const {
  static const std::set<std::string> allowed = {"navigate", "pick", "place", "say"};
  if (plan.goals.empty()) { why = "empty"; return false; }
  for (const auto & g : plan.goals) {
    if (!allowed.count(g.skill)) { why = "skill " + g.skill; return false; }
    if (is_id(g.target) && !find(std::stoul(g.target))) { why = "target " + g.target + " not tracked"; return false; }
  }
  return true;
}

bool Coordinator::is_id(const std::string & s) {
  return !s.empty() && s.find_first_not_of("0123456789") == std::string::npos;
}

const perception::msg::TrackedObject * Coordinator::find(uint32_t id) const {
  if (objects_) for (const auto & o : objects_->objects) if (o.id == id) return &o;
  return nullptr;
}

void Coordinator::run_step() {
  if (!running_ || mode_ == "MANUAL") return;
  if (idx_ >= plan_.goals.size()) {
    running_ = false;
    status(plan_.goals.back().step, TaskStatus::SUCCEEDED, "");
    say("퍽을 옮겼어요");
    return;
  }
  const auto & g = plan_.goals[idx_];
  status(g.step, TaskStatus::RUNNING, "");
  RCLCPP_INFO(get_logger(), "step %u: %s %s", g.step, g.skill.c_str(), g.target.c_str());
  if (g.skill == "navigate") navigate(g);
  else if (g.skill == "pick") pick(g);
  else if (g.skill == "place") {
    if (ready_or_retry(mtc_->action_server_is_ready(), "mtc_unavailable"))
      execute_mtc("place:" + g.target, [this](bool ok) { ok ? next() : fail("place_failed"); });
  } else { say(g.target); next(); }
}

bool Coordinator::ready_or_retry(bool ready, const std::string & reason) {
  if (ready) { waited_s_ = 0; return true; }
  if (waited_s_ >= server_wait_s_) { waited_s_ = 0; fail(reason); return false; }
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "%s — 서버 대기 중", reason.c_str());
  waited_s_ += 1.0;
  retry_ = create_wall_timer(std::chrono::seconds(1), [this]() { retry_->cancel(); run_step(); });
  return false;
}

void Coordinator::next() {
  if (!running_) return;  // 수동 전환 등으로 이미 멈춘 뒤 도착한 결과
  failures_.erase(key(plan_.goals[idx_]));
  ++idx_;
  run_step();
}

void Coordinator::fail(const std::string & reason) {
  if (!running_) return;  // 이미 멈춤 (수동 전환으로 취소된 goal의 결과 등)
  const auto & g = plan_.goals[idx_];
  const int n = ++failures_[key(g)];
  running_ = false;
  RCLCPP_WARN(get_logger(), "step %u 실패 (%s) %d/%d", g.step, reason.c_str(), n, max_step_failures_);
  status(g.step, TaskStatus::FAILED, reason);
  if (n >= max_step_failures_) {
    failures_.clear();
    say(g.skill == "navigate" ? "목표까지 갈 수 없어요" : "퍽을 잡을 수 없어요");
  }
}

std::string Coordinator::key(const SubGoal & g) { return g.skill + ":" + g.target; }

void Coordinator::navigate(const SubGoal & g) {
  if (!ready_or_retry(nav_->action_server_is_ready(), "nav_unavailable")) return;
  NavigateToPose::Goal goal;
  goal.pose.header.frame_id = "map";
  goal.pose.header.stamp = now();
  goal.pose.pose.orientation.w = 1.0;
  if (is_id(g.target)) {
    // TODO: 대상 앞 offset + 로봇 반경만큼 떨어진 접근 자세 (통로 방향 고려 — 미로 벽에 붙지 않게)
    const auto * o = find(std::stoul(g.target));
    goal.pose.pose.position.x = o->pose.pose.position.x - o->size.x / 2 - g.offset_m - 0.1;
    goal.pose.pose.position.y = o->pose.pose.position.y;
  }
  rclcpp_action::Client<NavigateToPose>::SendGoalOptions opt;
  opt.result_callback = [this](const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult & r) {
    r.code == rclcpp_action::ResultCode::SUCCEEDED ? next() : fail("nav_failed");
  };
  nav_->async_send_goal(goal, opt);
}

void Coordinator::pick(const SubGoal & g) {
  if (!ready_or_retry(clear_octomap_->service_is_ready() && apply_scene_->service_is_ready() &&
      score_->service_is_ready() && mtc_->action_server_is_ready(), "pick_unavailable")) return;
  const uint32_t object_id = std::stoul(g.target);
  clear_octomap_->async_send_request(std::make_shared<std_srvs::srv::Empty::Request>(),
    [this, object_id](rclcpp::Client<std_srvs::srv::Empty>::SharedFuture) {
      auto scene = std::make_shared<ApplyPlanningScene::Request>();
      scene->scene.is_diff = true;  // TODO: 퍽·주변 장애물 CollisionObject 등록
      apply_scene_->async_send_request(scene, [this, object_id](rclcpp::Client<ApplyPlanningScene>::SharedFuture) {
        auto req = std::make_shared<ScoreGrasps::Request>();
        req->object_id = object_id;
        req->max_candidates = 3;
        score_->async_send_request(req, [this](rclcpp::Client<ScoreGrasps>::SharedFuture f) {
          const auto res = f.get();
          if (!res->success || res->grasps.empty()) { fail("no_grasp"); return; }
          try_grasp(res, 0);
        });
      });
    });
}

void Coordinator::try_grasp(const ScoreGrasps::Response::SharedPtr & cands, size_t i) {
  if (i >= cands->grasps.size() || static_cast<int>(i) >= max_grasp_tries_) {
    fail(arm_contact_ ? "arm_contact" : "grasp_failed_all");
    return;
  }
  RCLCPP_INFO(get_logger(), "파지 후보 %zu (score %.2f)", i, cands->scores[i]);
  execute_mtc("pick:" + std::to_string(i), [this, cands, i](bool ok) { ok ? next() : try_grasp(cands, i + 1); });
}

void Coordinator::execute_mtc(const std::string & task_id, std::function<void(bool)> done) {
  arm_contact_ = false;
  ExecuteTaskSolution::Goal goal;
  goal.solution.task_id = task_id;  // TODO: MTC 태스크를 계획해 solution 채우기
  rclcpp_action::Client<ExecuteTaskSolution>::SendGoalOptions opt;
  opt.result_callback = [done](const rclcpp_action::ClientGoalHandle<ExecuteTaskSolution>::WrappedResult & r) {
    done(r.code == rclcpp_action::ResultCode::SUCCEEDED &&
      r.result->error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
  };
  mtc_->async_send_goal(goal, opt);
}

void Coordinator::status(uint32_t step, uint8_t state, const std::string & reason) {
  TaskStatus s;
  s.plan_id = plan_.plan_id;
  s.step = step;
  s.state = state;
  s.reason = reason;
  status_pub_->publish(s);
}

void Coordinator::say(const std::string & text) {
  std_msgs::msg::String m;
  m.data = text;
  say_pub_->publish(m);
}

}  // namespace plan
