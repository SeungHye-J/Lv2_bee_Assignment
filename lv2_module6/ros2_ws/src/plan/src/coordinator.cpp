// coordinator 구현 — 노드 설명·입출력·멤버 선언은 include/plan/coordinator.hpp, main은 coordinator_main.cpp
#include "plan/coordinator.hpp"
#include "plan/arm_ik.hpp"

#include <algorithm>

#include <cmath>
#include <cstdio>

#include "shape_msgs/msg/solid_primitive.hpp"

namespace plan {

std::vector<std::pair<double, double>> tray_slots(const std::vector<double> & flat) {
  std::vector<std::pair<double, double>> out;
  for (size_t i = 0; i + 1 < flat.size(); i += 2) out.push_back({flat[i], flat[i + 1]});
  return out;
}

geometry_msgs::msg::Pose approach_pose(double ox, double oy, double rx, double ry, double standoff) {
  double dx = ox - rx, dy = oy - ry;
  const double d = std::hypot(dx, dy);
  if (d < 1e-3) { dx = 1.0; dy = 0.0; } else { dx /= d; dy /= d; }  // 로봇 → 물체 방향
  geometry_msgs::msg::Pose p;
  p.position.x = ox - dx * standoff;
  p.position.y = oy - dy * standoff;
  const double yaw = std::atan2(dy, dx);  // 물체를 바라봄
  p.orientation.z = std::sin(yaw / 2);
  p.orientation.w = std::cos(yaw / 2);
  return p;
}

std::pair<double, double> spread_offset(int n, double s) {
  static const int k[][2] = {{0, 0}, {0, 1}, {0, -1}, {1, 0}, {-1, 0}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};
  const int i = n % 9, ring = n / 9 + 1;  // 9개 넘으면 한 칸 더 바깥으로
  return {k[i][0] * s * ring, k[i][1] * s * ring};
}

Coordinator::Coordinator()
: rclcpp::Node("coordinator") {
  max_grasp_tries_ = declare_parameter<int>("max_grasp_tries", 2);
  max_step_failures_ = declare_parameter<int>("max_step_failures", 2);
  server_wait_s_ = declare_parameter<double>("server_wait_s", 30.0);
  arm_plan_tries_ = declare_parameter<int>("arm_plan_tries", 3);
  tray_slots_ = tray_slots(declare_parameter<std::vector<double>>(
    "tray_slots", {-0.135, 0.05, -0.135, -0.05, -0.095, 0.05, -0.095, -0.05}));
  tray_release_z_ = declare_parameter<double>("tray_release_z", 0.22);
  tray_grasp_z_ = declare_parameter<double>("tray_grasp_z", 0.17);
  tray_approach_m_ = declare_parameter<double>("tray_approach_m", 0.05);
  place_spacing_ = declare_parameter<double>("place_spacing", 0.08);
  reach_m_ = declare_parameter<double>("reach_m", 0.30);
  approach_m_ = declare_parameter<double>("approach_m", 0.24);
  inspect_height_m_ = declare_parameter<double>("inspect_height_m", 0.15);
  inspect_wait_s_ = declare_parameter<double>("inspect_wait_s", 1.0);
  arm_yaw_tol_ = declare_parameter<double>("arm_yaw_tolerance", 0.1);
  last_q_ = top_down_q(0.0);
  stand_.puck_l = declare_parameter<double>("stand_puck_length_m", stand_.puck_l);
  stand_.puck_w = declare_parameter<double>("stand_puck_width_m", stand_.puck_w);
  // 깊이는 두 값으로만: 손가락 끝이 퍽 윗면 아래로 들어가는 깊이 · 하중 처짐 보정.
  // 잡는 점은 퍽 윗면 높이(두 모양 공통), 손끝(end_effector_link)은 손가락 끝보다 20.65 mm 위 → ee_up = 20.65 mm + 처짐 − 깊이
  const double grip_depth = declare_parameter<double>("stand_grip_depth_m", 0.010);
  const double sag_comp = declare_parameter<double>("stand_sag_comp_m", 0.027);
  stand_.contact_h_box = stand_.contact_h_cyl = stand_.puck_w;
  stand_.ee_up = 0.02065 + sag_comp - grip_depth;
  stand_.above_m = declare_parameter<double>("stand_above_m", stand_.above_m);
  stand_.lift_away_m = declare_parameter<double>("stand_lift_away_m", stand_.lift_away_m);
  stand_.tilt_over_m = declare_parameter<double>("stand_tilt_over_m", stand_.tilt_over_m);
  stand_.arc_end_deg = declare_parameter<double>("stand_arc_end_deg", stand_.arc_end_deg);
  stand_.set_dz_m = declare_parameter<double>("stand_set_dz_m", stand_.set_dz_m);
  stand_.drag_m = declare_parameter<double>("stand_drag_m", stand_.drag_m);
  stand_.drag_lift_m = declare_parameter<double>("stand_drag_lift_m", stand_.drag_lift_m);
  stand_open_m_ = declare_parameter<double>("stand_open_m", 0.012);
  stand_pinch_m_ = declare_parameter<double>("stand_pinch_m", 0.006);
  stand_pinch_effort_ = declare_parameter<double>("stand_pinch_effort", 0.00538);
  stand_close_effort_ = declare_parameter<double>("stand_close_effort", 0.01076);
  stand_grip_current_ = declare_parameter<double>("stand_grip_current", 0.007);
  stand_empty_m_ = declare_parameter<double>("stand_empty_m", 0.003);
  stand_grip_timeout_ = declare_parameter<double>("stand_grip_timeout_s", 4.0);
  stand_vel_ = declare_parameter<double>("stand_vel_scale", 0.3);
  stand_hang_s_ = declare_parameter<double>("stand_hang_s", 0.5);
  stand_verify_s_ = declare_parameter<double>("stand_verify_s", 1.5);
  support_levels_ = declare_parameter<std::vector<double>>("support_levels", {0.0, 0.07});
  support_snap_m_ = declare_parameter<double>("support_snap_m", 0.05);
  rl_policy_ = declare_parameter<bool>("rl_policy", false);
  rl_open_m_ = declare_parameter<double>("rl_open_m", 0.010);        // 손가락 하나 — 간격 40 mm (학습 open_gap_mm)
  rl_release_m_ = declare_parameter<double>("rl_release_m", 0.008);  // 간격 36 mm (학습 release_gap_mm)
  rl_grasp_z_ = declare_parameter<double>("rl_grasp_z", 0.0606);     // 손끝 ~ 퍽 바닥 (bee_common GRASP_Z)
  rl_above_m_ = declare_parameter<double>("rl_above_m", 0.05);
  initial_enabled_ = declare_parameter<bool>("initial_pose_enabled", true);
  initial_joints_ = declare_parameter<std::vector<std::string>>(
    "initial_pose_joints", {"joint1", "joint2", "joint3", "joint4", "joint5"});
  initial_pose_ = declare_parameter<std::vector<double>>(
    "initial_pose", {0.0, 0.0, 0.0, 0.0, 0.0});
  initial_tol_ = declare_parameter<double>("initial_pose_tolerance", 0.05);
  initial_move_s_ = declare_parameter<double>("initial_pose_move_s", 3.0);
  initial_retries_ = declare_parameter<int>("initial_pose_retries", 1);
  initial_gripper_ = declare_parameter<double>("initial_gripper", 0.0);
  arm_traj_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/arm_controller/joint_trajectory", rclcpp::QoS(10));
  scan_joints_ = declare_parameter<std::vector<std::string>>("scan_pose_joints", {"joint1", "joint2", "joint3", "joint4", "joint5"});
  scan_pose_ = declare_parameter<std::vector<double>>("scan_pose", {2.618, 0.681, -1.414, 1.082, 0.0});
  scan_tol_ = declare_parameter<double>("scan_pose_tolerance", 0.05);
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  const auto latched = rclcpp::QoS(10).reliable().transient_local();
  status_pub_ = create_publisher<TaskStatus>("/task/status", latched);
  say_pub_ = create_publisher<std_msgs::msg::String>("/robot/say", rclcpp::QoS(10).reliable());
  goals_sub_ = create_subscription<SubGoalArray>("/task/sub_goals", latched,
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
  mode_sub_ = create_subscription<std_msgs::msg::String>("/state/operation_mode", rclcpp::QoS(1).reliable().transient_local(),
    [this](std_msgs::msg::String::ConstSharedPtr m) { on_mode(m->data); });
  estop_sub_ = create_subscription<std_msgs::msg::Bool>("/state/emergency", rclcpp::QoS(1).reliable().transient_local(),
    [this](std_msgs::msg::Bool::ConstSharedPtr m) {
      const bool e = m->data;
      if (e == estop_) return;
      estop_ = e;
      RCLCPP_WARN(get_logger(), "비상정지 %s", e ? "ON — 진행 중 전부 취소" : "OFF");
      if (e && running_) {
        cancel_all();
        running_ = false;
        status(plan_.goals.empty() ? 0 : plan_.goals[idx_].step, TaskStatus::FAILED, "estop");
      }
    });
  map_mode_sub_ = create_subscription<std_msgs::msg::String>("/state/map_mode", rclcpp::QoS(1).reliable().transient_local(),
    [this](std_msgs::msg::String::ConstSharedPtr m) { map_mode_ = m->data; check_scan_pose(); });
  joints_sub_ = create_subscription<sensor_msgs::msg::JointState>("/joint_states", rclcpp::QoS(10),
    [this](sensor_msgs::msg::JointState::ConstSharedPtr m) {
      for (size_t i = 0; i < m->name.size() && i < m->position.size(); ++i) joints_[m->name[i]] = m->position[i];
      for (size_t i = 0; i < m->name.size() && i < m->effort.size(); ++i) efforts_[m->name[i]] = m->effort[i];  // 그리퍼 전류 A
    });
  // 시작 자세는 1초부터 자주 확인 (팔이 늦게 뜨는 경우 대비), 스캔 자세는 기존대로 2초
  initial_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() { check_initial_pose(); });
  scan_timer_ = create_wall_timer(std::chrono::seconds(2), [this]() { check_scan_pose(); });
  // initial_pose_enabled 실행 중 변경 — ros2 param set /coordinator initial_pose_enabled false
  param_cb_ = add_post_set_parameters_callback([this](const std::vector<rclcpp::Parameter> & ps) {
    for (const auto & p : ps) {
      if (p.get_name() != "initial_pose_enabled") continue;
      initial_enabled_ = p.as_bool();
      if (!initial_enabled_) {
        initial_timer_->cancel();
        if (initial_waiting_) hold_arm();  // 가는 중이면 그 자리에서 멈춤
        initial_waiting_ = false;
        RCLCPP_INFO(get_logger(), "팔 초기 자세 끔");
      } else {
        initial_sent_count_ = 0;
        initial_timer_->reset();
        RCLCPP_INFO(get_logger(), "팔 초기 자세 켬 — 다시 시도");
      }
    }
  });
  nav_ = rclcpp_action::create_client<NavigateToPose>(this, "/navigate_to_pose");
  motion_plan_ = create_client<GetMotionPlan>("/plan_kinematic_path");
  cartesian_ = create_client<GetCartesianPath>("/compute_cartesian_path");
  jtc_ = rclcpp_action::create_client<FollowJointTrajectory>(this, "/arm_controller/follow_joint_trajectory");
  ik_ = create_client<GetPositionIK>("/compute_ik");
  reset_sag_ = create_client<std_srvs::srv::Trigger>("/rl_arm_correction/reset_sag");
  rl_ctx_pub_ = create_publisher<std_msgs::msg::String>("/rl_policy/context", rclcpp::QoS(5).reliable());
  exec_ = rclcpp_action::create_client<ExecuteTrajectory>(this, "/execute_trajectory");
  gripper_ = rclcpp_action::create_client<ParallelGripperCommand>(this, "/gripper_controller/gripper_cmd");
  score_ = create_client<ScoreGrasps>("/score_grasps");
  clear_octomap_ = create_client<std_srvs::srv::Empty>("/clear_octomap");
  apply_scene_ = create_client<ApplyPlanningScene>("/apply_planning_scene");
  diag_.setHardwareID("beebot");
  diag_.add("상태", [this](diagnostic_updater::DiagnosticStatusWrapper & s) {
    using diagnostic_msgs::msg::DiagnosticStatus;
    if (mode_ != "AUTO") s.summary(DiagnosticStatus::WARN, mode_ + " 모드 — 자동 실행 멈춤");
    else if (waited_s_ > 0) s.summary(DiagnosticStatus::WARN, "Nav2·MoveIt 서버 대기");
    else if (running_) s.summary(DiagnosticStatus::OK, "실행 중: " + step_text());
    else s.summary(DiagnosticStatus::OK, "대기");
    s.add("모드", mode_);
    s.add("plan_id", plan_.plan_id);
    s.add("단계", step_text());
    s.add("실행 중", running_);
    s.add("연속 실패", failures_.size());
    s.add("적재함", std::to_string(tray_count_) + "/" + std::to_string(tray_slots_.size()));
  });
}

void Coordinator::on_mode(const std::string & mode) {
  if (mode == mode_) return;
  mode_ = mode;
  RCLCPP_WARN(get_logger(), "운영 모드: %s", mode.c_str());
  if (mode == "TEACH") {  // 손으로 자세를 잡는 중 — 시작 자세로 끌고 가지 않는다 (나온 뒤에도 그 자세 그대로)
    initial_waiting_ = false;
    if (initial_timer_) initial_timer_->cancel();
  }
  if (mode != "AUTO" && running_) {  // 수동 · 자동 탐사(EXPLORE) · TEACH — 작업 실행과 동시에 하면 안 됨
    cancel_all();
    running_ = false;
    status(idx_ < plan_.goals.size() ? plan_.goals[idx_].step : 0, TaskStatus::FAILED,
           mode == "MANUAL" ? "manual_override" : mode == "TEACH" ? "manual_teach_override" : "explore_override");
  }
}

std::string Coordinator::step_text() const {
  if (idx_ >= plan_.goals.size()) return "-";
  const auto & g = plan_.goals[idx_];
  return std::to_string(g.step) + " " + g.skill + "(" + g.target + ")";
}

namespace {
bool same_goals(const SubGoalArray & a, const SubGoalArray & b) {
  if (a.goals.size() != b.goals.size()) return false;
  for (size_t i = 0; i < a.goals.size(); ++i) {
    const auto & x = a.goals[i], & y = b.goals[i];
    if (x.step != y.step || x.skill != y.skill || x.target != y.target ||
        x.relation != y.relation || x.offset_m != y.offset_m)
      return false;
  }
  return true;
}
}  // namespace

void Coordinator::on_plan(const SubGoalArray & plan) {
  // transient_local 재수신은 무시 — 단, 같은 id에 내용이 다르면 다른 계획 (VLM 카운터와 sim 직접 발행 충돌 대비)
  if (estop_) {  // 비상정지 중엔 새 계획도 안 받음
    RCLCPP_WARN(get_logger(), "계획 %u 거부: 비상정지 중", plan.plan_id);
    status(0, TaskStatus::FAILED, "estop_blocked");
    return;
  }
  if (plan.plan_id == plan_.plan_id && running_ && same_goals(plan, plan_)) return;
  if (plan.plan_id == 0 && plan.goals.empty()) {  // 명시적 정지 (sim 정지 버튼 — VLM 카운터는 1부터라 충돌 없음)
    if (running_) cancel_all();
    running_ = false;
    plan_ = plan;
    RCLCPP_WARN(get_logger(), "정지 요청 — 진행 중 전부 취소");
    status(0, TaskStatus::FAILED, "cancelled");
    return;
  }
  if (mode_ != "AUTO") {
    plan_ = plan;
    RCLCPP_WARN(get_logger(), "계획 %u 거부: %s 모드", plan.plan_id, mode_.c_str());
    status(0, TaskStatus::FAILED, mode_ == "MANUAL" ? "manual_mode" : mode_ == "TEACH" ? "manual_teach_mode" : "explore_mode");
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
  if (running_) cancel_all();  // 진행 중 goal 전부 취소 — 옛 결과가 새 계획을 건드리지 않게
  plan_ = plan;
  idx_ = 0;
  approached_ = static_cast<size_t>(-1);
  placed_.clear();
  running_ = true;
  run_step();
}

bool Coordinator::validate(const SubGoalArray & plan, std::string & why) const {
  static const std::set<std::string> allowed = {"navigate", "pick", "place", "inspect", "stand", "say"};
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
  if (!running_ || mode_ != "AUTO" || estop_) return;
  if (idx_ >= plan_.goals.size()) {
    running_ = false;
    status(plan_.goals.back().step, TaskStatus::SUCCEEDED, "");
    const bool carried = std::any_of(plan_.goals.begin(), plan_.goals.end(),
      [](const SubGoal & s) { return s.skill == "place"; });
    if (carried) say("퍽을 옮겼어요");  // inspect만 한 계획(퍽 번호 보기)은 말 없이 — planner가 이어서 계획
    return;
  }
  const auto & g = plan_.goals[idx_];
  status(g.step, TaskStatus::RUNNING, "");
  RCLCPP_INFO(get_logger(), "step %u: %s %s", g.step, g.skill.c_str(), g.target.c_str());
  if (g.skill == "navigate") navigate(g);
  else if (g.skill == "pick") pick(g);
  else if (g.skill == "inspect") inspect(g);
  else if (g.skill == "stand") stand(g);
  else if (g.skill == "place") {
    if (!ready_or_retry(motion_plan_->service_is_ready() && exec_->action_server_is_ready() &&
        gripper_->action_server_is_ready(), "arm_unavailable")) return;
    if (g.target == "tray") {  // 적재함 다음 칸에 싣기 — 칸 위 tray_release_z에서 벌림
      if (tray_count_ >= static_cast<int>(tray_slots_.size())) { fail("tray_full"); return; }
      execute_place(tray_pose(tray_count_, tray_release_z_), [this](bool ok) {
        if (!ok) { fail("tray_place_failed"); return; }
        ++tray_count_;
        next();
      }, 0.0);
      return;
    }
    if (!is_id(g.target) || !find(std::stoul(g.target))) { fail("place_no_target"); return; }
    if (approach_if_far(g)) return;  // 내려놓을 곳이 팔 밖이면 먼저 다가감
    geometry_msgs::msg::PoseStamped target;
    target.header.frame_id = "map";
    target.header.stamp = now();
    target.pose = find(std::stoul(g.target))->pose.pose;
    const auto [dx, dy] = spread_offset(placed_[g.target], place_spacing_);  // 같은 곳에 여러 개면 비켜서
    target.pose.position.x += dx;
    target.pose.position.y += dy;
    auto after = [this, id = g.target](bool ok) {
      if (!ok) { fail("place_failed"); return; }
      ++placed_[id];
      next();
    };
    if (rl_policy_) rl_place(target, after);  // 학습 시간표 + 잔차 정책
    else execute_place(target, after);
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
    say(g.skill == "navigate" ? "목표까지 갈 수 없어요" : g.skill == "stand" ? "퍽을 세울 수 없어요" : "퍽을 잡을 수 없어요");
  }
}

std::string Coordinator::key(const SubGoal & g) { return g.skill + ":" + g.target; }

bool Coordinator::parse_point(const std::string & target, double & x, double & y) {
  const auto comma = target.find(',');
  if (comma == std::string::npos || comma == 0 || comma + 1 >= target.size()) return false;
  try {
    x = std::stod(target.substr(0, comma));
    y = std::stod(target.substr(comma + 1));
  } catch (const std::exception &) {
    return false;
  }
  return true;
}

bool Coordinator::at_scan_pose() const {
  for (size_t i = 0; i < scan_joints_.size() && i < scan_pose_.size(); ++i) {
    const auto it = joints_.find(scan_joints_[i]);
    if (it == joints_.end()) return false;
    const double d = std::remainder(it->second - scan_pose_[i], 2 * M_PI);  // joint1은 ±π가 같은 자리
    if (std::abs(d) > scan_tol_) return false;
  }
  return true;
}

bool Coordinator::at_initial_pose() const {
  for (size_t i = 0; i < initial_joints_.size() && i < initial_pose_.size(); ++i) {
    const auto it = joints_.find(initial_joints_[i]);
    if (it == joints_.end()) return false;
    if (std::abs(std::remainder(it->second - initial_pose_[i], 2 * M_PI)) > initial_tol_) return false;
  }
  return !joints_.empty();
}

std::string Coordinator::initial_pose_error() const {
  std::string out;
  for (size_t i = 0; i < initial_joints_.size() && i < initial_pose_.size(); ++i) {
    const auto it = joints_.find(initial_joints_[i]);
    if (it == joints_.end()) {
      out += (out.empty() ? "" : ", ") + initial_joints_[i] + " 모름";
      continue;
    }
    const double d = std::remainder(it->second - initial_pose_[i], 2 * M_PI);
    if (std::abs(d) <= initial_tol_) continue;
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%s %+.2f", initial_joints_[i].c_str(), d);
    out += (out.empty() ? "" : ", ") + std::string(buf);
  }
  return out.empty() ? "차이 없음" : out + " rad";
}

void Coordinator::hold_arm() {
  std::vector<std::string> names;
  std::vector<double> pos;
  for (const auto & j : initial_joints_) {
    const auto it = joints_.find(j);
    if (it == joints_.end()) continue;
    names.push_back(j);
    pos.push_back(it->second);
  }
  if (!names.empty()) publish_arm_targets(names, pos, 0.5);
}

void Coordinator::publish_arm_targets(const std::vector<std::string> & names,
                                      const std::vector<double> & pos, double seconds) {
  if (!arm_traj_pub_) return;
  trajectory_msgs::msg::JointTrajectory traj;
  trajectory_msgs::msg::JointTrajectoryPoint point;
  for (size_t i = 0; i < names.size() && i < pos.size(); ++i) {
    traj.joint_names.push_back(names[i]);
    point.positions.push_back(pos[i]);
  }
  point.time_from_start = rclcpp::Duration::from_seconds(seconds);
  traj.points.push_back(point);
  arm_traj_pub_->publish(traj);
}

void Coordinator::check_initial_pose() {
  // bringup 직후 한 번, 손으로 잡아 둔 초기 자세로 팔을 옮긴다.
  //  - MAPPING/EXPLORE면 스캔 자세가 우선이라 건너뜀
  //  - estop·진행 중 작업·비상정지 중에는 안 함
  //  - 이미 그 자세면 바로 완료
  if (!initial_enabled_) return;
  if (initial_waiting_) {
    // 보낸 뒤 도달 확인 — 도달하면 타이머 자체를 끔, 6초 넘게 못 가면 한 번 더 보냄 (JTC 재시작·일시 거절 대비)
    if (at_initial_pose()) {
      initial_waiting_ = false;
      if (initial_gripper_sent_ || initial_gripper_ < 0.0) initial_timer_->cancel();  // 그리퍼가 남았으면 다음 확인 때
      RCLCPP_INFO(get_logger(), "팔 초기 자세 완료");
    } else if (std::chrono::duration<double>(
                 std::chrono::steady_clock::now() - initial_sent_).count() > initial_move_s_ + 3.0) {
      initial_waiting_ = false;
      if (initial_sent_count_ > initial_retries_) {
        // 어딘가에 걸려 못 가는 것 — 계속 밀면 모터 과부하(Hardware Error)로 멈춘다
        initial_timer_->cancel();
        hold_arm();
        RCLCPP_ERROR(get_logger(), "팔 초기 자세 %d번 미도달 — 포기하고 지금 자세로 멈춤 (%s). "
                     "plan.yaml initial_pose 확인", initial_sent_count_, initial_pose_error().c_str());
      } else {
        RCLCPP_WARN(get_logger(), "팔 초기 자세 미도달 (%s) — 다시 보냄", initial_pose_error().c_str());
      }
    }
    return;
  }
  if (estop_ || running_ || joints_.empty()) return;
  if (!initial_gripper_sent_ && initial_gripper_ >= 0.0 && gripper_->action_server_is_ready()) {
    initial_gripper_sent_ = true;  // 한 번만 — 실패해도 다시 안 보냄 (쥔 채 계속 힘주지 않게)
    RCLCPP_INFO(get_logger(), "그리퍼를 시작 값으로 %.4fm", initial_gripper_);
    set_gripper(initial_gripper_, [this](bool ok) {
      if (!ok) RCLCPP_WARN(get_logger(), "그리퍼 시작 값 실패 — plan.yaml initial_gripper 확인");
    });
  }
  if (map_mode_ == "MAPPING" || mode_ == "EXPLORE") {
    initial_timer_->cancel();  // 스캔 자세(coordinator check_scan_pose)가 대신 잡는다
    return;
  }
  if (at_initial_pose()) {
    if (initial_gripper_sent_ || initial_gripper_ < 0.0) {  // 그리퍼 서버가 아직 없으면 1초마다 다시 확인
      initial_timer_->cancel();
      RCLCPP_INFO(get_logger(), "팔 초기 자세 확인 끝");
    }
    return;
  }
  if (arm_traj_pub_->get_subscription_count() == 0) return;  // JTC 아직 없음
  initial_waiting_ = true;
  ++initial_sent_count_;
  initial_sent_ = std::chrono::steady_clock::now();
  RCLCPP_INFO(get_logger(), "팔을 초기 자세로 이동 (%.1fs)", initial_move_s_);
  publish_arm_targets(initial_joints_, initial_pose_, initial_move_s_);
}

void Coordinator::check_scan_pose() {
  // 지도를 만드는 중(스캔 · 자동 탐사)이고 작업이 없을 때 — 작업(pick 등)이 끝나면 다음 확인 때 돌아감.
  // 수동 모드여도 (스캔은 보통 수동으로 몰고 다님 — 팔 수동 조작은 MAPPING 동안 manual_control·service_action_input이 막음)
  if (map_mode_ != "MAPPING" && mode_ != "EXPLORE") return;
  if (estop_ || running_ || scan_moving_ || joints_.empty() || at_scan_pose()) return;
  if (!motion_plan_->service_is_ready() || !exec_->action_server_is_ready()) return;
  std::map<std::string, double> target;
  for (size_t i = 0; i < scan_joints_.size() && i < scan_pose_.size(); ++i) target[scan_joints_[i]] = scan_pose_[i];
  scan_moving_ = true;
  RCLCPP_INFO(get_logger(), "지도 만드는 중 — 팔을 스캔 자세로 (gripper_cam이 왼쪽 뒤를 봄)");
  move_joints(target, [this](bool ok) {
    scan_moving_ = false;
    if (!ok) RCLCPP_WARN(get_logger(), "스캔 자세로 못 감 — 2초 뒤 다시");
  });
}

void Coordinator::move_joints(const std::map<std::string, double> & target, std::function<void(bool)> done, bool for_task) {
  auto req = std::make_shared<GetMotionPlan::Request>();
  auto & r = req->motion_plan_request;
  r.group_name = "arm";
  r.start_state.is_diff = true;
  r.num_planning_attempts = 3;
  r.allowed_planning_time = 5.0;
  r.max_velocity_scaling_factor = 0.3;
  r.max_acceleration_scaling_factor = 0.3;
  moveit_msgs::msg::Constraints c;
  for (const auto & [name, pos] : target) {
    moveit_msgs::msg::JointConstraint jc;
    jc.joint_name = name;
    jc.position = pos;
    jc.tolerance_above = jc.tolerance_below = 0.01;
    jc.weight = 1.0;
    c.joint_constraints.push_back(jc);
  }
  r.goal_constraints.push_back(c);
  // 작업 취소(gen_)와 상관없이 끝은 늘 알림 — scan_moving_이 걸린 채 남지 않게
  const uint64_t gen0 = gen_;
  motion_plan_->async_send_request(req, [this, done, for_task, gen0](rclcpp::Client<GetMotionPlan>::SharedFuture f) {
    const auto res = f.get();
    if (for_task && gen0 != gen_) return;  // 작업용: 그새 취소됨
    // 스캔 자세용(for_task false)은 작업이 시작됐으면 안 움직임 · 작업용은 실행 중이 정상
    if (res->motion_plan_response.error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS || (!for_task && running_)) {
      done(false);
      return;
    }
    ExecuteTrajectory::Goal goal;
    goal.trajectory = res->motion_plan_response.trajectory;
    rclcpp_action::Client<ExecuteTrajectory>::SendGoalOptions opt;
    opt.goal_response_callback = [done](const rclcpp_action::ClientGoalHandle<ExecuteTrajectory>::SharedPtr & h) {
      if (!h) done(false);
    };
    opt.result_callback = [done](const rclcpp_action::ClientGoalHandle<ExecuteTrajectory>::WrappedResult & res2) {
      done(res2.code == rclcpp_action::ResultCode::SUCCEEDED &&
        res2.result->error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
    };
    exec_->async_send_goal(goal, opt);
  });
}

void Coordinator::cancel_all() {
  ++gen_;
  if (retry_) retry_->cancel();
  if (stand_timer_) stand_timer_->cancel();
  if (grip_timer_) grip_timer_->cancel();
  if (rl_timer_) rl_timer_->cancel();
  if (jtc_) jtc_->async_cancel_all_goals();
  if (rl_ctx_pub_) rl_context_stop();
  nav_->async_cancel_all_goals();
  exec_->async_cancel_all_goals();
  gripper_->async_cancel_all_goals();
}

std::string Coordinator::world_frame() const {
  // 지도(SLAM)가 없으면 map 프레임이 없다 — perception은 그때 odom 좌표를 map이라 적어 냄 (map 원점 = odom 원점, perception_node to_map)
  return tf_buffer_->canTransform("map", "base_footprint", tf2::TimePointZero) ? "map" : "odom";
}

void Coordinator::fix_frame(std_msgs::msg::Header & h) const {
  if (h.frame_id == "map") h.frame_id = world_frame();
}

bool Coordinator::robot_xy(double & x, double & y) const {
  try {
    const auto t = tf_buffer_->lookupTransform(world_frame(), "base_footprint", tf2::TimePointZero);
    x = t.transform.translation.x;
    y = t.transform.translation.y;
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

bool Coordinator::approach_if_far(const SubGoal & g) {
  if (approached_ == idx_ || !is_id(g.target)) return false;  // 이 단계는 이미 다가감 → 그대로 실행 (안 닿으면 그 단계가 실패)
  const auto * o = find(std::stoul(g.target));
  double rx, ry;
  if (!o || !robot_xy(rx, ry)) return false;  // 로봇 위치를 모르면 판단 못 함 — 그대로 실행
  const double ox = o->pose.pose.position.x, oy = o->pose.pose.position.y;
  const double d = std::hypot(ox - rx, oy - ry);
  if (d <= reach_m_) return false;
  if (!ready_or_retry(nav_->action_server_is_ready(), "nav_unavailable")) return true;
  approached_ = idx_;
  RCLCPP_INFO(get_logger(), "%s %s: 대상이 %.2fm — 팔이 닿게 %.2fm 앞까지 다가감", g.skill.c_str(), g.target.c_str(), d, approach_m_);
  send_nav(approach_pose(ox, oy, rx, ry, approach_m_), [this](bool ok) {
    if (ok) run_step();  // 같은 단계를 다시 (이번엔 다가가기를 건너뜀)
    else fail("approach_failed");
  });
  return true;
}

void Coordinator::navigate(const SubGoal & g) {
  if (!ready_or_retry(nav_->action_server_is_ready(), "nav_unavailable")) return;
  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1.0;
  if (is_id(g.target)) {
    // 대상 앞 (물체 반 크기 + offset + 0.1) — 로봇이 있는 쪽에서 물체를 바라봄. TODO: 통로 방향 고려 (미로 벽에 붙지 않게)
    const auto * o = find(std::stoul(g.target));
    const double ox = o->pose.pose.position.x, oy = o->pose.pose.position.y;
    double rx = ox - 1.0, ry = oy;  // 로봇 위치를 모르면 물체의 −x 쪽 (예전과 같게)
    robot_xy(rx, ry);
    pose = approach_pose(ox, oy, rx, ry, std::max(o->size.x, o->size.y) / 2 + g.offset_m + 0.1);
  } else {
    double x, y;
    if (parse_point(g.target, x, y)) {  // 직접 지정 "x,y" (시뮬레이터 지도 클릭)
      pose.position.x = x;
      pose.position.y = y;
    }
  }
  send_nav(pose, [this](bool ok) { ok ? next() : fail("nav_failed"); });
}

void Coordinator::send_nav(const geometry_msgs::msg::Pose & pose, std::function<void(bool)> done) {
  NavigateToPose::Goal goal;
  goal.pose.header.frame_id = "map";
  goal.pose.header.stamp = now();
  goal.pose.pose = pose;
  const uint64_t gen = gen_;
  rclcpp_action::Client<NavigateToPose>::SendGoalOptions opt;
  opt.goal_response_callback =
    [this](const rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr & h) {
      if (h) nav_goal_ = h;
    };
  opt.result_callback = [this, gen, done](const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult & r) {
    if (gen != gen_) return;  // 취소된 옛 goal의 늦은 결과
    done(r.code == rclcpp_action::ResultCode::SUCCEEDED);
  };
  nav_->async_send_goal(goal, opt);
}

void Coordinator::pick(const SubGoal & g) {
  if (g.target == "tray") {  // 적재함에서 마지막에 실은 것부터 꺼내기 — 칸 위치를 알아서 grasp_scorer 없이
    if (!ready_or_retry(motion_plan_->service_is_ready() && exec_->action_server_is_ready() &&
        gripper_->action_server_is_ready(), "pick_unavailable")) return;
    if (tray_count_ <= 0) { fail("tray_empty"); return; }
    execute_pick(tray_pose(tray_count_ - 1, tray_grasp_z_), [this](bool ok) {
      if (!ok) { fail("tray_pick_failed"); return; }
      --tray_count_;
      next();
    }, tray_approach_m_);
    return;
  }
  if (is_id(g.target)) {  // 누운 퍽은 위에서 못 집음 (손가락이 상자에 닿음) — stand로 세운 뒤 pick
    const auto * o = find(std::stoul(g.target));
    if (o && is_lying(*o)) { fail("pick_lying"); return; }
  }
  if (!ready_or_retry(clear_octomap_->service_is_ready() && apply_scene_->service_is_ready() &&
      score_->service_is_ready() && motion_plan_->service_is_ready() &&
      exec_->action_server_is_ready() && gripper_->action_server_is_ready(), "pick_unavailable")) return;
  if (approach_if_far(g)) return;  // 집을 퍽이 팔 밖이면 먼저 다가감 (VLM이 navigate를 빼먹어도)
  const uint32_t object_id = std::stoul(g.target);
  pick_target_ = object_id;
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

void Coordinator::inspect(const SubGoal & g) {
  if (!ready_or_retry(motion_plan_->service_is_ready() && exec_->action_server_is_ready(), "arm_unavailable")) return;
  if (!is_id(g.target) || !find(std::stoul(g.target))) { fail("inspect_no_target"); return; }
  if (approach_if_far(g)) return;  // 팔 밖이면 먼저 다가감
  const auto * o = find(std::stoul(g.target));
  geometry_msgs::msg::PoseStamped above;
  above.header.frame_id = "map";
  above.header.stamp = now();
  above.pose = o->pose.pose;
  above.pose.position.z += o->size.z / 2 + inspect_height_m_;  // 윗면 위 — 손가락 아래라 gripper_cam이 윗면을 봄
  const uint64_t gen = gen_;
  move_arm(above, [this, gen, id = g.target](bool ok) {
    if (!ok) { fail("inspect_failed"); return; }
    inspect_timer_ = create_wall_timer(std::chrono::duration<double>(inspect_wait_s_), [this, gen, id]() {
      inspect_timer_->cancel();
      if (gen != gen_) return;  // 그새 취소됨
      const auto * obj = find(std::stoul(id));
      const bool read = obj && std::any_of(obj->details.begin(), obj->details.end(),
        [](const diagnostic_msgs::msg::KeyValue & kv) { return kv.key == "tag"; });
      RCLCPP_INFO(get_logger(), "inspect %s: 태그 %s", id.c_str(), read ? "읽음" : "못 읽음");
      next();
    });
  });
}

void Coordinator::stand(const SubGoal & g) {
  // 대상 확인은 서버 대기보다 먼저 — 세울 수 없는 대상이면 바로 실패
  if (!is_id(g.target) || !find(std::stoul(g.target))) { fail("stand_no_target"); return; }
  const uint32_t id = std::stoul(g.target);
  if (!is_lying(*find(id))) { fail("stand_not_lying"); return; }
  double axis = 0.0;
  if (!object_yaw(*find(id), axis)) { fail("stand_no_axis"); return; }  // 긴 축 방향을 perception이 못 구함
  if (!ready_or_retry(motion_plan_->service_is_ready() && ik_->service_is_ready() && jtc_->action_server_is_ready() &&
      exec_->action_server_is_ready() && gripper_->action_server_is_ready(), "stand_unavailable")) return;
  if (approach_if_far(g)) return;  // 팔 밖이면 먼저 다가감
  const auto * o = find(id);
  if (!o) { fail("stand_no_target"); return; }
  object_yaw(*o, axis);  // 다가가는 동안 다시 봤을 수 있음
  const auto & c = o->pose.pose.position;
  double rx = c.x - 1.0, ry = c.y;  // 로봇 위치를 모르면 퍽의 −x 쪽에 있다고 봄
  robot_xy(rx, ry);
  const double far = far_axis_yaw(axis, c.x, c.y, rx, ry);  // 먼 쪽 끝을 잡고 가까운 끝 모서리를 축으로
  const bool box = detail(*o, "shape") != "cyl";             // 모양을 모르면 사각기둥 높이 (윗면 — 더 높아 상자에 안 닿음)
  const double support_raw = support_top_z(*o, objects_->objects, stand_.puck_w);
  const double support = snap_support(support_raw, support_levels_, support_snap_m_);  // 바닥 · 상자 높이로 (잰 값 오차)
  if (support != support_raw) RCLCPP_INFO(get_logger(), "stand %u: 받침 z %.3f → %.3f (support_levels)", id, support_raw, support);
  const StandPath path = stand_path(c.x, c.y, support, far, box, stand_);
  RCLCPP_INFO(get_logger(), "stand %u: 축 %.0f° (먼 끝 쪽) · %s · 받침 z %.3f", id, far * 180.0 / M_PI,
              box ? "사각기둥" : "원기둥", path.pinch.position.z - stand_.ee_up - (box ? stand_.contact_h_box : stand_.contact_h_cyl));
  arm_contact_ = false;
  last_q_ = top_down_q(path.tool_yaw);  // IK가 ψ · ψ+π 중 지금 자세에 가까운 쪽을 고름 (손이 대칭)
  stand_preview_ = g.relation == "preview";  // 미리보기: 잡을 자리 위에서 멈추기만 (손 위치 눈으로 확인)
  RCLCPP_INFO(get_logger(), "stand %u: 퍽 중심 %s (%.3f, %.3f, %.3f) · 축 %.0f° · 로봇 (%.3f, %.3f) · 잡을 자리 (%.3f, %.3f, %.3f)%s", id,
              world_frame().c_str(), c.x, c.y, c.z, axis * 180.0 / M_PI, rx, ry, path.pinch.position.x, path.pinch.position.y,
              path.pinch.position.z, stand_preview_ ? " — 미리보기" : "");
  stand_run(id, path);  // 경로 전체 IK를 먼저 풀고 — 하나라도 안 되면 팔을 안 움직이고 실패
}

void Coordinator::stand_run(uint32_t id, StandPath path) {
  // 위에서 풀린 손 방향(ψ 또는 ψ+π)을 나머지 점 전부에 — 점마다 IK를 앞 점에서 이어 풀어 관절 경유점으로
  std::vector<geometry_msgs::msg::PoseStamped> poses;
  auto add = [&](const geometry_msgs::msg::Pose & p) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = "map";  // compute_ik가 지도 없으면 odom으로 (fix_frame)
    ps.header.stamp = now();
    ps.pose = p;
    ps.pose.orientation = last_q_;
    poses.push_back(ps);
  };
  add(path.pinch);
  for (const auto & p : path.arc_up) add(p);
  for (const auto & p : path.arc_back) add(p);
  // 들어 올리기는 IK 안 함 — 손을 아래로 향한 채로는 8 cm 위까지 못 가는 자리가 많음(실측 IK). 놓은 뒤라 MoveIt(방향 허용)으로
  const size_t n_up = path.arc_up.size(), n_back = path.arc_back.size();
  const uint64_t gen = gen_;
  // 점마다 이 팔 전용 IK (plan/arm_ik — 위치 + 손이 아래 + 손목 각을 같이). MoveIt IK(pick_ik)는 위치만 맞춰 손 방향을 보장 못 함
  std::vector<std::vector<double>> qs;
  std::vector<double> q_above;
  {
    // 먼 끝 위 대기 자리 — 손을 아래로 향한 채 높이 못 가는 거리면 4 → 3 → 2 → 1 cm로 낮춤 (퍽 0.276 m에서 4 cm 위가 안 닿아
    //   MoveIt 계획이 3번 다 실패했음 — 실기 11/03)
    double above_h = -1.0;
    for (double h : {stand_.above_m, 0.03, 0.02, 0.01}) {
      auto ap = path.pinch;
      ap.position.z += h;
      ap.orientation = last_q_;
      std::array<double, 3> pa;
      double psi = 0.0;
      if (to_arm_frame(ap, pa, psi) && arm_ik_down(pa, psi, arm_now(), q_above)) { above_h = h; break; }
    }
    if (above_h < 0) {
      std::array<double, 3> pa{0, 0, 0};
      double psi = 0.0;
      to_arm_frame(poses[0].pose, pa, psi);
      RCLCPP_WARN(get_logger(), "stand %u: 먼 끝 위로 손을 아래로 향한 채 못 감 (잡는 자리 arm_base 기준 %.3f %.3f %.3f) — "
                  "퍽을 로봇 중심 0.22~0.25 m로", id, pa[0], pa[1], pa[2]);
      fail("stand_plan_failed");
      return;
    }
    if (above_h < stand_.above_m) RCLCPP_INFO(get_logger(), "stand %u: 대기 높이 %.0f → %.0f cm (닿는 높이)", id, 100 * stand_.above_m, 100 * above_h);
    std::vector<double> seed = q_above;
    for (size_t k = 0; k < poses.size(); ++k) {
      std::array<double, 3> pa;
      double psi = 0.0;
      std::vector<double> q;
      bool ok = to_arm_frame(poses[k].pose, pa, psi) && arm_ik_down(pa, psi, seed, q);
      double jump = 0.0;
      if (ok && !qs.empty()) for (size_t i = 0; i < q.size(); ++i) jump = std::max(jump, std::abs(q[i] - qs.back()[i]));
      if (!ok || jump > 0.8) {  // 못 풂 · 이웃 점인데 크게 튐(손목 뒤집힘)
        static const char * names[] = {"잡는 자리", "호", "되돌아오기"};
        const int part = k == 0 ? 0 : k <= n_up ? 1 : 2;
        RCLCPP_WARN(get_logger(), "stand %u: IK 실패 — %zu/%zu번째 점 (%s, arm_base 기준 %.3f %.3f %.3f%s)", id, k + 1, poses.size(),
                    names[part], pa[0], pa[1], pa[2], ok ? ", 앞 점과 크게 튐" : " — 손을 아래로 향한 채 닿지 않음");
        fail("stand_plan_failed");
        return;
      }
      qs.push_back(q);
      seed = q;
    }
  }
  {
    const auto q_pinch = qs[0];
    std::vector<std::vector<double>> up{q_pinch}, back;
    up.insert(up.end(), qs.begin() + 1, qs.begin() + 1 + n_up);
    back.push_back(up.back());
    back.insert(back.end(), qs.begin() + 1 + n_up, qs.begin() + 1 + n_up + n_back);
    // 손 빼기 — 내려놓은 점에서 위로. 손을 아래로 향한 채 높이 못 가는 자리가 많아 8 → 6 → 4 → 3 cm로 낮춰 가며 IK
    //   (MoveIt은 처짐 보정으로 지금 관절이 궤적 시작과 0.01 rad 넘게 달라 'start point deviates'로 거부했음 — 실기 11/03)
    std::vector<std::vector<double>> lift;
    for (double h : {stand_.lift_away_m, 0.06, 0.04, 0.03}) {
      auto lp = path.arc_back.back();
      lp.position.z += h;
      lp.orientation = last_q_;
      std::array<double, 3> pa;
      double psi = 0.0;
      std::vector<double> q;
      if (to_arm_frame(lp, pa, psi) && arm_ik_down(pa, psi, back.back(), q)) { lift = {back.back(), q}; break; }
    }
    std::map<std::string, double> above_goal;
    for (size_t i = 0; i < q_above.size(); ++i) above_goal["joint" + std::to_string(i + 1)] = q_above[i];
    RCLCPP_INFO(get_logger(), "stand %u: 경로 IK %zu점 다 풀림 — 시작", id, poses.size() + 1);
    set_gripper(stand_open_m_, [=, this](bool ok0) {  // 벌리기 (옆 퍽을 안 치게 조금만)
    if (!ok0) { fail("stand_failed"); return; }
    // 먼 끝 위로: MoveIt 관절 목표 (몸과 안 부딪히는 경로) — 못 풀면 관절 직선
    auto go_down = [=, this]() {
    log_hand(id, "대기 자리", path.pinch.position.x, path.pinch.position.y, NAN);
    if (stand_preview_) {  // 미리보기 — 잡을 자리 위에서 멈춤 (그리퍼는 벌린 채)
      RCLCPP_INFO(get_logger(), "stand %u: 미리보기 끝 — 손이 잡을 자리 위에 있음", id);
      next();
      return;
    }
    move_joints_path({arm_now(), q_pinch}, stand_vel_, [=, this](bool ok1) {  // 곧게 내려감 (관절 직선 — 몇 cm라 거의 수직)
      if (!ok1) { fail("stand_plan_failed"); return; }
      log_hand(id, "잡는 자리", path.pinch.position.x, path.pinch.position.y, path.pinch.position.z);
      // 잡기 두 단계: ① stand_close_effort로 목표 간격까지 확실히 닫고 ② 같은 자리에서 stand_pinch_effort로 낮춰 살짝 쥠
      //   (5.38 mA로 바로 닫으면 손가락이 마찰을 못 이겨 거의 안 움직이는데 컨트롤러는 멈춤 = 파지로 보고 넘어감 — 실기 11/03)
      //   그리퍼 액션 결과는 손가락이 멈추기만 해도(덜 닫혀도) 성공이라 안 믿고, 전류(토크)가 걸려 손가락이 퍽에 멈춘 걸 보고 넘어감
      //   (결과만 보고 넘어가 호를 그리는 동안 손가락이 닫혔음 — 실기 11/03)
      set_gripper(stand_pinch_m_, [](bool) {}, stand_close_effort_);
      wait_grip([=, this](int grip) {
        if (grip <= 0) { stand_abort(grip == 0 ? "stand_no_grip" : "stand_grip_timeout"); return; }
        set_gripper(stand_pinch_m_, [](bool) {}, stand_pinch_effort_);  // 살짝 쥐기 — 퍽이 손가락 사이에서 돌 만큼
        stand_timer_ = create_wall_timer(std::chrono::milliseconds(300), [=, this]() {  // 전류 한계가 바뀔 시간
        stand_timer_->cancel();
        if (gen != gen_) return;
        move_joints_path(up, stand_vel_, [=, this](bool ok3) {  // 호: 선 자세를 넘겨 돌림
          if (!ok3) { stand_abort("stand_arc_failed"); return; }
          stand_timer_ = create_wall_timer(std::chrono::duration<double>(stand_hang_s_), [=, this]() {  // 잠깐 멈춤
            stand_timer_->cancel();
            if (gen != gen_) return;  // 그새 취소됨
            move_joints_path(back, stand_vel_, [=, this](bool ok4) {  // 90°로 돌아와 내려놓기
              if (!ok4) { stand_abort("stand_arc_failed"); return; }
              set_gripper(stand_open_m_, [=, this](bool ok5) {  // 놓기
                if (!ok5) { fail("stand_failed"); return; }
                if (lift.empty()) {  // 위로 뺄 자리를 못 풂 — 벌린 채 그 자리에서 확인
                  RCLCPP_WARN(get_logger(), "stand %u: 손 빼기 IK 실패 — 그 자리에서 확인", id);
                  stand_verify(id);
                  return;
                }
                move_joints_path(lift, stand_vel_, [=, this](bool ok6) {  // 손 빼기
                  if (!ok6) RCLCPP_WARN(get_logger(), "stand %u: 손 빼기 궤적 실패 — 그 자리에서 확인", id);
                  stand_verify(id);
                });
              });
            });
          });
        });
        });
      });
    });
    };
    move_joints(above_goal, [=, this](bool okA) {
      if (okA) { go_down(); return; }
      RCLCPP_WARN(get_logger(), "stand %u: 먼 끝 위로 MoveIt 계획 실패 — 관절 직선으로", id);
      move_joints_path({arm_now(), q_above}, stand_vel_, [=, this](bool okB) {
        if (!okB) { fail("stand_plan_failed"); return; }
        go_down();
      });
    }, true);
    });
  }
}

bool Coordinator::to_arm_frame(const geometry_msgs::msg::Pose & p, std::array<double, 3> & out, double & psi) const {
  // 물체 좌표계(map · 지도 없으면 odom) 손끝 목표 → arm_base_link (팔 받침은 수평 — 수직축 둘레 회전만 손 방향 ψ에 더함)
  double psi_w = 0.0;
  if (!top_down_yaw(p.orientation, psi_w)) return false;
  try {
    const auto t = tf_buffer_->lookupTransform("arm_base_link", world_frame(), tf2::TimePointZero).transform;
    const auto & r = t.rotation;
    // v' = R v + t — 쿼터니언 → 회전 행렬
    const double R[3][3] = {{1 - 2 * (r.y * r.y + r.z * r.z), 2 * (r.x * r.y - r.w * r.z), 2 * (r.x * r.z + r.w * r.y)},
                            {2 * (r.x * r.y + r.w * r.z), 1 - 2 * (r.x * r.x + r.z * r.z), 2 * (r.y * r.z - r.w * r.x)},
                            {2 * (r.x * r.z - r.w * r.y), 2 * (r.y * r.z + r.w * r.x), 1 - 2 * (r.x * r.x + r.y * r.y)}};
    const double v[3] = {p.position.x, p.position.y, p.position.z}, tr[3] = {t.translation.x, t.translation.y, t.translation.z};
    for (int i = 0; i < 3; ++i) out[i] = R[i][0] * v[0] + R[i][1] * v[1] + R[i][2] * v[2] + tr[i];
    psi = psi_w + std::atan2(2 * (r.w * r.z + r.x * r.y), 1 - 2 * (r.y * r.y + r.z * r.z));
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

void Coordinator::move_joints_path(const std::vector<std::vector<double>> & pts, double vel_scale, std::function<void(bool)> done) {
  if (!jtc_->action_server_is_ready()) { done(false); return; }
  FollowJointTrajectory::Goal goal;
  goal.trajectory.joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5"};
  for (const auto & [t, q] : joint_path(pts, vel_scale)) {
    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions = q;
    pt.time_from_start = rclcpp::Duration::from_seconds(t);
    goal.trajectory.points.push_back(pt);
  }
  const uint64_t gen = gen_;
  rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions opt;
  opt.result_callback = [this, done, gen](const rclcpp_action::ClientGoalHandle<FollowJointTrajectory>::WrappedResult & r) {
    if (gen != gen_) return;
    const bool ok = r.code == rclcpp_action::ResultCode::SUCCEEDED && r.result->error_code == 0;
    if (!ok) RCLCPP_WARN(get_logger(), "관절 궤적 실패 (%d)", r.result ? r.result->error_code : -1);
    done(ok);
  };
  jtc_->async_send_goal(goal, opt);
}

void Coordinator::wait_grip(std::function<void(int)> done) {
  // 50 ms마다 그리퍼 손가락 위치·전류를 봄 → 1 = 퍽을 쥠 (전류 ≥ stand_grip_current · 0.2 s 동안 안 움직임 · 간격이 빈손보다 넓음)
  //   0 = 빈손 (손가락이 stand_empty_m보다 더 닫힘 — 퍽이 없거나 빠짐) · −1 = stand_grip_timeout_s 안에 판단 못 함
  const uint64_t gen = gen_;
  auto t0 = std::make_shared<std::chrono::steady_clock::time_point>(std::chrono::steady_clock::now());
  auto still_since = std::make_shared<std::chrono::steady_clock::time_point>(*t0);
  auto last = std::make_shared<double>(-1.0);
  grip_timer_ = create_wall_timer(std::chrono::milliseconds(50), [this, gen, done, t0, still_since, last]() {
    if (gen != gen_) { grip_timer_->cancel(); return; }
    const auto now_t = std::chrono::steady_clock::now();
    const auto p = joints_.find("gripper_left_joint");
    const auto e = efforts_.find("gripper_left_joint");
    if (p == joints_.end()) return;
    const double pos = p->second, cur = e == efforts_.end() ? 0.0 : std::abs(e->second);
    if (*last < 0 || std::abs(pos - *last) > 0.0002) *still_since = now_t;  // 움직이는 중
    *last = pos;
    const double still_s = std::chrono::duration<double>(now_t - *still_since).count();
    const double t = std::chrono::duration<double>(now_t - *t0).count();
    int r = 2;  // 2 = 아직
    if (still_s > 0.2 && pos < stand_empty_m_) r = 0;
    else if (still_s > 0.2 && cur >= stand_grip_current_) r = 1;
    else if (t > stand_grip_timeout_) r = -1;
    if (r == 2) return;
    grip_timer_->cancel();
    RCLCPP_INFO(get_logger(), "그리퍼 %s: 간격 %.1f mm · 전류 %.1f mA (%.1f s)", r == 1 ? "쥠" : r == 0 ? "빈손" : "판단 못 함",
                20.0 + 2000.0 * pos, 1000.0 * cur, t);
    done(r);
  });
}

void Coordinator::log_hand(uint32_t id, const char * where, double px, double py, double pz) {
  // 실제 손끝(TF) − 계획 — 손이 엉뚱한 데로 가면 퍽 위치(perception)·좌표 변환·IK 중 어디가 틀렸는지 가르는 기준
  try {
    const auto t = tf_buffer_->lookupTransform(world_frame(), "end_effector_link", tf2::TimePointZero).transform.translation;
    RCLCPP_INFO(get_logger(), "stand %u: %s 손끝 (%.3f, %.3f, %.3f) · 계획과 차 x %+.1f y %+.1f%s mm", id, where, t.x, t.y, t.z,
                1000 * (t.x - px), 1000 * (t.y - py), std::isnan(pz) ? "" : (" z " + std::to_string(static_cast<int>(1000 * (t.z - pz)))).c_str());
  } catch (const std::exception &) {}
}

void Coordinator::stand_abort(const std::string & reason) {
  // 잡은 채 경로가 끊김 — 그 자리에서 벌려 퍽을 놓고 실패 (든 채로 멈추지 않게)
  set_gripper(stand_open_m_, [this, reason](bool) { fail(reason); });
}

void Coordinator::stand_verify(uint32_t id) {
  const uint64_t gen = gen_;
  stand_timer_ = create_wall_timer(std::chrono::duration<double>(stand_verify_s_), [this, gen, id]() {
    stand_timer_->cancel();
    if (gen != gen_) return;
    const auto * o = find(id);  // perception은 퍽 자세가 바뀌어도 같은 id
    if (!o) {
      RCLCPP_WARN(get_logger(), "stand %u: 퍽이 목록에 없어 섰는지 확인 못 함 — 성공으로 봄", id);
      next();
    } else if (is_lying(*o)) {
      fail("stand_failed");
    } else {
      RCLCPP_INFO(get_logger(), "stand %u: 섰음", id);
      next();
    }
  });
}

std::vector<double> Coordinator::arm_now() const {
  std::vector<double> q;
  for (const char * j : {"joint1", "joint2", "joint3", "joint4", "joint5"}) {
    const auto it = joints_.find(j);
    q.push_back(it == joints_.end() ? 0.0 : it->second);
  }
  return q;
}

void Coordinator::rl_context_stop() {
  std_msgs::msg::String m;
  m.data = rl_context_json("", 0.0, RlTimeline{}, {}, {}, 0, false, 0.0, {});
  rl_ctx_pub_->publish(m);
}

void Coordinator::compute_ik(const geometry_msgs::msg::PoseStamped & pose, const std::vector<double> & seed,
                             std::function<void(bool, std::vector<double>)> done) {
  auto req = std::make_shared<GetPositionIK::Request>();
  req->ik_request.group_name = "arm";
  req->ik_request.ik_link_name = "end_effector_link";
  req->ik_request.pose_stamped = pose;
  fix_frame(req->ik_request.pose_stamped.header);
  req->ik_request.avoid_collisions = false;  // 경로는 이미 MoveIt으로 퍽 위까지 — 여기선 위·아래 관절값만
  req->ik_request.timeout.nanosec = 200000000;
  req->ik_request.robot_state.is_diff = true;
  req->ik_request.robot_state.joint_state.name = {"joint1", "joint2", "joint3", "joint4", "joint5"};
  req->ik_request.robot_state.joint_state.position = seed;  // 가까운 해 (손목이 뒤집히지 않게)
  const uint64_t gen0 = gen_;
  ik_->async_send_request(req, [this, done, gen0](rclcpp::Client<GetPositionIK>::SharedFuture f) {
    if (gen0 != gen_) return;
    const auto res = f.get();
    if (res->error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) { done(false, {}); return; }
    std::vector<double> q;
    for (const char * j : {"joint1", "joint2", "joint3", "joint4", "joint5"}) {
      const auto & n = res->solution.joint_state.name;
      const auto it = std::find(n.begin(), n.end(), j);
      if (it == n.end()) { done(false, {}); return; }
      q.push_back(res->solution.joint_state.position[it - n.begin()]);
    }
    done(true, q);
  });
}

void Coordinator::rl_pick(const geometry_msgs::msg::PoseStamped & grasp, std::function<void(bool)> done) {
  const auto * o = find(pick_target_);
  if (!o) { done(false); return; }
  // 손끝 = 퍽 바닥 + GRASP_Z (학습 pick_env "down"), 위 = 그보다 rl_above_m — 방향은 grasp_scorer 후보 그대로
  auto down = grasp;
  down.pose.position.z = snap_support(support_top_z(*o, objects_->objects, 0.06), support_levels_, support_snap_m_) + rl_grasp_z_;
  auto above = down;
  above.pose.position.z += rl_above_m_;
  rl_segment("pick", above, down, static_cast<int>(pick_target_), detail(*o, "shape") == "box", {}, rl_open_m_,
             0.0, done);
}

void Coordinator::rl_place(const geometry_msgs::msg::PoseStamped & target, std::function<void(bool)> done) {
  // 놓을 면(target z = 상자 윗면) + GRASP_Z — 든 퍽 바닥이 면에 닿는 손끝 높이 (학습 place_m = 0). 손 방향 = 집을 때 풀린 방향
  auto down = target;
  down.pose.position.z = target.pose.position.z + rl_grasp_z_;
  down.pose.orientation = last_q_;
  auto above = down;
  above.pose.position.z += rl_above_m_;
  rl_segment("place", above, down, -1, false,
             {target.pose.position.x, target.pose.position.y, target.pose.position.z}, -1.0, rl_release_m_, done);
}

void Coordinator::rl_segment(const std::string & kind, geometry_msgs::msg::PoseStamped above, geometry_msgs::msg::PoseStamped down,
                             int target_id, bool is_box, std::vector<double> cell, double before_m, double act_m,
                             std::function<void(bool)> done) {
  if (!ik_->service_is_ready() || !jtc_->action_server_is_ready()) {
    RCLCPP_WARN(get_logger(), "RL %s: /compute_ik · arm_controller 없음", kind.c_str());
    done(false);
    return;
  }
  arm_contact_ = false;
  if (reset_sag_->service_is_ready())  // 스킬마다 처짐 적분을 0에서 (학습 에피소드처럼)
    reset_sag_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  const uint64_t gen = gen_;
  auto transit = [this, kind, above, down, target_id, is_box, cell, act_m, done, gen]() mutable {
    if (gen != gen_) return;
    move_arm(above, [this, kind, above, down, target_id, is_box, cell, act_m, done, gen](bool ok) mutable {  // 퍽 위까지 (MoveIt)
      if (!ok) { done(false); return; }
      above.pose.orientation = down.pose.orientation = last_q_;  // 풀린 손 방향
      compute_ik(above, arm_now(), [this, kind, down, target_id, is_box, cell, act_m, done, gen](bool ok1, std::vector<double> qa) {
        if (!ok1) { RCLCPP_WARN(get_logger(), "RL %s: 위 IK 실패", kind.c_str()); done(false); return; }
        compute_ik(down, qa, [this, kind, qa, target_id, is_box, cell, act_m, done, gen](bool ok2, std::vector<double> qd) {
          if (!ok2) { RCLCPP_WARN(get_logger(), "RL %s: 아래 IK 실패", kind.c_str()); done(false); return; }
          double jump = 0.0;
          for (size_t i = 0; i < qa.size(); ++i) jump = std::max(jump, std::abs(qd[i] - qa[i]));
          if (jump > 1.2) { RCLCPP_WARN(get_logger(), "RL %s: 위·아래 IK가 %.2f rad 떨어짐 (손목 뒤집힘)", kind.c_str(), jump); done(false); return; }
          RlTimes times;
          if (kind == "place") times.act = 0.4;  // 학습 release_s
          const auto q0 = arm_now();
          const RlTimeline tl = rl_timeline(q0, qa, qd, times);
          FollowJointTrajectory::Goal goal;
          goal.trajectory.joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5"};
          const double dt = 0.02, end = tl.t_ph.back();
          for (double t = dt; t < end + dt / 2; t += dt) {  // 50 Hz로 촘촘히 — JTC 보간이 smooth step을 그대로 따라가게
            trajectory_msgs::msg::JointTrajectoryPoint pt;
            pt.positions = rl_ref(std::min(t, end), q0, qa, qd, tl);
            pt.time_from_start = rclcpp::Duration::from_seconds(std::min(t, end));
            goal.trajectory.points.push_back(pt);
          }
          rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions opt;
          opt.goal_response_callback = [this, kind, tl, qa, qd, target_id, is_box, cell, act_m, gen](
              const rclcpp_action::ClientGoalHandle<FollowJointTrajectory>::SharedPtr & h) {
            if (!h || gen != gen_) return;
            std_msgs::msg::String m;  // 정책 노드에 구간 시작 알림 (t0 = 지금 — JTC는 받자마자 실행)
            double psi = 0.0;
            top_down_yaw(last_q_, psi);
            m.data = rl_context_json(kind, now().seconds(), tl, qa, qd, target_id, is_box, psi, cell);
            rl_ctx_pub_->publish(m);
            rl_timer_ = create_wall_timer(std::chrono::duration<double>(tl.t_ph[2]), [this, act_m, gen]() {  // 단계 3: 닫기·놓기
              rl_timer_->cancel();
              if (gen != gen_) return;
              set_gripper(act_m, [](bool) {});  // 결과는 기다리지 않음 — 시간표대로 올라감
            });
          };
          opt.result_callback = [this, kind, done, gen](const rclcpp_action::ClientGoalHandle<FollowJointTrajectory>::WrappedResult & r) {
            if (gen != gen_) return;
            rl_context_stop();
            const bool ok3 = r.code == rclcpp_action::ResultCode::SUCCEEDED && r.result->error_code == 0;
            if (!ok3) RCLCPP_WARN(get_logger(), "RL %s: 궤적 실패 (%d)", kind.c_str(), r.result ? r.result->error_code : -1);
            done(ok3);
          };
          RCLCPP_INFO(get_logger(), "RL %s: 구간 %.2f s (이동 %.2f · 내려감 %.2f)", kind.c_str(), tl.t_ph.back(), tl.t_tr, tl.t_mv);
          jtc_->async_send_goal(goal, opt);
        });
      });
    });
  };
  if (before_m >= 0) set_gripper(before_m, [transit](bool) mutable { transit(); });  // pick: 먼저 벌림
  else transit();
}

void Coordinator::move_cartesian(const std::vector<geometry_msgs::msg::Pose> & waypoints, double vel_scale,
                                 std::function<void(bool)> done) {
  auto req = std::make_shared<GetCartesianPath::Request>();
  req->header.frame_id = world_frame();
  req->header.stamp = now();
  req->start_state.is_diff = true;
  req->group_name = "arm";
  req->link_name = "end_effector_link";
  req->waypoints = waypoints;
  req->max_step = 0.005;            // [m] 직선 보간 간격
  req->jump_threshold = 0.0;        // 관절 튐 검사는 끔 (점이 촘촘하고 손 방향이 같음)
  req->avoid_collisions = true;
  req->max_velocity_scaling_factor = vel_scale;
  req->max_acceleration_scaling_factor = vel_scale;
  const uint64_t gen0 = gen_;
  cartesian_->async_send_request(req, [this, done, gen0](rclcpp::Client<GetCartesianPath>::SharedFuture f) {
    if (gen0 != gen_) return;  // 그새 취소됨
    const auto res = f.get();
    if (res->error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS || res->fraction < 0.99) {
      RCLCPP_WARN(get_logger(), "직교 경로 실패 (code %d, %.0f%%)", res->error_code.val, 100.0 * res->fraction);
      done(false);
      return;
    }
    ExecuteTrajectory::Goal goal;
    goal.trajectory = res->solution;
    const uint64_t gen = gen_;
    rclcpp_action::Client<ExecuteTrajectory>::SendGoalOptions opt;
    opt.result_callback = [this, done, gen](const rclcpp_action::ClientGoalHandle<ExecuteTrajectory>::WrappedResult & r) {
      if (gen != gen_) return;
      done(r.code == rclcpp_action::ResultCode::SUCCEEDED &&
        r.result->error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
    };
    exec_->async_send_goal(goal, opt);
  });
}

void Coordinator::try_grasp(const ScoreGrasps::Response::SharedPtr & cands, size_t i) {
  if (i >= cands->grasps.size() || static_cast<int>(i) >= max_grasp_tries_) {
    fail(arm_contact_ ? "arm_contact" : "grasp_failed_all");
    return;
  }
  RCLCPP_INFO(get_logger(), "파지 후보 %zu (score %.2f)", i, cands->scores[i]);
  auto after = [this, cands, i](bool ok) { ok ? next() : try_grasp(cands, i + 1); };
  if (rl_policy_) rl_pick(cands->grasps[i], after);  // 학습 시간표 + 잔차 정책
  else execute_pick(cands->grasps[i], after);
}

namespace {
constexpr double GRIPPER_OPEN_M = 0.0265;   // URDF 한계
constexpr double GRIPPER_CLOSED_M = 0.0;
}  // namespace

geometry_msgs::msg::PoseStamped Coordinator::tray_pose(size_t slot, double z) const {
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = "base_footprint";  // 적재함은 로봇에 붙어 있음 — 로봇 기준
  p.pose.position.x = tray_slots_[slot].first;
  p.pose.position.y = tray_slots_[slot].second;
  p.pose.position.z = z;
  p.pose.orientation.w = 1.0;  // 방향은 move_arm이 위→아래로 정함
  return p;
}

void Coordinator::execute_pick(const geometry_msgs::msg::PoseStamped & grasp, std::function<void(bool)> done,
                               double approach) {
  arm_contact_ = false;
  auto pre = grasp;
  pre.pose.position.z += approach;
  move_arm(pre, [this, grasp_in = grasp, done, approach](bool ok) {  // 접근
    if (!ok) { done(false); return; }
    auto grasp = grasp_in;
    double psi;
    if (top_down_yaw(grasp.pose.orientation, psi)) grasp.pose.orientation = last_q_;  // 접근에서 풀린 방향(ψ 또는 ψ+π) 그대로
    set_gripper(GRIPPER_OPEN_M, [this, grasp, done, approach](bool ok2) {  // 벌리기
      if (!ok2) { done(false); return; }
      move_arm(grasp, [this, grasp, done, approach](bool ok3) {  // 파지 위치
        if (!ok3) { done(false); return; }
        set_gripper(GRIPPER_CLOSED_M, [this, grasp, done, approach](bool ok4) {  // 닫기
          if (!ok4) { done(false); return; }
          auto lift = grasp;
          lift.pose.position.z += approach;
          move_arm(lift, done);  // 들어올리기
        });
      });
    });
  });
}

void Coordinator::execute_place(const geometry_msgs::msg::PoseStamped & target, std::function<void(bool)> done,
                                double approach) {
  arm_contact_ = false;
  auto above = target;
  above.pose.position.z += approach;
  move_arm(above, [this, done](bool ok) {  // 위로 이동
    if (!ok) { done(false); return; }
    set_gripper(GRIPPER_OPEN_M, done);  // 벌리기
  });
}

void Coordinator::move_arm(const geometry_msgs::msg::PoseStamped & target_in, std::function<void(bool)> done, int tries_left) {
  auto target = target_in;
  fix_frame(target.header);  // 지도 없이 돌 때 map → odom (MoveIt이 없는 프레임을 못 풀어 목표를 못 만듦)
  if (tries_left < 0) tries_left = arm_plan_tries_ - 1;
  auto req = std::make_shared<GetMotionPlan::Request>();
  req->motion_plan_request.group_name = "arm";
  req->motion_plan_request.start_state.is_diff = true;
  req->motion_plan_request.num_planning_attempts = 3;
  req->motion_plan_request.allowed_planning_time = 5.0;
  req->motion_plan_request.max_velocity_scaling_factor = 0.3;
  req->motion_plan_request.max_acceleration_scaling_factor = 0.3;
  moveit_msgs::msg::Constraints goal;
  moveit_msgs::msg::PositionConstraint pc;
  pc.header = target.header;
  // 목표 = 손가락 끝(end_effector_link, gripper_link에서 0.109m) — 퍽 위치에 손목을 대면 손가락이 바닥을 뚫는다
  pc.link_name = "end_effector_link";
  shape_msgs::msg::SolidPrimitive box;
  box.type = box.BOX;
  box.dimensions = {0.02, 0.02, 0.02};
  pc.constraint_region.primitives.push_back(box);
  pc.constraint_region.primitive_poses.push_back(target.pose);
  pc.weight = 1.0;
  moveit_msgs::msg::OrientationConstraint oc;
  oc.header = target.header;
  oc.link_name = "end_effector_link";
  // 방향 = 항상 위에서 아래로 (손가락이 바닥을 봄: 손 z축 → 지면 아래, x축 180° 회전). 물체 자체의 방향(perception yaw · 항등)은
  // 손 자세가 아니라 쓰지 않는다 — 항등이면 손가락이 하늘을 봐서 바닥 퍽에 닿을 수 없다.
  // 목표가 위→아래 손 자세 Rz(ψ)·Rx(π)면(grasp_scorer 후보 · stand) 수직축도 arm_yaw_tolerance 안으로, 아니면 풀어 둔다 (z = π)
  double psi = 0.0;
  const bool yawed = top_down_yaw(target.pose.orientation, psi);
  oc.orientation = yawed ? target.pose.orientation : top_down_q(0.0);
  oc.absolute_x_axis_tolerance = 0.15;
  oc.absolute_y_axis_tolerance = 0.15;
  oc.absolute_z_axis_tolerance = yawed ? arm_yaw_tol_ : M_PI;
  oc.weight = 1.0;
  goal.position_constraints.push_back(pc);
  goal.orientation_constraints.push_back(oc);
  req->motion_plan_request.goal_constraints.push_back(goal);
  const uint64_t gen0 = gen_;
  motion_plan_->async_send_request(req, [this, done, target, tries_left, gen0](rclcpp::Client<GetMotionPlan>::SharedFuture f) {
    if (gen0 != gen_) return;  // 그새 취소됨
    const auto res = f.get();
    if (res->motion_plan_response.error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      RCLCPP_WARN(get_logger(), "플랜 실패 (%d) — 남은 재시도 %d", res->motion_plan_response.error_code.val, tries_left);
      // IK·OMPL이 무작위라 닿는 곳도 가끔 못 찾음 (적재함 뒷줄 실측 9/12) → 같은 목표로 다시
      // 손 방향이 정해진 목표는 ψ와 ψ+π를 번갈아 (손이 대칭 — 같은 파지가 손목 반대쪽으로 닿을 수 있음)
      auto again = target;
      double psi2;
      if (top_down_yaw(target.pose.orientation, psi2)) again.pose.orientation = top_down_q(psi2 + M_PI);
      if (tries_left > 0) move_arm(again, done, tries_left - 1);
      else done(false);
      return;
    }
    double psi3;
    if (top_down_yaw(target.pose.orientation, psi3)) last_q_ = target.pose.orientation;  // 풀린 방향 — 같은 동작의 다음 목표에
    ExecuteTrajectory::Goal goal;
    goal.trajectory = res->motion_plan_response.trajectory;
    const uint64_t gen = gen_;
    rclcpp_action::Client<ExecuteTrajectory>::SendGoalOptions opt;
    opt.result_callback = [this, done, gen](const rclcpp_action::ClientGoalHandle<ExecuteTrajectory>::WrappedResult & r) {
      if (gen != gen_) return;  // 취소된 옛 goal의 늦은 결과
      done(r.code == rclcpp_action::ResultCode::SUCCEEDED &&
        r.result->error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
    };
    exec_->async_send_goal(goal, opt);
  });
}

void Coordinator::set_gripper(double width_m, std::function<void(bool)> done, double effort_a) {
  ParallelGripperCommand::Goal goal;
  goal.command.position = {width_m};
  // effort 비움 = gripper_controller max_effort (controllers.yaml, A) — 보통 쥐는 힘은 거기서 한 곳만 정한다.
  // 세우기의 살짝 잡기만 그 동작에 한해 effort를 준다 (다음 goal은 다시 비움 → max_effort)
  if (effort_a > 0) goal.command.effort = {effort_a};
  const uint64_t gen = gen_;
  rclcpp_action::Client<ParallelGripperCommand>::SendGoalOptions opt;
  opt.result_callback = [this, done, width_m, gen](const rclcpp_action::ClientGoalHandle<ParallelGripperCommand>::WrappedResult & r) {
    if (gen != gen_) return;  // 취소된 옛 goal의 늦은 결과
    const bool ok = r.code == rclcpp_action::ResultCode::SUCCEEDED;
    if (!ok) RCLCPP_WARN(get_logger(), "그리퍼 %.3fm 실패", width_m);
    done(ok);
  };
  gripper_->async_send_goal(goal, opt);
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
