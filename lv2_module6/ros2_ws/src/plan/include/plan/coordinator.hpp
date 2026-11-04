#pragma once

// coordinator (🛠 직접 작성, 담당: 권형중) — 설계: documents/project/ros/03-plan.md "coordinator가 지키는 실행 규칙"
//   입력: /task/sub_goals (VLM 계획), /tracked_objects (대상 id 검증), /arm_safety_event (팔 접촉)
//   출력: 🎬 /navigate_to_pose (Nav2), 🎬 /plan_kinematic_path + /execute_trajectory (move_group),
//         🎬 /gripper_controller/gripper_cmd, 📞 /score_grasps (grasp_scorer),
//         📞 /clear_octomap · /apply_planning_scene (move_group), ✉️ /task/status → VLM·UI, ✉️ /robot/say → user_input
// 규칙 (골격에 들어간 것):
//   1. VLM 출력 검증 — 허용 동작(navigate·pick·place·inspect·stand·say), 숫자 대상 id가 /tracked_objects에 있는지
//   stand <누운 퍽 id> (세우기, isaac/lab/right_env.py tilt 모드): 멀면 다가가고 → 로봇에서 먼 쪽 끝 위 → 직선으로 내려가
//     위쪽 절반을 살짝 잡음(stand_pinch_m · stand_pinch_effort — 손가락 사이에서 퍽이 돌 만큼) → 가까운 끝 아래 모서리를 축으로
//     호 → 멈춤 → 90°로 돌아와 내려놓음 → 벌림 → 들어 올림 → stand_verify_s 뒤 /tracked_objects가 서 있으면 성공.
//     퍽 위까지만 MoveIt(OMPL). 그 뒤는 점마다 이 팔 전용 IK(plan/arm_ik — 위치 + 손이 아래 + 손목 각) → 관절 궤적을 JTC로
//     (학습 right_env처럼 관절 직선 보간). MoveIt 직교 경로·IK(pick_ik rotation_scale 0)는 손 방향을 못 지켜 0%로 실패했음 (실기 11/03)
//     경로 식은 plan/stand_geometry.hpp. 누운 퍽은 pick이 거절(pick_lying) — 세운 뒤 pick
//   손 방향: 목표가 위→아래 자세 Rz(ψ)·Rx(π)(grasp_scorer · stand)면 수직축도 arm_yaw_tolerance 안으로 지킴, 못 풀면 ψ+π로 다시
//     (손이 대칭이라 같은 파지). 한 동작(pick·stand) 안에서는 처음 풀린 방향을 끝까지 씀. 그 밖의 목표(적재함 · inspect)는 수직축 자유
//   RL 정책 구간 (rl_policy: true일 때 pick <퍽 id> · place <물체 id>): 퍽 위까지는 MoveIt → IK(/compute_ik)로 위·아래 관절 목표 →
//     학습과 같은 시간표(plan/rl_segment.hpp)의 관절 궤적을 /arm_controller/follow_joint_trajectory로 + 그 구간 정보를
//     /rl_policy/context(JSON)로 → ml/rl_policy_node가 잔차 정책 Δ를 rl_arm_correction에 넣음. 스킬 시작마다 /rl_arm_correction/reset_sag
//     높이: 손끝 = 퍽 바닥(place는 놓을 면) + rl_grasp_z (bee_common GRASP_Z), 위 = 그보다 rl_above_m. 꺼져 있으면 예전 MoveIt 동작
//   inspect <퍽 id> (신호등 암호 미션 — 퍽 윗면 16mm AprilTag는 gripper_cam이 위에서 봐야 읽힘): 멀면 다가가고 손끝을 퍽 윗면 위
//     inspect_height_m로 (손가락 아래) → inspect_wait_s 기다림 → 다음 단계. 읽었는지(details tag)는 planner가 /tracked_objects로 본다
//   4. 파지 재시도는 coordinator 몫 — grasp_scorer 후보를 점수순으로 최대 max_grasp_tries(2)개, 다 실패하면 FAILED
//   5. 같은 단계 2회 연속 실패면 /robot/say로 알리고 정지
//   적재함 (MISSION ③ MOVE_TO_B — 퍽을 실어 운반): target "tray"
//     place tray = 다음 빈 칸에 싣기 · pick tray = 마지막에 실은 것부터 꺼내기 (칸 = tray_slots, base_footprint 기준 — MoveIt 실측으로
//     손끝 아래 방향이 닿는 곳). 몇 개 실렸는지 tray_count로 센다 (노드가 다시 뜨면 0). 꽉 차면 tray_full · 비면 tray_empty로 실패
//   팔이 닿게 다가가기: pick · place <물체 id> 차례에 대상이 로봇(base_footprint, TF)에서 reach_m 넘게 떨어져 있으면
//     먼저 그 물체를 바라보며 approach_m 떨어진 자리로 navigate한 뒤 그 단계를 실행 (단계마다 한 번 — VLM이 navigate를 빼먹어도).
//     navigate <물체 id>도 같은 접근 자세 — 로봇이 있는 쪽에서 물체를 바라봄 (예전: 늘 물체의 −x 쪽 · yaw 0)
//   시작 자세: bringup 직후 한 번 팔을 initial_pose(plan.yaml)로 — MoveIt 없이 /arm_controller/joint_trajectory 토픽
//   스캔 자세: 지도를 만들 때(/state/map_mode MAPPING · 자동 탐사 EXPLORE) 작업이 없으면 팔을 scan_pose로 (2초마다 확인, MoveIt 관절 목표)
//     — gripper_cam이 왼쪽 뒤를 보는 두 번째 고정 카메라가 됨 (ros_system/config/scan_pose.yaml · scan_cam_gate). 수동 모드여도
//     (스캔은 보통 수동 주행) — MAPPING 동안 팔 수동 조작은 manual_control(/cmd/manual/arm)·service_action_input(/move_action)이 거절
//   같은 물체(구역) 위에 여러 번 place하면 겹치지 않게 place_spacing 간격으로 비켜 놓는다 (B 지역 판에 퍽 여러 개)
//   AUTO가 아닌 모드(/state/operation_mode = MANUAL 수동 · EXPLORE 자동 탐사): 진행 중 이동·팔 goal을 취소하고 FAILED(manual_override · explore_override),
//     그동안 새 계획은 받지 않는다 (VLM은 manual_* · explore_* 실패에는 재계획하지 않음). AUTO로 돌아오면 새 명령을 기다린다.
//   상태: /diagnostics (모드 · 계획 · 단계 · 실패 횟수)
// TODO(권형중): 2. 이동 전 팔 stow·퍽을 든 채 footprint 확장, 3. 정지 후 0.5초 대기, map→odom TF 대기,
//               접근 자세 계산, MTC 태스크 구성(접근·열기·파지·attach·들어올리기), 장면에 퍽·장애물 등록
#include <array>
#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "control/msg/arm_safety_event.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "control_msgs/action/parallel_gripper_command.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "ml/srv/score_grasps.hpp"
#include "moveit_msgs/action/execute_trajectory.hpp"
#include "moveit_msgs/srv/apply_planning_scene.hpp"
#include "moveit_msgs/srv/get_cartesian_path.hpp"
#include "moveit_msgs/srv/get_motion_plan.hpp"
#include "moveit_msgs/srv/get_position_ik.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "perception/msg/tracked_object_array.hpp"
#include "plan/msg/sub_goal_array.hpp"
#include "plan/msg/task_status.hpp"
#include "plan/rl_segment.hpp"
#include "plan/stand_geometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace plan {

// 적재함 칸 — tray_slots 파라미터 [x0, y0, x1, y1, …] (base_footprint) → 칸 목록 (홀수 개면 마지막 값 버림)
std::vector<std::pair<double, double>> tray_slots(const std::vector<double> & flat);
// 물체를 바라보며 standoff 떨어진 로봇 자세 (map) — 로봇이 있는 쪽에서. 로봇이 물체와 거의 같은 자리면 물체의 −x 쪽
geometry_msgs::msg::Pose approach_pose(double ox, double oy, double rx, double ry, double standoff);
// 같은 곳에 n번째(0부터)로 내려놓을 때 중심에서 비킬 (dx, dy) — 가운데 · 좌 · 우 · 앞 · 뒤 · 대각선 순, spacing 간격
std::pair<double, double> spread_offset(int n, double spacing);

using control::msg::ArmSafetyEvent;
using ExecuteTrajectory = moveit_msgs::action::ExecuteTrajectory;
using GetMotionPlan = moveit_msgs::srv::GetMotionPlan;
using GetCartesianPath = moveit_msgs::srv::GetCartesianPath;
using GetPositionIK = moveit_msgs::srv::GetPositionIK;
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using ParallelGripperCommand = control_msgs::action::ParallelGripperCommand;
using ml::srv::ScoreGrasps;
using moveit_msgs::srv::ApplyPlanningScene;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using perception::msg::TrackedObjectArray;
using plan::msg::SubGoal;
using plan::msg::SubGoalArray;
using plan::msg::TaskStatus;

class Coordinator : public rclcpp::Node {
public:
  Coordinator();

private:
  // ---------- 계획 받기 · 검증 (규칙 1)
  // ---------- 수동 모드
  void on_mode(const std::string & mode);

  std::string step_text() const;

  void on_plan(const SubGoalArray & plan);

  bool validate(const SubGoalArray & plan, std::string & why) const;

  static bool is_id(const std::string & s);

  const perception::msg::TrackedObject * find(uint32_t id) const;

  // ---------- 단계 실행
  void run_step();

  // Nav2·MoveIt이 아직 안 떴으면 (lifecycle 활성화 중 등) 1초 뒤 같은 단계를 다시. server_wait_s 넘으면 실패.
  bool ready_or_retry(bool ready, const std::string & reason);

  void next();

  // 규칙 5: 같은 단계 max_step_failures회 연속 실패면 알리고 정지. 그 전엔 FAILED만 보내고 VLM 재계획을 기다린다.
  void fail(const std::string & reason);

  static std::string key(const SubGoal & g);

  // ---------- navigate: 🎬 NavigateToPose
  void navigate(const SubGoal & g);
  void send_nav(const geometry_msgs::msg::Pose & pose, std::function<void(bool)> done);
  // 물체·팔 목표 좌표계 — map TF가 있으면 map, 없으면(지도 없이 실행) odom. perception도 그때 odom 좌표를 map으로 적어 냄
  std::string world_frame() const;
  void fix_frame(std_msgs::msg::Header & h) const;  // "map"인데 map TF가 없으면 odom으로
  // 로봇 위치 (world_frame → base_footprint TF). 없으면 false
  bool robot_xy(double & x, double & y) const;
  // pick·place <물체 id> 전에: 대상이 reach_m 밖이면 다가가는 navigate를 시작하고 true (끝나면 같은 단계를 다시) · 아니면 false
  bool approach_if_far(const SubGoal & g);

  // ---------- pick: octomap 지우기 → 장면 등록 → 📞 /score_grasps → 후보 순서대로 move_group (규칙 4)
  void pick(const SubGoal & g);

  void try_grasp(const ScoreGrasps::Response::SharedPtr & cands, size_t i);

  // ---------- 🎬 move_group: 접근 → 벌리기 → 파지 → 닫기 → 들어올리기
  void execute_pick(const geometry_msgs::msg::PoseStamped & grasp, std::function<void(bool)> done, double approach = 0.10);

  // ---------- 🎬 move_group: 위로 이동 → 벌리기
  void execute_place(const geometry_msgs::msg::PoseStamped & target, std::function<void(bool)> done, double approach = 0.10);

  // ---------- inspect: 퍽 위에서 gripper_cam으로 내려다보기 (윗면 태그 읽기)
  void inspect(const SubGoal & g);

  // ---------- stand: 누운 퍽 세우기 (먼 끝 살짝 잡기 → 호 → 내려놓기 → 확인)
  void stand(const SubGoal & g);
  void stand_run(uint32_t id, StandPath path);   // 위로 간 뒤 (손 방향 확정) 나머지
  void stand_abort(const std::string & reason);   // 잡은 채 실패하면 벌리고 FAILED
  void log_hand(uint32_t id, const char * where, double px, double py, double pz);  // 실제 손끝(TF)과 계획의 차
  void stand_verify(uint32_t id);
  // 그리퍼가 퍽을 쥘 때까지 (전류·위치) — done(1 쥠 · 0 빈손 · −1 시간 초과)
  void wait_grip(std::function<void(int)> done);
  // 위→아래 손끝 목표(물체 좌표계) → arm_base_link 위치 + 손 방향 ψ (TF). 손 자세가 위→아래 꼴이 아니거나 TF 없으면 false
  bool to_arm_frame(const geometry_msgs::msg::Pose & p, std::array<double, 3> & out, double & psi) const;
  // 🎬 /arm_controller/follow_joint_trajectory — 관절 경유점들을 한 번의 움직임으로 (rl_segment joint_path)
  void move_joints_path(const std::vector<std::vector<double>> & pts, double vel_scale, std::function<void(bool)> done);

  // ---------- RL 정책 구간 (rl_policy) — kind "pick" · "place"
  void rl_pick(const geometry_msgs::msg::PoseStamped & grasp, std::function<void(bool)> done);
  void rl_place(const geometry_msgs::msg::PoseStamped & target, std::function<void(bool)> done);
  // before_m ≥ 0이면 먼저 그리퍼를 그 폭으로 · act_m = 단계 3(닫기·놓기)에서 그리퍼 목표
  void rl_segment(const std::string & kind, geometry_msgs::msg::PoseStamped above, geometry_msgs::msg::PoseStamped down,
                  int target_id, bool is_box, std::vector<double> cell, double before_m, double act_m, std::function<void(bool)> done);
  void compute_ik(const geometry_msgs::msg::PoseStamped & pose, const std::vector<double> & seed,
                  std::function<void(bool, std::vector<double>)> done);
  std::vector<double> arm_now() const;  // joint1~5 지금 각 (/joint_states)
  void rl_context_stop();

  // ---------- 🎬 /compute_cartesian_path + /execute_trajectory — 손끝(end_effector_link)이 점들을 직선으로 이음
  void move_cartesian(const std::vector<geometry_msgs::msg::Pose> & waypoints, double vel_scale, std::function<void(bool)> done);

  // ---------- 적재함: 칸 위치 (base_footprint) 손끝 목표
  geometry_msgs::msg::PoseStamped tray_pose(size_t slot, double z) const;

  // ---------- 🎬 /plan_kinematic_path + /execute_trajectory
  //     계획이 실패하면 같은 목표로 arm_plan_tries번까지 (tries_left −1 = 처음)
  void move_arm(const geometry_msgs::msg::PoseStamped & target, std::function<void(bool)> done, int tries_left = -1);

  // ---------- 시작 자세 — bringup 직후 한 번 팔을 initial_pose로 (MoveIt 없이 JTC 토픽)
  //   못 가면 initial_pose_retries번만 다시 보내고 지금 자세로 멈춤 (걸린 채 계속 밀면 과부하)
  //   initial_pose_enabled는 실행 중에도 바꿀 수 있음 (false → 바로 멈춤, true → 다시 시도)
  void check_initial_pose();
  bool at_initial_pose() const;
  std::string initial_pose_error() const;  // 목표와 0.05rad 넘게 다른 관절 "joint2 −0.31" …
  void hold_arm();                          // 지금 각도를 목표로 보내 JTC가 더 밀지 않게
  void publish_arm_targets(const std::vector<std::string> & names, const std::vector<double> & pos, double seconds);

  // ---------- 스캔 자세 — 지도 만드는 중이고 작업이 없으면 팔을 scan_pose로
  void check_scan_pose();
  bool at_scan_pose() const;
  // for_task: 작업(stand) 중에도 움직임 (기본은 스캔 자세용 — 작업이 시작되면 안 움직임)
  void move_joints(const std::map<std::string, double> & target, std::function<void(bool)> done, bool for_task = false);

  // ---------- 🎬 /gripper_controller/gripper_cmd (m)
  //     effort_a > 0 = 그 동작만 쥐는 힘 한계 [A] (그리퍼 액션 goal effort → 펌웨어 Goal Current) · 0 = gripper_controller max_effort
  void set_gripper(double width_m, std::function<void(bool)> done, double effort_a = 0.0);

  // ---------- 새 계획·직접 목표가 들어오면 진행 중 goal 전부 취소 (옛 결과가 새 단계를 건너뛰지 않게)
  void cancel_all();

  // ---------- "x,y" (map 좌표) 목표 파싱 — navigate 직접 지정용
  static bool parse_point(const std::string & target, double & x, double & y);

  // ---------- 출력
  void status(uint32_t step, uint8_t state, const std::string & reason);

  void say(const std::string & text);

  int max_grasp_tries_, max_step_failures_, arm_plan_tries_;
  double reach_m_, approach_m_;
  double inspect_height_m_, inspect_wait_s_;
  double arm_yaw_tol_;                         // [rad] 위→아래 목표의 수직축 허용 오차
  geometry_msgs::msg::Quaternion last_q_;      // 마지막으로 풀린 위→아래 손 자세 (ψ 또는 ψ+π) — 같은 동작의 다음 목표에 씀
  StandParams stand_;
  std::vector<double> support_levels_;  // 퍽이 놓이는 면 높이 [m] (바닥 · 상자) — 잰 받침 높이를 맞춤
  double support_snap_m_;
  double stand_open_m_, stand_pinch_m_, stand_pinch_effort_, stand_close_effort_, stand_vel_, stand_hang_s_, stand_verify_s_;
  rclcpp::TimerBase::SharedPtr stand_timer_;
  bool stand_preview_ = false;  // stand의 relation "preview" — 잡을 자리 위에서 멈추기만
  bool rl_policy_ = false;
  double rl_open_m_, rl_release_m_, rl_grasp_z_, rl_above_m_;
  uint32_t pick_target_ = 0;           // 지금 pick의 퍽 id (RL 구간 관측 — 퍽 − 손)
  rclcpp::TimerBase::SharedPtr rl_timer_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr jtc_;
  rclcpp::Client<GetPositionIK>::SharedPtr ik_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_sag_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rl_ctx_pub_;
  rclcpp::TimerBase::SharedPtr inspect_timer_;
  std::vector<std::string> initial_joints_;
  std::vector<double> initial_pose_;
  double initial_tol_ = 0.05;
  double initial_move_s_ = 3.0;
  bool initial_enabled_ = true;
  bool initial_waiting_ = false;
  int initial_retries_ = 1;
  double initial_gripper_ = 0.0;      // [m] 음수 = 안 건드림
  bool initial_gripper_sent_ = false;
  int initial_sent_count_ = 0;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr param_cb_;
  std::chrono::steady_clock::time_point initial_sent_{};
  rclcpp::TimerBase::SharedPtr initial_timer_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr arm_traj_pub_;
  std::vector<std::string> scan_joints_;
  std::vector<double> scan_pose_;
  double scan_tol_ = 0.05;
  bool scan_moving_ = false;
  std::string map_mode_;
  std::map<std::string, double> joints_;
  std::map<std::string, double> efforts_;  // /joint_states effort (그리퍼 = 전류 A)
  rclcpp::TimerBase::SharedPtr grip_timer_;
  double stand_grip_current_, stand_empty_m_, stand_grip_timeout_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr map_mode_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joints_sub_;
  rclcpp::TimerBase::SharedPtr scan_timer_;
  size_t approached_ = static_cast<size_t>(-1);  // 다가가기를 이미 한 단계 (idx_) — 단계마다 한 번만
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::vector<std::pair<double, double>> tray_slots_;
  double tray_release_z_, tray_grasp_z_, tray_approach_m_, place_spacing_;
  int tray_count_ = 0;                 // 적재함에 실린 퍽 수 (칸 0부터 차례로)
  std::map<std::string, int> placed_;  // 계획 안에서 물체(구역) id별로 내려놓은 수 — 겹치지 않게 비킴
  std::string mode_ = "AUTO";
  bool estop_ = false;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  double server_wait_s_, waited_s_ = 0;
  rclcpp::TimerBase::SharedPtr retry_;
  SubGoalArray plan_;
  size_t idx_ = 0;
  bool running_ = false, arm_contact_ = false;
  uint64_t gen_ = 0;  // 새 계획·취소 때마다 증가 — 옛 goal 결과가 새 단계를 건드리지 않게
  std::map<std::string, int> failures_;
  TrackedObjectArray::ConstSharedPtr objects_;
  std::shared_ptr<SubGoalArray> waiting_plan_;
  rclcpp::Publisher<TaskStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr say_pub_;
  rclcpp::Subscription<SubGoalArray>::SharedPtr goals_sub_;
  rclcpp::Subscription<TrackedObjectArray>::SharedPtr objects_sub_;
  rclcpp::Subscription<ArmSafetyEvent>::SharedPtr safety_sub_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_;
  rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr nav_goal_;
  rclcpp::Client<GetMotionPlan>::SharedPtr motion_plan_;
  rclcpp::Client<GetCartesianPath>::SharedPtr cartesian_;
  rclcpp_action::Client<ExecuteTrajectory>::SharedPtr exec_;
  rclcpp_action::Client<ParallelGripperCommand>::SharedPtr gripper_;
  rclcpp::Client<ScoreGrasps>::SharedPtr score_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr clear_octomap_;
  rclcpp::Client<ApplyPlanningScene>::SharedPtr apply_scene_;
  diagnostic_updater::Updater diag_{this};
};

}  // namespace plan
