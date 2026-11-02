#pragma once

// coordinator (🛠 직접 작성, 담당: 권형중) — 설계: documents/project/ros/03-plan.md "coordinator가 지키는 실행 규칙"
//   입력: /task/sub_goals (VLM 계획), /tracked_objects (대상 id 검증), /arm_safety_event (팔 접촉)
//   출력: 🎬 /navigate_to_pose (Nav2), 🎬 /plan_kinematic_path + /execute_trajectory (move_group),
//         🎬 /gripper_controller/gripper_cmd, 📞 /score_grasps (grasp_scorer),
//         📞 /clear_octomap · /apply_planning_scene (move_group), ✉️ /task/status → VLM·UI, ✉️ /robot/say → user_input
// 규칙 (골격에 들어간 것):
//   skill 정의: documents/project/ros/05-arm.md (자세 · skill · 흐름 · 실패 이유)
//   1. 계획 검증 — 허용 동작(navigate·pick·place·inspect·say·pose·load·unload), 숫자 대상 id가 /tracked_objects에 있는지
//      (pose·load·unload의 target은 자세 이름·칸 번호라 물체 검증 안 함)
//   inspect <퍽 id> (신호등 암호 미션 — 퍽 윗면 16mm AprilTag는 gripper_cam이 위에서 봐야 읽힘): 멀면 다가가고 손끝을 퍽 윗면 위
//     inspect_height_m로 (손가락 아래) → inspect_wait_s 기다림 → 다음 단계. 읽었는지(details tag)는 planner가 /tracked_objects로 본다
//   4. 파지 재시도는 coordinator 몫 — grasp_scorer 후보를 점수순으로 최대 max_grasp_tries(2)개, 다 실패하면 FAILED
//   5. 같은 단계 2회 연속 실패면 /robot/say로 알리고 정지
//   자세: pose stow · carry_left · carry_right — 관절 목표로 MoveIt (값: config/basket.yaml, isaac-lab slot_table)
//   적재함 12칸 (basket_slot_0~11): load <칸 | next> · unload <칸 | last> — 칸·퍽 모양(box·cyl)별 관절값(basket.yaml)을 MoveIt 관절 목표로
//     load = carry_<칸 방향> → q_over → q_low → 벌림 → q_lift → stow · unload = carry → 벌림 → q_over → q_grasp → 닫기 → 잡기 판정 → q_over → carry
//     적재함 장부는 coordinator 하나만 씀: 칸마다 {모양 · 색 · tag · 확인 필요} + 실은 순서 + 손에 든 퍽 → 📢 /coordinator/basket (latched)
//       pick 때 그 퍽의 tag·색·모양을 손에 기억 → load 성공이면 칸에 적음 → unload 성공이면 손으로 옮김. 파일(basket_file)에 저장해 다시 떠도 이어 감
//       📞 /coordinator/basket_clear — 사람이 적재함을 비웠을 때. unload가 빈손이면 칸을 비우지 않고 check(확인 필요)로 표시
//     load next는 basket_use 칸 중 빈 칸 (팔이 잘 못 닿는 칸은 설정에서 뺌). 옛 target "tray"는 호환 이름: place tray = load next · pick tray = unload last
//   처짐 보정: 팔 skill(pick·place·load·unload) 시작 때 한 번 📞 /rl_arm_correction/reset_sag (없으면 WARN 한 번, 계속)
//   잡기 판정: 닫은 뒤 gripper_left_joint 위치(퍽이 끼어 덜 닫힘) + 전류(effort) — grasp_check (pick·unload)
//   쥐는 힘: 닫을 때 grip_current_a(0.35 A)를 액션 goal effort로 보냄 (비우면 0.6 A — 놓을 때 전류가 튐)
//   place <location_box>: 상자 윗면(size.z) 위에 손 안 퍽 바닥(held_bottom_m) + place_release_m에서 벌림, 여러 개면 상자 안·상자 축으로 비켜 놓음
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
#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "control/msg/arm_safety_event.hpp"
#include "control_msgs/action/parallel_gripper_command.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "ml/srv/score_grasps.hpp"
#include "moveit_msgs/action/execute_trajectory.hpp"
#include "moveit_msgs/srv/apply_planning_scene.hpp"
#include "moveit_msgs/srv/get_motion_plan.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "perception/msg/tracked_object_array.hpp"
#include "plan/msg/sub_goal_array.hpp"
#include "plan/msg/task_status.hpp"
#include "plan/msg/basket_state.hpp"
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

// 적재함 칸 고르기 — filled[i] = 실린 퍽 모양("" = 빔) · order = 실은 순서.
//   load: "next"·"tray" = 빈 칸 중 가장 낮은 번호 · 숫자 = 그 칸 (차 있으면 slot_full)
//   unload: "last"·"tray" = 마지막에 실은 칸 · 숫자 = 그 칸 (비었으면 slot_empty). 못 고르면 −1 + why(slot_invalid · slot_full · slot_empty)
//   usable[i] = false인 칸은 next가 고르지 않음 (비면 전부 씀 · 칸 번호를 직접 주면 그대로)
int basket_slot(const std::string & target, const std::vector<std::string> & filled, const std::vector<int> & order,
                bool load, std::string & why, const std::vector<bool> & usable = {});
// 상자 위 n번째(0부터)로 놓을 자리의 (dx, dy) [m, map] — spread_offset을 상자 안(반 변 − margin)으로 자르고 상자 yaw로 돌림
std::pair<double, double> box_spread(int n, double spacing, double half_x, double half_y, double margin, double yaw);
// 잡기 판정 — 위치(손가락이 퍽에 걸려 덜 닫힘) ≥ min_pos · 전류 |effort| ≥ min_effort (effort가 NaN이면 위치만)
bool grasp_held(double pos, double effort, double min_pos, double min_effort);
// 물체를 바라보며 standoff 떨어진 로봇 자세 (map) — 로봇이 있는 쪽에서. 로봇이 물체와 거의 같은 자리면 물체의 −x 쪽
geometry_msgs::msg::Pose approach_pose(double ox, double oy, double rx, double ry, double standoff);
// 같은 곳에 n번째(0부터)로 내려놓을 때 중심에서 비킬 (dx, dy) — 가운데 · 좌 · 우 · 앞 · 뒤 · 대각선 순, spacing 간격
std::pair<double, double> spread_offset(int n, double spacing);

using control::msg::ArmSafetyEvent;
using ExecuteTrajectory = moveit_msgs::action::ExecuteTrajectory;
using GetMotionPlan = moveit_msgs::srv::GetMotionPlan;
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
  // 로봇 위치 (map → base_footprint TF). 없으면 false
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

  // ---------- pose: 이름 있는 관절 자세로 (stow · carry_left · carry_right)
  void pose(const SubGoal & g);

  // ---------- 적재함 load · unload (관절 자세 순서 — MoveIt 관절 목표)
  void load(const SubGoal & g);
  void unload(const SubGoal & g);
  std::vector<double> slot_q(const std::string & shape, const std::string & key, int slot) const;
  std::map<std::string, double> arm_target(const std::vector<double> & q) const;
  using Step = std::function<void(std::function<void(bool)>)>;
  // 단계를 차례로 — 하나라도 실패하면 done(false)
  void run_steps(std::shared_ptr<std::vector<Step>> steps, size_t i, std::function<void(bool)> done);
  Step joints_step(const std::vector<double> & q);
  Step gripper_step(double width_m, double effort_a = -1.0);
  Step held_step();  // 잡기 판정 (grasp_check 끄면 통과)

  // ---------- 처짐 보정 초기화 — 팔 skill 시작 때 한 번 (서비스 없으면 WARN 한 번)
  void reset_sag();
  std::string held_shape_of(uint32_t id) const;  // 퍽 details shape (모르면 box)
  // ---------- 적재함 장부 — 발행 · 파일 저장/읽기
  void basket_changed();
  void basket_save() const;
  void basket_load();
  std::vector<std::string> basket_shapes() const;

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
  // task=false(스캔 자세): 작업 중이면 안 감 · task=true(skill): 작업 취소(gen_)되면 결과 무시
  void move_joints(const std::map<std::string, double> & target, std::function<void(bool)> done, bool task = false);

  // ---------- 🎬 /gripper_controller/gripper_cmd (m)
  // effort_a [A] = 쥐는 힘 한계 (액션 goal effort → gripper_controller max_effort_interface → 펌웨어 C 9번째 칸 Goal Current,
  //   펌웨어가 0.00269~1.2 A로 자름). 음수 = 비움 → controllers.yaml max_effort(0.6)
  void set_gripper(double width_m, std::function<void(bool)> done, double effort_a = -1.0);

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
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr map_mode_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joints_sub_;
  rclcpp::TimerBase::SharedPtr scan_timer_;
  size_t approached_ = static_cast<size_t>(-1);  // 다가가기를 이미 한 단계 (idx_) — 단계마다 한 번만
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  double place_spacing_, held_bottom_m_, place_release_m_, box_margin_m_;
  std::map<std::string, std::vector<double>> poses_;      // stow · carry_left · carry_right
  std::vector<std::string> basket_carry_;                 // 칸마다 left | right
  std::map<std::string, std::vector<double>> basket_q_;   // "box/q_over" … → 칸 0~11 × 관절 5
  struct PuckInfo { std::string shape, color; int tag = -1; bool check = false; };  // shape "" = 빔
  std::vector<PuckInfo> basket_;                          // 칸마다 (장부 — 이 노드만 씀)
  std::vector<int> basket_order_;                         // 실은 순서 (unload last)
  std::vector<bool> basket_use_;                          // load next가 쓰는 칸
  PuckInfo held_;                                         // 손에 든 퍽 (load가 모양으로 표를 고름)
  std::string basket_file_;
  rclcpp::Publisher<plan::msg::BasketState>::SharedPtr basket_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr basket_clear_srv_;
  bool grasp_check_ = true;
  double grip_current_a_ = 0.35;  // 닫을 때 쥐는 힘 [A] — 팀 결정 0.3~0.4 A (22~29 N)
  double grasp_min_pos_m_, grasp_min_effort_;
  std::map<std::string, double> joint_effort_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_sag_;
  bool sag_warned_ = false;
  bool grasp_empty_ = false;      // 마지막 잡기 판정이 빈손 — 실패 이유 grasp_empty
  uint32_t picking_id_ = 0;       // 지금 집는 퍽 id (모양 기억용)
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
  rclcpp_action::Client<ExecuteTrajectory>::SharedPtr exec_;
  rclcpp_action::Client<ParallelGripperCommand>::SharedPtr gripper_;
  rclcpp::Client<ScoreGrasps>::SharedPtr score_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr clear_octomap_;
  rclcpp::Client<ApplyPlanningScene>::SharedPtr apply_scene_;
  diagnostic_updater::Updater diag_{this};
};

}  // namespace plan
