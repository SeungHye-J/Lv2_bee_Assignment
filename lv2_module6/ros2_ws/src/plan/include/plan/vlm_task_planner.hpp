#pragma once

// vlm_task_planner (🛠 직접 작성, 담당: 권형중) — 설계: documents/project/ros/03-plan.md
//   입력: /cmd/user_command (명령), /mission_qr (장소 바코드 작업 지시 — perception), /tracked_objects (지금 보이는 물체),
//         /task_status (FAILED면 재계획), /mission_signal (신호등 암호 미션 — signal_decoder)
//   출력: /sub_goals (plan/SubGoalArray, reliable + transient_local) → coordinator
// VLM은 "무엇을, 어떤 순서로"만 정한다. 어떻게 움직일지·파지 후보 재시도는 coordinator 몫.
// VLM 경로: vlm_endpoint가 비어 있지 않으면 호스트 ai-service에 POST /plan (계약: lv2_module6/ai-service/README)
//   → SubGoalArray 발행. 실패·타임아웃·검증 실패면 템플릿으로 폴백.
//   컨테이너는 network_mode: host라 127.0.0.1이 호스트다. endpoint가 비면(기본·CI) 템플릿만.
// 재계획은 명령 하나당 max_replans(기본 1)회 — coordinator의 "같은 단계 2회 연속 실패면 정지"와 짝.
// 신호등 암호 미션 (documents/mission/README.md)은 VLM 없이 규칙으로 — /mission_signal {색, 개수, 번호 합, 목적지 태그}가 오면
//   번호(퍽 윗면 태그)를 아는 그 색 퍽 중 합이 맞는 조합(로봇에서 가까운 것 우선)을 골라 적재함에 싣고 목적지 장소(location, tag)로 →
//   조합이 없으면 번호를 모르는 그 색(또는 색 모름) 퍽을 inspect하는 계획 → 끝나면 다시 판단. 목적지 태그를 아직 못 봤으면 기다림.
#include <atomic>
#include <cstdint>
#include <map>
#include <mutex>
#include <set>
#include <string>
#include <vector>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "perception/msg/tracked_object_array.hpp"
#include "plan/msg/sub_goal_array.hpp"
#include "plan/msg/mission_signal.hpp"
#include "plan/msg/task_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace plan {

using perception::msg::TrackedObjectArray;
using plan::msg::SubGoal;
using plan::msg::SubGoalArray;
using plan::msg::TaskStatus;

// 재계획 문맥 — 그대로 ai-service replan{plan_id, failed_step, reason}에 실린다
struct ReplanInfo {
  uint32_t plan_id = 0;
  uint32_t step = 0;
  std::string reason;
  bool active = false;
};

// 카메라 최신 프레임 1장 (ai-service images[]용 — pole_front_cam·gripper_cam)
struct ImageFrame {
  std::string camera;
  sensor_msgs::msg::Image::ConstSharedPtr msg;
};

// 옮기기 — 퍽마다 navigate·pick·place tray → navigate dest → (pick tray · place dest)×개수 (적재함으로 실어 나름)
std::vector<SubGoal> carry_goals(const std::vector<std::string> & pucks, const std::string & dest);

// 번호 합 조합 — tags[i] 중 count개의 합이 sum인 것 · cost[i] 합이 가장 작은 것의 인덱스 (없으면 빈 목록)
std::vector<size_t> pick_combo(const std::vector<int> & tags, const std::vector<double> & cost, int count, int sum);

// 물체의 details 값 (없으면 "")
std::string detail(const perception::msg::TrackedObject & o, const std::string & key);

// 신호등 암호 미션 한 번 판단 — kind: carry(옮기기) · inspect(번호 보기) · wait(목적지 태그 대기) · fail(조합 없음)
struct SignalPlan {
  std::string kind;
  std::vector<SubGoal> goals;
  std::vector<uint32_t> inspect_ids;
  std::string why;
};
SignalPlan signal_plan(const TrackedObjectArray & objs, const plan::msg::MissionSignal & sig, double rx, double ry,
  const std::set<uint32_t> & inspected, int capacity);

// 계획 요청에 실을 로봇 위치 (map) — ok=false면 뺌
struct RobotPose {
  bool ok = false;
  double x = 0, y = 0, yaw = 0;
};

class VlmTaskPlanner : public rclcpp::Node {
public:
  VlmTaskPlanner();

private:
  void on_status(const TaskStatus & s);

  static std::string find_in(const TrackedObjectArray & objs, const std::string & cls);

  void on_command(const char * source, const std::string & text);  // /cmd/user_command · /mission_qr 공통
  void make_plan();                            // 물체 확인 후 VLM 시도 (endpoint 없으면 템플릿 즉시)
  void make_template(const TrackedObjectArray & objs, const std::string & cmd);  // 고정 4단계 발행
  void make_signal_plan();                     // 신호등 암호 미션 — 규칙으로 (VLM 안 씀)
  void publish_plan(std::vector<SubGoal> goals, const char * how);
  void try_vlm_async();                        // 스냅샷 → 별 스레드 POST → 성공 발행·실패 템플릿

  static SubGoal goal(uint32_t step, const std::string & skill, const std::string & target,
    const std::string & relation, float offset);

  // ai-service 계약 (POST /plan → {"ok","goals"}) — 의존성 없이 소켓+최소 파서
  //   objects[]: /tracked_objects 그대로의 뜻 — id · class_name · position(map) · size · confidence · color ·
  //              camera · px {카메라: [u, v]} (ai-service images.py가 그 카메라 영상에 번호를 찍는 형식)
  //   robot: 로봇 위치 {x, y, yaw} (map, TF map→base_footprint) — VLM이 물체까지 거리를 보고 navigate를 넣게. TF 없으면 뺌
  static std::string plan_request(
    const std::string & cmd, const TrackedObjectArray & objs, const ReplanInfo & rp,
    const std::vector<ImageFrame> & imgs, const RobotPose & robot = {});
  RobotPose robot_pose() const;
  static bool fetch_goals(const std::string & endpoint, const std::string & body,
    std::vector<SubGoal> & goals, double timeout_s, std::string * err = nullptr);
  // 320px JPEG base64 (ai-service images[]용) — OpenCV 의존
  static std::string encode_jpeg_base64(const sensor_msgs::msg::Image & msg);

  int tray_capacity_;  // 적재함 용량 (coordinator tray_slots 개수와 같게)
  int max_replans_, replans_ = 0;  // 메인 스레드만 (구독 콜백·진단)
  std::string vlm_endpoint_;       // 비면 VLM 안 씀 (기본·CI) — 예: http://127.0.0.1:8000
  double vlm_timeout_s_;
  std::atomic<bool> pending_{false};   // 계획할 명령이 남아 있음
  std::atomic<uint32_t> plan_id_{0};
  std::atomic<uint64_t> epoch_{0};     // make_plan마다 증가 — 낡은 워커 응답 버림
  std::atomic<bool> vlm_busy_{false};  // VLM 워커 비행 중
  std::string cmd_;
  ReplanInfo replan_;
  TrackedObjectArray::ConstSharedPtr objects_;
  std::string last_objs_sig_;  // 마지막으로 계획 시도한 물체 집합 — 같으면 재시도 안 함 (VLM 폭주 방지)
  std::map<std::string, sensor_msgs::msg::Image::ConstSharedPtr> images_;  // 카메라별 최신 1장
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::mutex mutex_;  // cmd_·replan_·objects_·images_·발행 결정 (구독 콜백 ↔ 워커)
  rclcpp::Publisher<SubGoalArray>::SharedPtr goals_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr qr_sub_;
  rclcpp::Subscription<TrackedObjectArray>::SharedPtr objects_sub_;
  rclcpp::Subscription<TaskStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<plan::msg::MissionSignal>::SharedPtr signal_sub_;
  plan::msg::MissionSignal signal_;
  bool signal_active_ = false;          // 신호등 미션 진행 중 (옮기기 끝나거나 조합 없음이면 끝)
  std::set<uint32_t> inspected_;        // 이번 미션에서 inspect한 퍽 (같은 퍽을 다시 보지 않게)
  std::string plan_kind_;               // 마지막으로 낸 계획 종류 (inspect면 끝난 뒤 이어서 판단)
  std::string signal_note_ = "-";       // 진단
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> image_subs_;
  diagnostic_updater::Updater diag_{this};
};

}  // namespace plan
