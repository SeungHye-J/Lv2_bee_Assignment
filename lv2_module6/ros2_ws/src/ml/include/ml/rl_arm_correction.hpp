#pragma once

// rl_arm_correction (담당: 송정혁) — 설계: documents/project/ros/05-arm.md, 08-ctrl.md · 배포 결정 2026-11-03 (bee skill-training 9장)
// ROS 노드가 아니라 controller_manager 안에서 arm_controller(JTC) 바로 뒤에 체인으로 붙는 컨트롤러 플러그인.
//   reference interface: rl_arm_correction/<joint>/position  ← JTC가 command_joints로 여기에 쓴다 (기준 궤적 q_ref)
//   command interface:   <joint>/position                     → hardware interface
//   state interface:     <joint>/position, velocity           (관측)
//   구독: /skill_context (ml/SkillContext, coordinator) · /hand_relative (perception/HandRelative)
// update (update_rate, 100Hz): command = q_ref + 처짐 적분 corr + 학습 보정 Δ
//   - corr = 처짐 보정 적분 (송정혁, isaac/lab place_env._sag_step과 같은 법칙, 시간 기준): reference가 멈춰 있을 때만
//     (모든 관절 |d ref/dt| < still_rate) corr += sag_gain·dt·(reference + Δ − 현재각), ±sag_max. 움직이는 동안은 유지.
//     학습은 85 ms마다 오차 30 % 제거 → sag_gain = −ln(0.7)/0.085 ≈ 4.2 /s. 스킬 시작마다 ~/reset_sag 로 0.
//     skill과 상관없이 늘 돈다 (sag_enabled). 발열 방지: stow(rest_pose)에선 0으로 풂 · 상한에 windup_s 붙으면 풀고 쉼
//   - Δ: skill_context의 skill에 정책 파일이 있을 때만 (닿는 구간). 아니면 Δ = 0
//   - 정책: policy_hz(50)로 arm_obs(…) → MlpPolicy → a ∈ [−1, 1] → residual_live면 Δ = residual_rad·a (아니면 마지막 Δ 유지)
//   - run_id가 바뀌면 corr·Δ·이전 행동을 0부터 (시뮬 에피소드 시작)
//   - 손 카메라 값(/hand_relative)이 max_cam_age_s보다 오래되거나 valid가 아니면 Δ를 바꾸지 않음
#include <algorithm>
#include <array>
#include <cmath>
#include <atomic>
#include <map>
#include <string>
#include <vector>

#include "controller_interface/chainable_controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "ml/arm_obs.hpp"
#include "ml/mlp_policy.hpp"
#include "ml/msg/skill_context.hpp"
#include "perception/msg/hand_relative.hpp"
#include "realtime_tools/realtime_thread_safe_box.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace ml {

// 처짐 보정 적분 한 스텝 (송정혁): corr + gain·dt·(ref + delta − q), ±max로 자름
inline double sag_step(double corr, double ref, double delta, double q, double dt, double gain, double max) {
  return std::clamp(corr + gain * dt * (ref + delta - q), -max, max);
}

// 발열 방지 (11-03): 처짐 적분이 서보를 계속 밀지 않게
//   rest    = reference가 쉬는 자세(stow) → 적분 안 하고 0으로 풂 (주행 내내 접힌 팔을 밀지 않음)
//   blocked = 적분이 상한(±max)에 windup_s 넘게 붙어 있음 → 처짐이 아니라 무언가에 닿아 못 감 → 0으로 풀고 다음 reset까지 쉼
//   moving  = reference가 움직이는 중 → 그대로 유지 (송정혁 방식)
struct SagJoint {
  double corr = 0.0, sat_s = 0.0;
  bool blocked = false;
};
inline void sag_joint_step(SagJoint & j, bool still, bool rest, double ref, double delta, double q, double dt,
                           double gain, double max, double windup_s) {
  if (rest || j.blocked) {
    j.corr -= j.corr * std::min(1.0, gain * dt);  // 한 번에 튀지 않게 같은 속도로 0까지
    j.sat_s = 0.0;
    return;
  }
  if (!still) return;
  j.corr = sag_step(j.corr, ref, delta, q, dt, gain, max);
  j.sat_s = std::abs(j.corr) >= max - 1e-9 ? j.sat_s + dt : 0.0;
  if (windup_s > 0.0 && j.sat_s >= windup_s) j.blocked = true;
}

using controller_interface::CallbackReturn;
using controller_interface::InterfaceConfiguration;
using controller_interface::return_type;

class RlArmCorrection : public controller_interface::ChainableControllerInterface {
public:
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  InterfaceConfiguration command_interface_configuration() const override;
  InterfaceConfiguration state_interface_configuration() const override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

protected:
  std::vector<hardware_interface::CommandInterface::SharedPtr> on_export_reference_interfaces_list() override;
  // 토픽으로 reference를 받지 않음 — 항상 JTC 뒤 체인 모드
  return_type update_reference_from_subscribers(const rclcpp::Time &, const rclcpp::Duration &) override;
  return_type update_and_write_commands(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  struct Skill {
    ArmLayout layout = ArmLayout::Place;
    MlpPolicy policy;
  };
  void reset_run();

  std::vector<std::string> joints_;
  std::vector<hardware_interface::CommandInterface::SharedPtr> refs_;
  std::map<std::string, Skill> skills_;  // skill 이름 → 정책 (파일이 있는 것만)

  realtime_tools::RealtimeThreadSafeBox<ml::msg::SkillContext> ctx_box_;
  realtime_tools::RealtimeThreadSafeBox<perception::msg::HandRelative> hand_box_;
  rclcpp::Subscription<ml::msg::SkillContext>::SharedPtr ctx_sub_;
  rclcpp::Subscription<perception::msg::HandRelative>::SharedPtr hand_sub_;

  ml::msg::SkillContext ctx_;            // 마지막으로 받은 것 (실시간 쪽 복사본)
  perception::msg::HandRelative hand_;
  uint32_t run_id_ = 0;
  J5 corr_{}, delta_{}, prev_act_{}, prev_ref_{};
  std::array<SagJoint, 5> sag_{};
  std::vector<double> rest_pose_;  // 비면 쉬는 자세 판정 안 함
  bool have_prev_ = false;
  std::atomic<bool> reset_sag_{false};
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
  double since_policy_ = 0.0;
  std::vector<double> obs_, act_;
  // 파라미터 (on_configure에서 읽음 — 실시간 루프에서 get_parameter 안 함)
  bool enabled_ = true, sag_enabled_ = true;
  double max_delta_ = 0.05, residual_rad_ = 0.05, sag_gain_ = 4.2, sag_max_ = 0.2, still_rate_ = 0.005;
  double rest_tol_ = 0.03, windup_s_ = 2.0;
  double policy_dt_ = 0.02, max_cam_age_s_ = 1.0;
};

}  // namespace ml
