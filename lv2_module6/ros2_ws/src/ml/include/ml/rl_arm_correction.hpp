#pragma once

// rl_arm_correction (담당: 송정혁) — 설계: documents/project/ros/05-arm.md, 08-ctrl.md · 배포 결정 2026-11-03 (bee skill-training 9장)
// ROS 노드가 아니라 controller_manager 안에서 arm_controller(JTC) 바로 뒤에 체인으로 붙는 컨트롤러 플러그인.
//   reference interface: rl_arm_correction/<joint>/position  ← JTC가 command_joints로 여기에 쓴다 (기준 궤적 q_ref)
//   command interface:   <joint>/position                     → hardware interface
//   state interface:     <joint>/position, velocity           (관측)
//   구독: /skill_context (ml/SkillContext, coordinator) · /hand_relative (perception/HandRelative)
// update (update_rate, 100Hz): command = q_ref + 처짐 적분 corr + 학습 보정 Δ
//   - skill_context의 skill에 정책 파일이 있을 때만 켜짐 (닿는 구간). 아니면 corr = Δ = 0 → JTC 그대로 통과
//   - 처짐 적분: holding일 때 corr_dt마다 corr += gain·(q_ref + Δ − q) (시뮬 place_env._sag_step과 같음)
//   - 정책: policy_hz(50)로 arm_obs(…) → MlpPolicy → a ∈ [−1, 1] → residual_live면 Δ = residual_rad·a (아니면 마지막 Δ 유지)
//   - run_id가 바뀌면 corr·Δ·이전 행동을 0부터 (시뮬 에피소드 시작)
//   - 손 카메라 값(/hand_relative)이 max_cam_age_s보다 오래되거나 valid가 아니면 Δ를 바꾸지 않음
#include <array>
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

namespace ml {

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
  J5 corr_{}, delta_{}, prev_act_{};
  double since_sag_ = 0.0, since_policy_ = 0.0;
  std::vector<double> obs_, act_;
  // 파라미터 (on_configure에서 읽음 — 실시간 루프에서 get_parameter 안 함)
  bool enabled_ = true;
  double max_delta_ = 0.05, residual_rad_ = 0.05, gain_ = 0.3, corr_dt_ = 5.0 / 60.0, corr_max_ = 0.2;
  double policy_dt_ = 0.02, max_cam_age_s_ = 1.0;
};

}  // namespace ml
