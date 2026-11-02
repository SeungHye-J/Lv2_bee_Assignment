#pragma once

// rl_arm_correction (🛠 직접 작성, 담당: 송정혁) — 설계: documents/project/ros/05-arm.md, 08-ctrl.md
// ROS 노드가 아니라 controller_manager 안에서 arm_controller(JTC) 바로 뒤에 체인으로 붙는 컨트롤러 플러그인.
//   reference interface: rl_arm_correction/<joint>/position  ← JTC가 command_joints로 여기에 쓴다
//   command interface:   <joint>/position                     → hardware interface
//   state interface:     <joint>/position, velocity           (관측)
// update: command = reference + corr + Δ (Δ는 ±max_delta, 기본 0.05rad). enabled:=false 면 Δ=0.
//   corr = 처짐 보정 적분 (isaac/lab place_env._sag_step과 같은 법칙, 시간 기준):
//     reference가 멈춰 있을 때만 (모든 관절 |d ref/dt| < still_rate) corr += sag_gain·dt·(reference + Δ − 현재각), ±sag_max.
//     움직이는 동안은 corr 유지(추종 지연을 처짐으로 쌓지 않게). 오차에 Δ를 넣어 정책 보정은 지우지 않는다.
//     학습은 85 ms마다 오차의 30 % 제거 → 같은 감쇠율 sag_gain = -ln(0.7)/0.085 ≈ 4.2 /s. 처짐은 자세마다 달라서 스킬 시작마다 ~/reset_sag 로 0.
// TODO(송정혁): 관측(목표·현재 관절각·속도·추종 오차·파지 여부·직전 Δ) → ONNX 정책 → Δ
#include <algorithm>
#include <atomic>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/chainable_controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace ml {

// 처짐 보정 적분 한 스텝: corr + gain·dt·(ref + delta − q), ±max로 자름
inline double sag_step(double corr, double ref, double delta, double q, double dt, double gain, double max) {
  return std::clamp(corr + gain * dt * (ref + delta - q), -max, max);
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

  // 토픽 입력 없음 — 항상 JTC 뒤 체인 모드로만 쓴다
  return_type update_reference_from_subscribers(const rclcpp::Time &, const rclcpp::Duration &) override;

  return_type update_and_write_commands(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  std::vector<std::string> joints_;
  std::vector<hardware_interface::CommandInterface::SharedPtr> refs_;
  std::vector<double> corr_, prev_ref_;
  bool have_prev_ = false;
  std::atomic<bool> reset_sag_{false};
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
};

}  // namespace ml
