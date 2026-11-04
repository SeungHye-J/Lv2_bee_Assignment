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
// Δ = 정책 노드(ml/rl_policy_node — 관측 구성·추론은 거기서)가 ~/delta(Float64MultiArray, 관절 순서 joints)로 보낸 값.
//   delta_timeout 넘게 안 오면 0 (정책 노드가 죽거나 스킬 밖). 관측용으로 ~/state = [reference×N, corr×N, Δ×N]를 매 주기 발행.
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/chainable_controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace ml {

// 처짐 보정 적분 한 스텝: corr + gain·dt·(ref + delta − q), ±max로 자름
inline double sag_step(double corr, double ref, double delta, double q, double dt, double gain, double max) {
  return std::clamp(corr + gain * dt * (ref + delta - q), -max, max);
}

// 정책 Δ 한 관절: 받은 지 timeout_s 넘었으면 0 (낡은 값을 계속 더하지 않게), 아니면 ±max로 자름
inline double fresh_delta(double value, double age_s, double timeout_s, double max) {
  if (!(age_s >= 0.0) || age_s > timeout_s || std::isnan(value)) return 0.0;
  return std::clamp(value, -max, max);
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
  struct DeltaIn {
    std::vector<double> d;
    int64_t stamp_ns = 0;  // 받은 시각 (steady_clock) — 제어 주기 시각과 시계 종류가 달라도 비교되게
  };
  realtime_tools::RealtimeBuffer<DeltaIn> delta_in_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr delta_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr state_raw_;
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>> state_pub_;
  std::vector<double> last_delta_;
};

}  // namespace ml
