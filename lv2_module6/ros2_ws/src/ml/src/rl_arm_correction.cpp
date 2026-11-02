// rl_arm_correction 구현 — 설명·선언은 include/ml/rl_arm_correction.hpp
#include "ml/rl_arm_correction.hpp"

#include <cmath>

#include "pluginlib/class_list_macros.hpp"

namespace ml {

CallbackReturn RlArmCorrection::on_init() {
  auto_declare<std::vector<std::string>>("joints", {"joint1", "joint2", "joint3", "joint4", "joint5"});
  auto_declare<bool>("enabled", true);
  auto_declare<double>("max_delta", 0.05);
  auto_declare<bool>("sag_enabled", true);
  auto_declare<double>("sag_gain", 4.2);     // [1/s] 학습: 85 ms마다 오차 30 % 제거 = -ln(0.7)/0.085
  auto_declare<double>("sag_max", 0.2);      // [rad]
  auto_declare<double>("still_rate", 0.005); // [rad/s] reference가 이보다 느리면 '멈춤'
  return CallbackReturn::SUCCESS;
}

CallbackReturn RlArmCorrection::on_configure(const rclcpp_lifecycle::State &) {
  joints_ = get_node()->get_parameter("joints").as_string_array();
  corr_.assign(joints_.size(), 0.0);
  prev_ref_.assign(joints_.size(), 0.0);
  // 스킬 시작마다 호출 (학습 에피소드도 corr = 0에서 시작). 실제 리셋은 다음 update에서
  reset_srv_ = get_node()->create_service<std_srvs::srv::Trigger>(
    "~/reset_sag", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
      reset_sag_ = true;
      res->success = true;
    });
  return CallbackReturn::SUCCESS;
}

InterfaceConfiguration RlArmCorrection::command_interface_configuration() const {
  InterfaceConfiguration cfg{controller_interface::interface_configuration_type::INDIVIDUAL, {}};
  for (const auto & j : joints_) cfg.names.push_back(j + "/" + hardware_interface::HW_IF_POSITION);
  return cfg;
}

InterfaceConfiguration RlArmCorrection::state_interface_configuration() const {
  InterfaceConfiguration cfg{controller_interface::interface_configuration_type::INDIVIDUAL, {}};
  for (const auto & j : joints_) {
    cfg.names.push_back(j + "/" + hardware_interface::HW_IF_POSITION);
    cfg.names.push_back(j + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  return cfg;
}

CallbackReturn RlArmCorrection::on_activate(const rclcpp_lifecycle::State &) {
  // 시작할 때 reference를 현재 자세로 — 첫 주기에 0rad로 튀지 않게
  for (size_t i = 0; i < refs_.size(); ++i) {
    const auto pos = state_interfaces_[2 * i].get_optional<double>();
    if (pos) (void)refs_[i]->set_value(*pos);
  }
  std::fill(corr_.begin(), corr_.end(), 0.0);
  have_prev_ = false;
  return CallbackReturn::SUCCESS;
}

CallbackReturn RlArmCorrection::on_deactivate(const rclcpp_lifecycle::State &) { return CallbackReturn::SUCCESS; }

std::vector<hardware_interface::CommandInterface::SharedPtr> RlArmCorrection::on_export_reference_interfaces_list() {
  refs_.clear();
  for (const auto & j : joints_) {
    hardware_interface::InterfaceInfo info;
    info.name = j + "/" + hardware_interface::HW_IF_POSITION;
    info.data_type = "double";
    refs_.push_back(std::make_shared<hardware_interface::CommandInterface>(
      hardware_interface::InterfaceDescription(get_node()->get_name(), info)));
  }
  return refs_;
}

return_type RlArmCorrection::update_reference_from_subscribers(const rclcpp::Time &, const rclcpp::Duration &) {
  return return_type::OK;
}

return_type RlArmCorrection::update_and_write_commands(const rclcpp::Time &, const rclcpp::Duration & period) {
  const auto node = get_node();
  const bool enabled = node->get_parameter("enabled").as_bool();
  const double max_delta = node->get_parameter("max_delta").as_double();
  const bool sag_enabled = node->get_parameter("sag_enabled").as_bool();
  const double sag_gain = node->get_parameter("sag_gain").as_double();
  const double sag_max = node->get_parameter("sag_max").as_double();
  const double still_rate = node->get_parameter("still_rate").as_double();
  const double dt = period.seconds();
  if (reset_sag_.exchange(false) || !sag_enabled) std::fill(corr_.begin(), corr_.end(), 0.0);

  // 멈춤 판정: 모든 관절의 reference 변화율이 still_rate 미만 (첫 주기·dt 이상은 움직이는 것으로 본다)
  bool still = have_prev_ && dt > 0.0;
  bool all_ref = true;
  for (size_t i = 0; i < joints_.size(); ++i) {
    const auto ref = refs_[i]->get_optional<double>();
    if (!ref || std::isnan(*ref)) {
      all_ref = false;
      continue;
    }
    if (still && std::abs(*ref - prev_ref_[i]) > still_rate * dt) still = false;
    prev_ref_[i] = *ref;
  }
  have_prev_ = all_ref;

  for (size_t i = 0; i < joints_.size(); ++i) {
    const auto ref = refs_[i]->get_optional<double>();
    if (!ref || std::isnan(*ref)) continue;
    double delta = 0.0;  // TODO: 정책 추론 (50 Hz로 학습: 2주기에 1번 추론, 사이엔 유지)
    if (!enabled) delta = 0.0;
    delta = std::clamp(delta, -max_delta, max_delta);
    const auto q = state_interfaces_[2 * i].get_optional<double>();
    if (sag_enabled && still && q) corr_[i] = sag_step(corr_[i], *ref, delta, *q, dt, sag_gain, sag_max);
    (void)command_interfaces_[i].set_value(*ref + corr_[i] + delta);
  }
  return return_type::OK;
}

}  // namespace ml

PLUGINLIB_EXPORT_CLASS(ml::RlArmCorrection, controller_interface::ChainableControllerInterface)
