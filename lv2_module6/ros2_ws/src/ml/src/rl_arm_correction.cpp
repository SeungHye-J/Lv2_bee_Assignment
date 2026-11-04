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
  auto_declare<double>("delta_timeout", 0.2); // [s] 정책 Δ가 이보다 오래 안 오면 0
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
  // 정책 Δ (rl_policy_node, 50 Hz) — 비실시간 스레드에서 받아 실시간 버퍼로
  delta_sub_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
    "~/delta", rclcpp::QoS(1).best_effort(), [this](std_msgs::msg::Float64MultiArray::ConstSharedPtr m) {
      DeltaIn in;
      in.d = m->data;
      in.stamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
      delta_in_.writeFromNonRT(in);
    });
  state_raw_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("~/state", rclcpp::QoS(1).best_effort());
  state_pub_ = std::make_unique<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(state_raw_);
  last_delta_.assign(joints_.size(), 0.0);
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
  const double delta_timeout = node->get_parameter("delta_timeout").as_double();
  const DeltaIn * in = delta_in_.readFromRT();
  const double age = in ? 1e-9 * static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count() - in->stamp_ns) : -1.0;
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
    // 정책 Δ (50 Hz로 받아 사이 주기엔 유지) — 꺼졌거나 낡았거나 길이가 안 맞으면 0
    double delta = (enabled && in && in->d.size() == joints_.size()) ? fresh_delta(in->d[i], age, delta_timeout, max_delta) : 0.0;
    last_delta_[i] = delta;
    const auto q = state_interfaces_[2 * i].get_optional<double>();
    if (sag_enabled && still && q) corr_[i] = sag_step(corr_[i], *ref, delta, *q, dt, sag_gain, sag_max);
    (void)command_interfaces_[i].set_value(*ref + corr_[i] + delta);
  }
  // 관측용 상태 [reference, corr, Δ] — 잠금을 못 잡으면 이번 주기는 건너뜀 (실시간 루프를 막지 않음)
  std_msgs::msg::Float64MultiArray st;
  st.data.reserve(3 * joints_.size());
  for (size_t i = 0; i < joints_.size(); ++i) {
    const auto ref = refs_[i]->get_optional<double>();
    st.data.push_back(ref ? *ref : std::numeric_limits<double>::quiet_NaN());
  }
  st.data.insert(st.data.end(), corr_.begin(), corr_.end());
  st.data.insert(st.data.end(), last_delta_.begin(), last_delta_.end());
  if (state_pub_) state_pub_->try_publish(st);
  return return_type::OK;
}

}  // namespace ml

PLUGINLIB_EXPORT_CLASS(ml::RlArmCorrection, controller_interface::ChainableControllerInterface)
