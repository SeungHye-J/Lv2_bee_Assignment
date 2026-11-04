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
  auto_declare<bool>("gravity_enabled", false);       // 중력 처짐 앞먹임 (헤더 설명)
  auto_declare<double>("gravity_compliance_j2", 0.0817); // [rad/Nm] = 1/강성 (실측 isaac/sim/R2S/sag_fit.py 2026-11-04: j2 12.2 · j3 18.8 Nm/rad, j4는 0과 구분 안 됨)
  auto_declare<double>("gravity_compliance_j3", 0.0532);
  auto_declare<double>("gravity_compliance_j4", 0.0);
  auto_declare<double>("gravity_z_offset_m", 0.0);    // [m] 고정 높이 오차를 올리는 양 (실측 처짐이 −5 mm 고정이면 0.005)
  auto_declare<double>("gravity_ramp_s", 2.0);        // [s] 켜고 끌 때 서서히
  auto_declare<double>("gravity_max", 0.2);           // [rad] 앞먹임 한 관절 상한
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
  grav_raw_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("~/gravity", rclcpp::QoS(1).best_effort());
  grav_pub_ = std::make_unique<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(grav_raw_);
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
  grav_scale_ = 0.0;
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
  const bool grav_enabled = node->get_parameter("gravity_enabled").as_bool();
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

  // 중력 처짐 앞먹임 (헤더 설명) — 켜졌거나 서서히 끄는 중일 때만. joint2~4 각도는 엔코더 값 (state_interfaces_[2 * i] = 관절 i의 position)
  std::array<double, 5> grav{};
  const double ramp_step = dt / std::max(node->get_parameter("gravity_ramp_s").as_double(), 1e-3);
  grav_scale_ = grav_enabled ? std::min(1.0, grav_scale_ + ramp_step) : std::max(0.0, grav_scale_ - ramp_step);
  if (grav_scale_ > 0.0 && joints_.size() == 5) {
    const auto q2 = state_interfaces_[2].get_optional<double>();
    const auto q3 = state_interfaces_[4].get_optional<double>();
    const auto q4 = state_interfaces_[6].get_optional<double>();
    if (q2 && q3 && q4) {
      const auto tau = gravity::torques(*q2, *q3, *q4);
      const double c[3] = {node->get_parameter("gravity_compliance_j2").as_double(),
                           node->get_parameter("gravity_compliance_j3").as_double(),
                           node->get_parameter("gravity_compliance_j4").as_double()};
      std::array<double, 3> d = {-c[0] * tau[0], -c[1] * tau[1], -c[2] * tau[2]};  // 비틀림 d = c·τ 를 미리 뺀다
      const double z_off = node->get_parameter("gravity_z_offset_m").as_double();
      std::array<double, 3> up{};
      if (z_off != 0.0 && gravity::lift(*q2, *q3, *q4, z_off, up))
        for (size_t k = 0; k < 3; ++k) d[k] += up[k];
      const double gmax = node->get_parameter("gravity_max").as_double();
      for (size_t k = 0; k < 3; ++k) grav[k + 1] = grav_scale_ * std::clamp(d[k], -gmax, gmax);
    }
  }

  for (size_t i = 0; i < joints_.size(); ++i) {
    const auto ref = refs_[i]->get_optional<double>();
    if (!ref || std::isnan(*ref)) continue;
    // 정책 Δ (50 Hz로 받아 사이 주기엔 유지) — 꺼졌거나 낡았거나 길이가 안 맞으면 0
    double delta = (enabled && in && in->d.size() == joints_.size()) ? fresh_delta(in->d[i], age, delta_timeout, max_delta) : 0.0;
    last_delta_[i] = delta;
    const auto q = state_interfaces_[2 * i].get_optional<double>();
    const double total = delta + grav[i];  // 정책 Δ + 중력 앞먹임 — 둘 다 오차에 넣어 처짐 보정 적분이 지우지 않는다
    if (sag_enabled && still && q) corr_[i] = sag_step(corr_[i], *ref, total, *q, dt, sag_gain, sag_max);
    (void)command_interfaces_[i].set_value(*ref + corr_[i] + total);
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
  if (grav_pub_) {
    std_msgs::msg::Float64MultiArray g;
    g.data.assign(grav.begin(), grav.begin() + static_cast<long>(joints_.size() < grav.size() ? joints_.size() : grav.size()));
    grav_pub_->try_publish(g);
  }
  return return_type::OK;
}

}  // namespace ml

PLUGINLIB_EXPORT_CLASS(ml::RlArmCorrection, controller_interface::ChainableControllerInterface)
