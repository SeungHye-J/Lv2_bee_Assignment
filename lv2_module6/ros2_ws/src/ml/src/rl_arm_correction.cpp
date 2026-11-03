// rl_arm_correction 구현 — 설명·선언은 include/ml/rl_arm_correction.hpp
#include "ml/rl_arm_correction.hpp"

#include <cmath>
#include <filesystem>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace ml {

CallbackReturn RlArmCorrection::on_init() {
  auto_declare<std::vector<std::string>>("joints", {"joint1", "joint2", "joint3", "joint4", "joint5"});
  auto_declare<bool>("enabled", true);
  auto_declare<double>("max_delta", 0.05);          // Δ 상한 [rad] (안전)
  auto_declare<double>("residual_rad", 0.05);       // 행동 1 = 이만큼 [rad] (시뮬 residual_rad)
  auto_declare<bool>("sag_enabled", true);
  auto_declare<double>("sag_gain", 4.2);            // [1/s] 학습: 85 ms마다 오차 30 % 제거 = −ln(0.7)/0.085 (송정혁)
  auto_declare<double>("sag_max", 0.2);             // [rad]
  auto_declare<double>("still_rate", 0.005);        // [rad/s] reference가 이보다 느리면 '멈춤'
  auto_declare<double>("policy_hz", 50.0);          // 시뮬 정책 주기
  auto_declare<double>("max_cam_age_s", 1.0);       // 손 카메라 값이 이보다 오래되면 Δ 그대로
  auto_declare<std::string>("model_dir", "");       // 비면 share/ml/model/arm_correction
  auto_declare<std::vector<std::string>>("skills", {"place_cell", "load_slot", "pick_upright", "unload_slot"});
  auto_declare<std::vector<std::string>>("layouts", {"place", "place", "pick", "pick"});  // skills와 같은 순서
  return CallbackReturn::SUCCESS;
}

CallbackReturn RlArmCorrection::on_configure(const rclcpp_lifecycle::State &) {
  auto node = get_node();
  joints_ = node->get_parameter("joints").as_string_array();
  enabled_ = node->get_parameter("enabled").as_bool();
  max_delta_ = node->get_parameter("max_delta").as_double();
  residual_rad_ = node->get_parameter("residual_rad").as_double();
  sag_enabled_ = node->get_parameter("sag_enabled").as_bool();
  sag_gain_ = node->get_parameter("sag_gain").as_double();
  sag_max_ = node->get_parameter("sag_max").as_double();
  still_rate_ = node->get_parameter("still_rate").as_double();
  // 스킬 시작마다 호출 (학습 에피소드도 corr = 0에서 시작). 실제 리셋은 다음 update에서 (송정혁)
  reset_srv_ = node->create_service<std_srvs::srv::Trigger>(
    "~/reset_sag", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
      reset_sag_ = true;
      res->success = true;
    });
  policy_dt_ = 1.0 / node->get_parameter("policy_hz").as_double();
  max_cam_age_s_ = node->get_parameter("max_cam_age_s").as_double();
  std::string dir = node->get_parameter("model_dir").as_string();
  if (dir.empty()) dir = ament_index_cpp::get_package_share_directory("ml") + "/model/arm_correction";
  const auto names = node->get_parameter("skills").as_string_array();
  const auto layouts = node->get_parameter("layouts").as_string_array();
  skills_.clear();
  for (size_t i = 0; i < names.size() && i < layouts.size(); ++i) {
    Skill s;
    std::string why;
    const std::string path = dir + "/" + names[i] + ".policy";
    if (!arm_layout_from(layouts[i], s.layout)) {
      RCLCPP_WARN(node->get_logger(), "%s: 입력 모양 '%s' 모름 — 이 skill은 보정 없음", names[i].c_str(), layouts[i].c_str());
      continue;
    }
    if (!std::filesystem::exists(path)) {
      RCLCPP_INFO(node->get_logger(), "%s: 정책 파일 없음 (%s) — 이 skill은 보정 없음", names[i].c_str(), path.c_str());
      continue;
    }
    if (!s.policy.load(path, why) || s.policy.obs_dim() != arm_obs_size(s.layout) || s.policy.act_dim() != 5) {
      RCLCPP_ERROR(node->get_logger(), "%s: 정책을 못 씀 — %s (입력 %zu, 필요 %zu)", names[i].c_str(), why.c_str(),
                   s.policy.obs_dim(), arm_obs_size(s.layout));
      continue;
    }
    RCLCPP_INFO(node->get_logger(), "%s: 정책 %s (%s 모양)", names[i].c_str(), path.c_str(), layouts[i].c_str());
    skills_[names[i]] = std::move(s);
  }
  obs_.assign(41, 0.0);
  act_.assign(5, 0.0);
  ctx_sub_ = node->create_subscription<ml::msg::SkillContext>(
    "/skill_context", rclcpp::QoS(10).reliable(), [this](ml::msg::SkillContext::ConstSharedPtr m) { ctx_box_.set(*m); });
  hand_sub_ = node->create_subscription<perception::msg::HandRelative>(
    "/hand_relative", 10, [this](perception::msg::HandRelative::ConstSharedPtr m) { hand_box_.set(*m); });
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
  ctx_ = ml::msg::SkillContext();
  reset_run();
  corr_.fill(0.0);
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

void RlArmCorrection::reset_run() {  // 보정 Δ만 (처짐 적분은 ~/reset_sag · 닿는 구간 시작에서)
  delta_.fill(0.0);
  prev_act_.fill(0.0);
  since_policy_ = 0.0;
}

return_type RlArmCorrection::update_and_write_commands(const rclcpp::Time & time, const rclcpp::Duration & period) {
  if (reset_sag_.exchange(false) || !sag_enabled_) corr_.fill(0.0);
  if (auto c = ctx_box_.try_get()) {  // 못 잡으면 (콜백이 쓰는 중) 지난 값으로
    if (c->run_id != ctx_.run_id || c->skill != ctx_.skill) {
      // 새 닿는 구간: 시뮬 에피소드처럼 보정·처짐 적분 0부터 (skill이 꺼질 때는 처짐 적분 유지)
      if (!c->skill.empty()) corr_.fill(0.0);
      run_id_ = c->run_id;
      reset_run();
    }
    ctx_ = *c;
  }
  if (auto h = hand_box_.try_get()) hand_ = *h;

  J5 q{}, qd{}, ref{};
  bool ok = joints_.size() == 5;
  for (size_t i = 0; i < joints_.size() && i < 5; ++i) {
    const auto r = refs_[i]->get_optional<double>();
    const auto p = state_interfaces_[2 * i].get_optional<double>();
    const auto v = state_interfaces_[2 * i + 1].get_optional<double>();
    if (!r || std::isnan(*r) || !p) { ok = false; break; }
    ref[i] = *r;
    q[i] = *p;
    qd[i] = v ? *v : 0.0;
  }

  const double dt = period.seconds();
  // 멈춤 판정 (송정혁): 모든 관절의 reference 변화율이 still_rate 미만 (첫 주기·reference 없음은 움직이는 것으로)
  bool still = have_prev_ && ok && dt > 0.0;
  for (int i = 0; ok && i < 5; ++i) {
    if (std::abs(ref[i] - prev_ref_[i]) > still_rate_ * dt) still = false;
    prev_ref_[i] = ref[i];
  }
  have_prev_ = ok;

  const auto it = enabled_ && ok ? skills_.find(ctx_.skill) : skills_.end();
  if (it == skills_.end()) {  // 보정 꺼짐: 닿는 구간이 아니거나 이 skill 정책이 없음 → Δ = 0 (처짐 적분은 계속)
    if (delta_ != J5{}) reset_run();
  } else {
    since_policy_ += dt;
    const bool cam_kind = it->second.layout == ArmLayout::Pick ? hand_.kind == "puck" : (hand_.kind == "cell" || hand_.kind == "slot");
    const double cam_age = (time - rclcpp::Time(hand_.header.stamp, time.get_clock_type())).seconds();
    const bool cam_ok = hand_.valid && cam_kind && cam_age >= -0.1 && cam_age <= max_cam_age_s_;
    if (since_policy_ >= policy_dt_ - 1e-6 && ctx_.q_above.size() == 5 && ctx_.q_down.size() == 5 && cam_ok) {
      since_policy_ = 0.0;
      ArmObsIn in;
      in.q = q; in.qd = qd; in.q_ref = ref; in.prev_act = prev_act_; in.corr = corr_;
      std::copy(ctx_.q_above.begin(), ctx_.q_above.end(), in.q_above.begin());
      std::copy(ctx_.q_down.begin(), ctx_.q_down.end(), in.q_down.begin());
      in.phase = ctx_.phase;
      in.prog_s = (time - rclcpp::Time(ctx_.header.stamp, time.get_clock_type())).seconds();
      in.cam_x = hand_.rel.x;
      in.cam_y = hand_.rel.y;
      in.face_yaw = hand_.face_yaw;
      in.is_box = hand_.is_box;
      arm_obs(it->second.layout, in, obs_.data());
      it->second.policy.act_into(obs_.data(), act_.data());
      for (int i = 0; i < 5; ++i) {
        const double a = std::clamp(act_[i], -1.0, 1.0);
        if (ctx_.residual_live) delta_[i] = std::clamp(residual_rad_ * a, -max_delta_, max_delta_);
        prev_act_[i] = a;
      }
    }
  }
  for (size_t i = 0; i < joints_.size() && i < 5; ++i) {
    if (!ok) continue;  // reference가 아직 없음 — 아무것도 안 씀 (예전처럼)
    if (sag_enabled_ && still) corr_[i] = sag_step(corr_[i], ref[i], delta_[i], q[i], dt, sag_gain_, sag_max_);
    (void)command_interfaces_[i].set_value(ref[i] + corr_[i] + delta_[i]);
  }
  return return_type::OK;
}

}  // namespace ml

PLUGINLIB_EXPORT_CLASS(ml::RlArmCorrection, controller_interface::ChainableControllerInterface)
