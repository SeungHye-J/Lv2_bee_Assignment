// rl_nav_correction 구현 — 노드 설명·입출력·멤버 선언은 include/ml/rl_nav_correction.hpp
//   정책: Isaac Lab/rsl_rl → policy.onnx (관측 6 track 모드). ONNX Runtime은 onnxruntime_vendor 사용.
#include "ml/rl_nav_correction.hpp"

#include <filesystem>
#include <utility>

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace ml {

namespace {
constexpr int64_t OBS_TRACK = 6;
}

RlNavCorrection::RlNavCorrection()
: rclcpp::Node("rl_nav_correction") {
  declare_parameter<bool>("enabled", true);
  declare_parameter<std::string>("model_path", "rl_nav_correction/policy.onnx");  // share/ml/model/<노드>/<이름>. "" = 통과
  declare_parameter<double>("max_vel_x", 0.26);
  declare_parameter<double>("max_vel_theta", 1.82);
  declare_parameter<double>("timeout_s", 0.2);
  declare_parameter<double>("action_scale_lin", 0.05);
  declare_parameter<double>("action_scale_ang", 0.10);
  declare_parameter<double>("max_delta_lin", 0.05);
  declare_parameter<double>("max_delta_ang", 0.10);
  declare_parameter<double>("t_change_max", 2.0);

  max_v_ = get_parameter("max_vel_x").as_double();
  max_w_ = get_parameter("max_vel_theta").as_double();
  timeout_s_ = get_parameter("timeout_s").as_double();
  action_scale_lin_ = get_parameter("action_scale_lin").as_double();
  action_scale_ang_ = get_parameter("action_scale_ang").as_double();
  max_delta_lin_ = get_parameter("max_delta_lin").as_double();
  max_delta_ang_ = get_parameter("max_delta_ang").as_double();
  t_change_max_ = get_parameter("t_change_max").as_double();
  cmd_changed_ = now();
  load_model(get_parameter("model_path").as_string());

  // model_path를 실행 중에 바꾸면 즉시 다시 로드 (빈 값 = 통과)
  post_cb_ = add_post_set_parameters_callback([this](const std::vector<rclcpp::Parameter> & ps) {
    for (const auto & p : ps) {
      if (p.get_name() == "model_path") {
        load_model(p.as_string());
      } else if (p.get_name() == "enabled") {
        RCLCPP_WARN(get_logger(), "ONNX 보정 %s", p.as_bool() ? "켬" : "끔 (Δ=0)");
      }
    }
  });

  cmd_sub_ = create_subscription<TwistStamped>(
    "/cmd_vel_nav", 10, [this](TwistStamped::ConstSharedPtr m) { on_cmd(*m); });
  plan_sub_ = create_subscription<nav_msgs::msg::Path>(
    "/plan", 1, [this](nav_msgs::msg::Path::ConstSharedPtr m) { plan_ = m; });
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10, [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
      odom_ = m;
      last_sensor_ = now();
    });
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    "/imu", rclcpp::SensorDataQoS().keep_last(1),
    [this](sensor_msgs::msg::Imu::ConstSharedPtr m) { imu_ = m; });
  pub_ = create_publisher<TwistStamped>("/cmd_vel_corrected", rclcpp::QoS(1).reliable());
  debug_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("~/debug", rclcpp::QoS(1).reliable());
  toggle_srv_ = create_service<std_srvs::srv::SetBool>(
    "~/toggle", [this](
                  const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                  std::shared_ptr<std_srvs::srv::SetBool::Response> res) { on_toggle(req, res); });

  diag_.setHardwareID("beebot");
  diag_.add("상태", [this](diagnostic_updater::DiagnosticStatusWrapper & s) {
    const bool fresh = last_sensor_.nanoseconds() != 0 && (now() - last_sensor_).seconds() <= timeout_s_;
    const bool on = get_parameter("enabled").as_bool() && model_ok_;
    if (!model_ok_) {
      s.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "모델 없음 — Δ=0 (통과): " + model_path_);
    } else if (!on) {
      s.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Δ=0 (보정 꺼짐)");
    } else if (!fresh) {
      s.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "/odom 끊김 — Δ=0 (통과만)");
    } else {
      s.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "ONNX 보정 중");
    }
    s.add("enabled", get_parameter("enabled").as_bool());
    s.add("모델", model_ok_ ? model_path_ : std::string("없음/실패"));
    s.add("관측 차원", static_cast<int>(obs_dim_));
    s.add("마지막 Δ [m/s, rad/s]", std::to_string(last_delta_v_).substr(0, 7) + ", " +
                                      std::to_string(last_delta_w_).substr(0, 7));
    s.add("속도 상한 [m/s]", max_v_);
    s.add("보정한 명령 수", corrected_);
    s.add("추론 실패 수", infer_fail_);
  });
}

std::string RlNavCorrection::resolve_model_path(const std::string & path) const {
  if (path.empty()) return {};
  namespace fs = std::filesystem;
  const fs::path p(path);
  if (p.is_absolute()) return path;  // 절대 경로는 그대로 (개발용 직접 지정)
  // 상대 경로: share/ml/model/<경로> 하나만 본다 (설치 공간이 단일 진실 공급원).
  //   "rl_nav_correction/policy.onnx"처럼 노드별 폴더 포함 — 노드마다 onnx를 가져와도 이름이 안 겹친다.
  //   bare 파일명("policy.onnx")도 그대로 받음. 소스 트리 폴백 없음.
  std::string share;
  try {
    share = (fs::path(ament_index_cpp::get_package_share_directory("ml")) / "model" / p).string();
    if (fs::exists(share)) return share;
  } catch (const std::exception & e) {
    RCLCPP_WARN(get_logger(), "share/ml 조회 실패 (%s): %s — Δ=0 (통과)", path.c_str(), e.what());
    return {};
  }
  RCLCPP_WARN(get_logger(), "모델 파일 없음: %s (기대: %s) — Δ=0 (통과)", path.c_str(), share.c_str());
  return {};
}

bool RlNavCorrection::load_model(const std::string & path) {
  model_ok_ = false;
  obs_dim_ = OBS_TRACK;
  policy_.unload();
  model_path_.clear();
  if (path.empty()) {
    RCLCPP_INFO(get_logger(), "model_path 없음 — Δ=0 (통과)");
    return false;
  }
  const std::string resolved = resolve_model_path(path);
  if (resolved.empty()) return false;  // 없음·실패는 resolve_model_path가 경고
  try {
    policy_.load(resolved);
    obs_dim_ = policy_.inputs();
    model_path_ = resolved;
    model_ok_ = (policy_.inputs() == OBS_TRACK && policy_.outputs() >= 2);
    if (!model_ok_) {
      RCLCPP_WARN(get_logger(), "모델 차원 불일치 (관측 %ld, 출력 %ld) — 기대 관측 6·출력 2 이상: Δ=0",
                  static_cast<long>(policy_.inputs()), static_cast<long>(policy_.outputs()));
    } else {
      RCLCPP_INFO(get_logger(), "ONNX 로드: %s (관측 %ld → 출력 %ld, action_scale %.3f/%.3f, max_delta %.3f/%.3f)",
                  resolved.c_str(), static_cast<long>(policy_.inputs()),
                  static_cast<long>(policy_.outputs()), action_scale_lin_, action_scale_ang_,
                  max_delta_lin_, max_delta_ang_);
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN(get_logger(), "ONNX 로드 실패 (%s): Δ=0 (통과)", e.what());
  }
  return model_ok_;
}

void RlNavCorrection::on_toggle(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
  std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
  set_parameter(rclcpp::Parameter("enabled", static_cast<bool>(req->data)));
  res->success = true;
  res->message = std::string("enabled=") + (req->data ? "true" : "false") +
                 " · 모델=" + (model_ok_ ? model_path_ : "없음(통과)") +
                 " · 관측=" + std::to_string(static_cast<int>(obs_dim_));
  RCLCPP_INFO(get_logger(), "toggle: %s", res->message.c_str());
}

void RlNavCorrection::on_cmd(const TwistStamped & in) {
  const bool fresh = last_sensor_.nanoseconds() != 0 && (now() - last_sensor_).seconds() <= timeout_s_;
  const bool on = get_parameter("enabled").as_bool() && model_ok_ && fresh;

  // 명령이 바뀐 시각 (t_change 관측)
  if (std::abs(in.twist.linear.x - last_cmd_.first) > 1e-3 ||
      std::abs(in.twist.angular.z - last_cmd_.second) > 1e-3) {
    cmd_changed_ = now();
  }
  last_cmd_ = {in.twist.linear.x, in.twist.angular.z};

  double dv = 0.0, dw = 0.0, t_change_obs = 0.0;
  if (on) {
    const double v_odom = odom_ ? odom_->twist.twist.linear.x : 0.0;
    const double w_odom = odom_ ? odom_->twist.twist.angular.z : 0.0;
    const double w_gyro = imu_ ? imu_->angular_velocity.z : w_odom;
    t_change_obs = std::min(t_change_max_, std::max(0.0, (now() - cmd_changed_).seconds()));
    std::vector<float> obs{static_cast<float>(in.twist.linear.x), static_cast<float>(in.twist.angular.z),
                           static_cast<float>(v_odom), static_cast<float>(w_odom),
                           static_cast<float>(w_gyro), static_cast<float>(t_change_obs)};
    try {
      const std::vector<float> out = policy_.run(obs);
      const auto scaled = scale_action(out[0], out[1], action_scale_lin_, action_scale_ang_);
      dv = std::clamp(scaled.first, -max_delta_lin_, max_delta_lin_);
      dw = std::clamp(scaled.second, -max_delta_ang_, max_delta_ang_);
    } catch (const std::exception & e) {
      ++infer_fail_;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "ONNX 추론 실패: %s — Δ=0", e.what());
    }
  }
  last_delta_v_ = dv;
  last_delta_w_ = dw;

  const auto out = apply_delta({in.twist.linear.x, in.twist.angular.z}, {dv, dw}, max_v_, max_w_);
  TwistStamped msg = in;
  msg.twist.linear.x = out.first;
  msg.twist.angular.z = out.second;
  pub_->publish(msg);
  ++corrected_;

  std_msgs::msg::Float32MultiArray dbg;
  dbg.data = {static_cast<float>(in.twist.linear.x), static_cast<float>(in.twist.angular.z),
              static_cast<float>(odom_ ? odom_->twist.twist.linear.x : 0.0),
              static_cast<float>(odom_ ? odom_->twist.twist.angular.z : 0.0),
              static_cast<float>(imu_ ? imu_->angular_velocity.z : 0.0),
              static_cast<float>(t_change_obs), static_cast<float>(dv), static_cast<float>(dw),
              static_cast<float>(out.first), static_cast<float>(out.second),
              static_cast<float>(on ? 1.0 : 0.0)};
  debug_pub_->publish(dbg);

  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 2000,
    "보정 %s: in=(%+.3f,%+.3f) Δ=(%+.3f,%+.3f) out=(%+.3f,%+.3f)", on ? "ON" : "OFF",
    in.twist.linear.x, in.twist.angular.z, dv, dw, out.first, out.second);
}

}  // namespace ml
