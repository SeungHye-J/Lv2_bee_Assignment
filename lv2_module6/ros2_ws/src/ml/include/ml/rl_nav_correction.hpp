#pragma once

// rl_nav_correction (🛠 직접 작성, 담당: 송정혁) — 설계: documents/project/ros/06-base.md, 11-learn.md
//   입력: /cmd_vel_nav (Nav2, 20Hz), /plan (경로 오차), /odom·/imu (실제 움직임), /speed_limit (재클리핑 상한)
//   출력: /cmd_vel_corrected (geometry_msgs/TwistStamped) → velocity_smoother
//   서비스: ~/toggle (std_srvs/SetBool) — ONNX 보정 켜기/끄기 (파라미터 enabled와 같은 값)
// ONNX 정책 (Isaac Lab/rsl_rl 학습, ONNX Runtime C++):
//   관측 6 = [v_cmd, w_cmd, v_odom, w_odom, w_gyro, t_change]  (track 모드 — 학습 때와 같은 순서·단위)
//     v_cmd·w_cmd = /cmd_vel_nav, v_odom·w_odom = /odom twist, w_gyro = /imu angular_velocity.z
//     t_change = 명령이 마지막으로 바뀐 뒤 경과 [s] (t_change_max로 자름)
//   출력 2 → clamp [-1,1] × (action_scale_lin, action_scale_ang) = Δ [m/s, rad/s]
//   Δ는 max_delta_lin·max_delta_ang로 한 번 더 자르고, 더한 뒤 min(프리셋 상한, /speed_limit)으로 재클리핑.
//   model_path가 비었거나 파일이 없거나 obs 차원이 6이 아니면 Δ=0 (통과) — 로봇은 계속 움직인다.
//   enabled=false · 센서(timeout_s=0.2s) 끊김도 Δ=0.
// 모델 경로: 상대 경로면 share/ml/model/<경로> 하나만 본다 (설치 공간이 단일 진실 공급원).
//   "nav_correction/policy.onnx"처럼 노드별 폴더 포함 — 노드마다 onnx를 가져와도 이름이 안 겹친다.
//   기본값 "nav_correction/policy.onnx" (= share/ml/model/nav_correction/policy.onnx). 직접 지정은 절대 경로로.
#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "ml/onnx_policy.hpp"
#include "nav2_msgs/msg/speed_limit.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace ml {

using geometry_msgs::msg::TwistStamped;

// 정책 출력 [-1,1] → 물리 보정량 (학습 env와 같은 clamp × action_scale)
inline std::pair<double, double> scale_action(float o0, float o1, double scale_lin, double scale_ang) {
  return {std::clamp(double(o0), -1.0, 1.0) * scale_lin, std::clamp(double(o1), -1.0, 1.0) * scale_ang};
}

// 보정을 더하고 자른다. 0 명령은 0 그대로 (정지 명령에 Δ를 주지 않는다)
inline std::pair<double, double> apply_delta(
  std::pair<double, double> cmd, std::pair<double, double> delta, double max_lin, double max_ang) {
  if (cmd.first == 0.0 && cmd.second == 0.0) return {0.0, 0.0};
  return {std::clamp(cmd.first + delta.first, -max_lin, max_lin),
          std::clamp(cmd.second + delta.second, -max_ang, max_ang)};
}

class RlNavCorrection : public rclcpp::Node {
public:
  RlNavCorrection();

private:
  void on_cmd(const TwistStamped & in);
  bool load_model(const std::string & path);  // true = 로드됨, false = 통과 모드
  std::string resolve_model_path(const std::string & path) const;
  void on_toggle(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                 std::shared_ptr<std_srvs::srv::SetBool::Response> res);

  double max_v_, max_w_, timeout_s_, limit_v_;
  double action_scale_lin_ = 0.05, action_scale_ang_ = 0.10;
  double max_delta_lin_ = 0.05, max_delta_ang_ = 0.10;
  double t_change_max_ = 2.0;
  int64_t obs_dim_ = 6;  // 기대 관측 차원 (모델에서 읽은 값으로 갱신)
  bool model_ok_ = false;
  std::string model_path_;  // 실제로 연 경로 (진단 표시)
  OnnxPolicy policy_;
  rclcpp::Time last_sensor_{0, 0, RCL_ROS_TIME};
  std::pair<double, double> last_cmd_{0.0, 0.0};
  rclcpp::Time cmd_changed_{0, 0, RCL_ROS_TIME};
  nav_msgs::msg::Path::ConstSharedPtr plan_;
  nav_msgs::msg::Odometry::ConstSharedPtr odom_;
  sensor_msgs::msg::Imu::ConstSharedPtr imu_;
  rclcpp::Subscription<TwistStamped>::SharedPtr cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav2_msgs::msg::SpeedLimit>::SharedPtr limit_sub_;
  rclcpp::Publisher<TwistStamped>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr debug_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr toggle_srv_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr post_cb_;
  size_t corrected_ = 0, infer_fail_ = 0;
  double last_delta_v_ = 0.0, last_delta_w_ = 0.0;
  diagnostic_updater::Updater diag_{this};
};

}  // namespace ml
