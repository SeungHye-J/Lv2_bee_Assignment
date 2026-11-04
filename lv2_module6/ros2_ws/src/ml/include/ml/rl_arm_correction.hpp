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
// 중력 처짐 앞먹임 (gravity_enabled, 기본 꺼짐): 서보 엔코더가 못 보는 관절 비틀림(링크·기어 유격 — 손 쪽 무게가 치우쳐 생김)을
//   자세별로 미리 뺀다. 관절 i가 중력 토크 τ_i에 비례해 d_i = c_i·τ_i 만큼 비틀리므로 (c = 1/강성 [rad/Nm], 실측: isaac/sim/R2S/sag_fit.py)
//   command = reference + corr + Δ − d. τ_i는 현재 관절각(엔코더)에서 팔 평면 점질량 모델(gravity::torques, URDF 값)로 계산.
//   gravity_z_offset_m: 자세와 무관한 고정 높이 오차(실측 −5 mm)를 올리는 양 [m] — 손가락 끝면 중심을 x·손 피치 그대로 위로.
//   앞먹임 값은 오차(ref + Δ + 앞먹임 − q)에도 들어가서 처짐 보정 적분이 지우지 않는다. 켜고 끌 때는 gravity_ramp_s 동안 서서히. 관측: ~/gravity = [앞먹임×N].
#include <algorithm>
#include <array>
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

// 팔 평면(joint2~4, 피치 축)의 점질량 모델 — 값은 URDF(beebot.urdf), URDF가 바뀌면 여기를 갱신 (isaac/sim/R2S/arm_model.py가 같은 값)
namespace gravity {
constexpr double G = 9.81;
// 질량 [kg]·무게중심 (x, z) [m] — 각 링크 프레임 안. 손 = gripper_link + 손가락 + 카메라를 하나로 (gripper_link 프레임)
constexpr double M_UA = 0.128865, UA_X = 0.008487, UA_Z = 0.104997;  // upper_arm (joint2 뒤)
constexpr double M_FA = 0.117300, FA_X = 0.093796;                   // forearm (joint3 뒤)
constexpr double M_WR = 0.090152, WR_X = 0.043623, WR_Z = 0.011449;  // wrist (joint4 뒤)
constexpr double M_HD = 0.371242, HD_X = 0.052686, HD_Z = 0.014472;  // 손 (joint5 뒤, 롤은 무게중심이 축 위라 무시)
// 관절 위치: upper_arm 안 joint3, forearm 안 joint4, wrist 안 gripper_link 원점, gripper_link 안 손가락 끝면 중심
constexpr double J3_X = 0.024, J3_Z = 0.128, J4_X = 0.124, HAND_X = 0.0629;
constexpr double MARK_X = 0.109055 + 0.0206;  // end_effector_link 0.109055 + 거기서 면까지 0.0206 (plan/stand_plan.hpp FINGER_TIP)

struct V2 {
  double x, z;
};
// 팔 평면 안 벡터를 피치 th [rad]만큼 돌림 (θ=0: 앞 +x, θ=90°: 아래를 가리킴)
inline V2 rot(double th, double x, double z) {
  const double c = std::cos(th), s = std::sin(th);
  return {x * c + z * s, -x * s + z * c};
}

// joint2, 3, 4 중력 토크 [Nm] — 각 관절 축(피치 y) 둘레, 앞으로 처지는 쪽이 +. τ_j = g · Σ m_k (x_k − x_j)
inline std::array<double, 3> torques(double q2, double q3, double q4) {
  const double t23 = q2 + q3, t234 = t23 + q4;
  const V2 ua = rot(q2, UA_X, UA_Z), p3 = rot(q2, J3_X, J3_Z);
  const V2 fa = rot(t23, FA_X, 0.0), p4v = rot(t23, J4_X, 0.0);
  const double p4x = p3.x + p4v.x;
  const double wr = p4x + rot(t234, WR_X, WR_Z).x, hd = p4x + rot(t234, HAND_X + HD_X, HD_Z).x;
  const double fax = p3.x + fa.x;
  return {G * (M_UA * ua.x + M_FA * fax + M_WR * wr + M_HD * hd),
          G * (M_FA * (fax - p3.x) + M_WR * (wr - p3.x) + M_HD * (hd - p3.x)),
          G * (M_WR * (wr - p4x) + M_HD * (hd - p4x))};
}

// 손가락 끝면 중심을 dz [m] 위로 올리는 joint2~4 각도 변화 [rad] (x와 손 피치 그대로): 평면 야코비안 J·Δq = [0, dz, 0]. 풀 수 없으면 false
inline bool lift(double q2, double q3, double q4, double dz, std::array<double, 3> & dq) {
  const double t23 = q2 + q3, t234 = t23 + q4;
  const V2 a = rot(q2, J3_X, J3_Z), b = rot(t23, J4_X, 0.0), c = rot(t234, HAND_X + MARK_X, 0.0);
  // d(rot v)/dθ = (v.z, −v.x) (돌린 뒤 값 기준)
  const double j4x = c.z, j4z = -c.x;
  const double j3x = j4x + b.z, j3z = j4z - b.x;
  const double j2x = j3x + a.z, j2z = j3z - a.x;
  // 행: x, z, 피치(=1,1,1)  열: q2, q3, q4
  const double m[3][3] = {{j2x, j3x, j4x}, {j2z, j3z, j4z}, {1.0, 1.0, 1.0}};
  const double rhs[3] = {0.0, dz, 0.0};
  const auto det3 = [](const double r[3][3]) {
    return r[0][0] * (r[1][1] * r[2][2] - r[1][2] * r[2][1]) - r[0][1] * (r[1][0] * r[2][2] - r[1][2] * r[2][0]) +
           r[0][2] * (r[1][0] * r[2][1] - r[1][1] * r[2][0]);
  };
  const double d = det3(m);
  if (std::abs(d) < 1e-6) return false;
  for (int k = 0; k < 3; ++k) {  // 크래머: k번째 열을 rhs로 바꾼 행렬식 / d
    double t[3][3];
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j) t[i][j] = (j == k) ? rhs[i] : m[i][j];
    dq[k] = det3(t) / d;
  }
  return true;
}
}  // namespace gravity

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
  double grav_scale_ = 0.0;  // 중력 앞먹임 세기 0..1 (켜고 끌 때 서서히)
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr grav_raw_;
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>> grav_pub_;
};

}  // namespace ml
