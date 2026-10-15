#pragma once

// ML 대역 (faker) — 도커·CI에는 GPU·학습된 모델이 없으므로, ml 패키지의 노드와 같은 이름·토픽·서비스로 대신 돈다.
//   grasp_scorer      📞 /score_grasps (ml/srv/ScoreGrasps) — 대상 물체 위치에 고정 후보 3개 (규칙 점수)
//   rl_nav_tuner      ✉️ /speed_limit — 프리셋 A(100%) 고정, controller_server 파라미터는 안 건드림
//   rl_nav_correction ✉️ /cmd_vel_nav → /cmd_vel_corrected — Δ=0, /speed_limit 상한으로만 자름
//   rl_arm_correction → 컨트롤러 플러그인 faker/PassthroughArmCorrection (passthrough_arm_correction.hpp)
// 모두 /diagnostics에 "대역(fake)"으로 상태를 낸다. 진짜 ML 노드는 ros2 launch faker faker.launch.py ml:=real
#include <algorithm>
#include <string>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "ml/srv/score_grasps.hpp"
#include "nav2_msgs/msg/speed_limit.hpp"
#include "perception/msg/tracked_object_array.hpp"
#include "rclcpp/rclcpp.hpp"

namespace faker {

class FakeGraspScorer : public rclcpp::Node {
public:
  FakeGraspScorer();

private:
  void score(const ml::srv::ScoreGrasps::Request & req, ml::srv::ScoreGrasps::Response & res);

  perception::msg::TrackedObjectArray::ConstSharedPtr objects_;
  size_t requests_ = 0;
  rclcpp::Subscription<perception::msg::TrackedObjectArray>::SharedPtr objects_sub_;
  rclcpp::Service<ml::srv::ScoreGrasps>::SharedPtr srv_;
  diagnostic_updater::Updater diag_{this};
};

class FakeNavTuner : public rclcpp::Node {
public:
  FakeNavTuner();

private:
  rclcpp::Publisher<nav2_msgs::msg::SpeedLimit>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  diagnostic_updater::Updater diag_{this};
};

class FakeNavCorrection : public rclcpp::Node {
public:
  FakeNavCorrection();

private:
  void on_cmd(const geometry_msgs::msg::TwistStamped & in);

  double max_v_, limit_v_;
  size_t passed_ = 0;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_sub_;
  rclcpp::Subscription<nav2_msgs::msg::SpeedLimit>::SharedPtr limit_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_;
  diagnostic_updater::Updater diag_{this};
};

}  // namespace faker
