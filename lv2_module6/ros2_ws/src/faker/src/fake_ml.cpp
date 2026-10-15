// fake_ml 구현 — 설명·선언은 include/faker/fake_ml.hpp
#include "faker/fake_ml.hpp"

namespace faker {

using diagnostic_msgs::msg::DiagnosticStatus;

FakeGraspScorer::FakeGraspScorer()
: rclcpp::Node("grasp_scorer") {
  objects_sub_ = create_subscription<perception::msg::TrackedObjectArray>("/tracked_objects", rclcpp::QoS(5).reliable(),
    [this](perception::msg::TrackedObjectArray::ConstSharedPtr m) { objects_ = m; });
  srv_ = create_service<ml::srv::ScoreGrasps>("/score_grasps",
    [this](const ml::srv::ScoreGrasps::Request::SharedPtr req, ml::srv::ScoreGrasps::Response::SharedPtr res) {
      score(*req, *res);
    });
  diag_.setHardwareID("beebot");
  diag_.add("상태", [this](diagnostic_updater::DiagnosticStatusWrapper & s) {
    s.summary(DiagnosticStatus::OK, "대역(fake) — 고정 후보");
    s.add("요청 수", requests_);
  });
}

void FakeGraspScorer::score(const ml::srv::ScoreGrasps::Request & req, ml::srv::ScoreGrasps::Response & res) {
  ++requests_;
  const perception::msg::TrackedObject * target = nullptr;
  if (objects_) {
    for (const auto & o : objects_->objects) if (o.id == req.object_id) target = &o;
  }
  if (!target) {
    res.success = false;
    res.message = "object " + std::to_string(req.object_id) + " not in /tracked_objects";
    return;
  }
  const float scores[] = {0.87f, 0.62f, 0.41f};
  const size_t n = std::min<size_t>(3, req.max_candidates ? req.max_candidates : 3);
  for (size_t i = 0; i < n; ++i) {
    geometry_msgs::msg::PoseStamped p;
    p.header.frame_id = "map";
    p.header.stamp = now();
    p.pose = target->pose.pose;
    res.grasps.push_back(p);
    res.scores.push_back(scores[i]);
  }
  res.success = true;
  res.message = "fake";
}

FakeNavTuner::FakeNavTuner()
: rclcpp::Node("rl_nav_tuner") {
  const double pct = declare_parameter<double>("speed_percent", 100.0);
  pub_ = create_publisher<nav2_msgs::msg::SpeedLimit>("/speed_limit", rclcpp::QoS(1).reliable());
  timer_ = create_wall_timer(std::chrono::seconds(2), [this, pct]() {
    nav2_msgs::msg::SpeedLimit m;
    m.header.stamp = now();
    m.percentage = true;
    m.speed_limit = pct;
    pub_->publish(m);
  });
  diag_.setHardwareID("beebot");
  diag_.add("상태", [pct](diagnostic_updater::DiagnosticStatusWrapper & s) {
    s.summary(DiagnosticStatus::OK, "대역(fake) — 프리셋 고정");
    s.add("속도 상한 [%]", pct);
  });
}

FakeNavCorrection::FakeNavCorrection()
: rclcpp::Node("rl_nav_correction") {
  max_v_ = declare_parameter<double>("max_vel_x", 0.26);
  limit_v_ = max_v_;
  cmd_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>("/cmd_vel_nav", 10,
    [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr m) { on_cmd(*m); });
  limit_sub_ = create_subscription<nav2_msgs::msg::SpeedLimit>("/speed_limit", rclcpp::QoS(1).reliable(),
    [this](nav2_msgs::msg::SpeedLimit::ConstSharedPtr m) {
      const double v = m->percentage ? max_v_ * m->speed_limit / 100.0 : m->speed_limit;
      limit_v_ = (m->speed_limit <= 0.0) ? max_v_ : std::min(max_v_, v);
    });
  pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel_corrected", rclcpp::QoS(1).reliable());
  diag_.setHardwareID("beebot");
  diag_.add("상태", [this](diagnostic_updater::DiagnosticStatusWrapper & s) {
    s.summary(DiagnosticStatus::OK, "대역(fake) — Δ=0 통과");
    s.add("통과한 명령 수", passed_);
    s.add("속도 상한 [m/s]", limit_v_);
  });
}

void FakeNavCorrection::on_cmd(const geometry_msgs::msg::TwistStamped & in) {
  auto out = in;
  out.twist.linear.x = std::clamp(in.twist.linear.x, -limit_v_, limit_v_);
  pub_->publish(out);
  ++passed_;
}

}  // namespace faker
