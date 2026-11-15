// rewind C++판 (11-16) — scripts/rewind.py와 같음: 실패한 주행의 실제 궤적(odom)을 따라 0.4 m 후진
//   /navigate_to_pose·/navigate_through_poses 액션이 ABORTED 되면 자동 시작 · 수동: ros2 service call /rewind std_srvs/srv/Trigger
//   /cmd_vel_smoothed로 발행 → collision_monitor를 그대로 통과
#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <deque>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "action_msgs/msg/goal_status_array.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros_system/helpers.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace ros_system {

namespace {
// 파이썬 float repr과 같게 (가장 짧은 표기 · 정수면 .0) — 서비스 응답 'rewind to (x, y)'
std::string py_repr(double v) {
  char buf[64];
  auto [end, ec] = std::to_chars(buf, buf + sizeof(buf), v);
  std::string s(buf, end);
  if (s.find_first_of(".eEn") == std::string::npos) s += ".0";
  return s;
}
}  // namespace

class Rewind : public rclcpp::Node {
public:
  explicit Rewind(const rclcpp::NodeOptions & options) : Node("rewind", options) {
    // 11-16: Nav2 실패 때 자동 후진 켬/끔 — mission_manager가 실패하면 도면을 보고 직접 물러나는데(retreat) 이 노드도
    //   동시에 /cmd_vel_smoothed로 후진을 내서 두 명령이 섞여 L1 상자를 침 (시뮬). 끄면 수동(/rewind 서비스)만
    auto_on_abort_ = declare_parameter<bool>("rewind_auto_on_abort", true);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/odometry/filtered", 10,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m) { on_odom(*m); });
    for (const char * topic : {"/navigate_to_pose/_action/status", "/navigate_through_poses/_action/status"})
      status_subs_.push_back(create_subscription<action_msgs::msg::GoalStatusArray>(topic, 10,
        [this](action_msgs::msg::GoalStatusArray::ConstSharedPtr m) { on_status(*m); }));
    srv_ = create_service<std_srvs::srv::Trigger>("/rewind",
      [this](std_srvs::srv::Trigger::Request::ConstSharedPtr, std_srvs::srv::Trigger::Response::SharedPtr res) {
        const bool ok = start_rewind();
        res->success = ok;
        res->message = ok ? "rewind to (" + py_repr(target_[0]) + ", " + py_repr(target_[1]) + ")" : "no trajectory";
      });
    pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel_smoothed", 10);
    timer_ = create_timer(std::chrono::milliseconds(50), [this]() { tick(); });
    RCLCPP_INFO(get_logger(), "rewind ready — abort 시 자동 0.4 m 후진 · 수동 /rewind");
  }

private:
  using XY = std::array<double, 2>;
  static double dist(const XY & a, const XY & b) { return std::hypot(a[0] - b[0], a[1] - b[1]); }

  void on_odom(const nav_msgs::msg::Odometry & msg) {
    const auto & p = msg.pose.pose;
    const auto & q = p.orientation;
    odom_ = {p.position.x, p.position.y, std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))};
    have_odom_ = true;
    const XY xy{p.position.x, p.position.y};
    if (traj_.empty() || dist(xy, traj_.back()) > 0.01) {
      traj_.push_back(xy);
      if (traj_.size() > 4000) traj_.pop_front();  // 파이썬 deque(maxlen=4000)
    }
  }

  void on_status(const action_msgs::msg::GoalStatusArray & msg) {
    for (const auto & s : msg.status_list) {
      const auto & key = s.goal_info.goal_id.uuid;
      if (s.status == 1 || s.status == 2 || s.status == 3) {  // ACCEPTED / EXECUTING / CANCELING
        active_goals_.insert(key);
      } else if (s.status == 6 && active_goals_.count(key)) {  // ABORTED
        active_goals_.erase(key);
        if (auto_on_abort_) start_rewind();
      }
    }
  }

  bool start_rewind() {
    if (active_) return true;
    if (!have_odom_ || traj_.size() < 5) return false;
    const double x = odom_[0], y = odom_[1], yaw = odom_[2];
    double acc = 0.0;
    long idx = static_cast<long>(traj_.size()) - 1;
    while (idx > 0 && acc < 0.40) {
      acc += dist(traj_[idx], traj_[idx - 1]);
      --idx;
    }
    auto ex_of = [&](long i) { return std::cos(yaw) * (traj_[i][0] - x) + std::sin(yaw) * (traj_[i][1] - y); };
    double ex = ex_of(idx);
    while (idx > 0 && ex > -0.05) {  // 목표는 반드시 로봇 뒤쪽
      --idx;
      ex = ex_of(idx);
    }
    target_ = traj_[idx];
    start_xy_ = {x, y};
    t0_ = now().seconds();
    active_ = true;
    RCLCPP_INFO(get_logger(), "rewind start: idx=%ld/%zu target=(%.2f,%.2f) ex=%.2f", idx, traj_.size(), target_[0], target_[1], ex);
    return true;
  }

  void tick() {
    if (!active_) return;
    const double t = now().seconds();
    const double x = odom_[0], y = odom_[1], yaw = odom_[2];
    const double dx = target_[0] - x, dy = target_[1] - y;
    const double ex = std::cos(yaw) * dx + std::sin(yaw) * dy;
    const double ey = -std::sin(yaw) * dx + std::cos(yaw) * dy;
    const double dist_to_target = std::hypot(dx, dy);
    const double moved = dist({x, y}, start_xy_);
    const bool stop = dist_to_target < 0.06 || moved >= 0.45 || (t - t0_) > 8.0 || ex > 0.05;
    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = "base_footprint";
    if (stop) {
      RCLCPP_INFO(get_logger(), "rewind stop: target=%.2f moved=%.2f ex=%.2f", dist_to_target, moved, ex);
      active_ = false;
    } else {
      msg.twist.linear.x = -0.05;
      msg.twist.angular.z = std::clamp(-1.2 * std::atan2(ey, -ex), -0.8, 0.8);
    }
    pub_->publish(msg);
  }

  std::deque<XY> traj_;  // 실제 주행 궤적
  std::array<double, 3> odom_{};  // (x, y, yaw)
  bool have_odom_ = false, active_ = false;
  XY target_{}, start_xy_{};
  double t0_ = 0.0;
  std::set<std::array<uint8_t, 16>> active_goals_;
  bool auto_on_abort_ = true;  // Nav2 실패(ABORTED) 때 자동 후진 (rewind_auto_on_abort)
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::vector<rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr> status_subs_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

std::shared_ptr<rclcpp::Node> make_rewind(const rclcpp::NodeOptions & options) { return std::make_shared<Rewind>(options); }

}  // namespace ros_system
