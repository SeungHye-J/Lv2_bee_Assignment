#include "skills/arm_state_cache.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>

namespace skills {
namespace {
std::mutex registry_mutex;
std::unordered_map<const rclcpp::Context *, std::weak_ptr<ArmStateCache>> registry;
}

std::shared_ptr<ArmStateCache> ArmStateCache::for_node(rclcpp::Node & node) {
  // The subscriptions belong to the first node that creates this cache. skills.launch.py
  // statically loads all components for the container lifetime; if individual component
  // unloading is introduced, move cache ownership to a container-lifetime node.
  const auto context = node.get_node_base_interface()->get_context();
  std::lock_guard<std::mutex> lock(registry_mutex);
  auto & entry = registry[context.get()];
  if (auto shared = entry.lock()) return shared;
  auto shared = std::shared_ptr<ArmStateCache>(new ArmStateCache(node));
  entry = shared;
  return shared;
}

ArmStateCache::ArmStateCache(rclcpp::Node & node)
: clock_(node.get_clock()), tf_buffer_(std::make_shared<tf2_ros::Buffer>(clock_)) {
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  joints_sub_ = node.create_subscription<sensor_msgs::msg::JointState>("/joint_states", rclcpp::QoS(10),
    [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) { on_joint_state(*msg); });
  debug_sub_ = node.create_subscription<ml::msg::ArmCorrectionDebug>("/rl_arm_correction/debug", rclcpp::QoS(10),
    [this](ml::msg::ArmCorrectionDebug::ConstSharedPtr msg) { on_debug(*msg); });
}

void ArmStateCache::on_joint_state(const sensor_msgs::msg::JointState & msg) {
  const double now = clock_->now().seconds();
  std::lock_guard<std::mutex> lock(mutex_);
  for (size_t i = 0; i < msg.name.size() && i < msg.position.size(); ++i) pos_[msg.name[i]] = msg.position[i];
  for (size_t i = 0; i < msg.name.size() && i < msg.effort.size(); ++i) effort_[msg.name[i]] = msg.effort[i];
  const auto e2 = effort_.find("joint2"), e3 = effort_.find("joint3");
  if (e2 != effort_.end() && e3 != effort_.end()) {
    jam_hist_.emplace_back(now, e2->second + e3->second);
    while (!jam_hist_.empty() && jam_hist_.front().first < now - 1.0) jam_hist_.pop_front();
  }
}

void ArmStateCache::on_debug(const ml::msg::ArmCorrectionDebug & msg) {
  const double now = clock_->now().seconds();
  if (msg.delta.size() < 5) return;
  std::array<double, 5> delta{};
  std::copy_n(msg.delta.begin(), delta.size(), delta.begin());
  std::lock_guard<std::mutex> lock(mutex_);
  delta_hist_.emplace_back(now, delta);
  while (!delta_hist_.empty() && delta_hist_.front().first < now - 1.0) delta_hist_.pop_front();
}

bool ArmStateCache::position(const std::string & joint_name, double & value) const {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = pos_.find(joint_name);
  if (it == pos_.end()) return false;
  value = it->second;
  return true;
}

bool ArmStateCache::joint(const std::string & joint_name, double & position_value, double & effort_value) const {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto p = pos_.find(joint_name);
  if (p == pos_.end()) return false;
  position_value = p->second;
  const auto e = effort_.find(joint_name);
  effort_value = e == effort_.end() ? std::numeric_limits<double>::quiet_NaN() : e->second;
  return true;
}

double ArmStateCache::jam_mean(double window_s) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (jam_hist_.empty()) return 0.0;
  const double t0 = jam_hist_.back().first - window_s;
  double sum = 0.0;
  int n = 0;
  for (const auto & [time, effort] : jam_hist_) {
    if (time >= t0) { sum += effort; ++n; }
  }
  return n ? sum / n : 0.0;
}

bool ArmStateCache::delta_stable(double now, double fresh_s, double window_s,
                                 double max_spread, size_t min_samples) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (delta_hist_.empty() || delta_hist_.back().first <= now - fresh_s) return true;
  auto lo = delta_hist_.back().second, hi = lo;
  size_t count = 0;
  for (const auto & [time, delta] : delta_hist_) {
    if (time < now - window_s) continue;
    ++count;
    for (size_t i = 0; i < delta.size(); ++i) {
      lo[i] = std::min(lo[i], delta[i]);
      hi[i] = std::max(hi[i], delta[i]);
    }
  }
  for (size_t i = 0; i < lo.size(); ++i) {
    if (hi[i] - lo[i] > max_spread) return false;
  }
  return count >= min_samples;
}

}  // namespace skills
