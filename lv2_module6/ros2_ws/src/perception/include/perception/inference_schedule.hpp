#pragma once

#include <cmath>
#include <limits>
#include <stdexcept>

namespace perception {

// Only image inference is throttled. Call under the perception frame mutex.
// 주행 중 인지 줄이기 (navigation_throttle) — /skills/navigate가 실행 중이면 navigation_hz로만 추론 (0 = 끔).
//   near_m > 0이면 Nav2 남은 거리(distance_remaining)가 near_m 안이거나 남은 거리를 feedback_timeout_s 넘게 모를 때는
//   평소 속도로 — 도착 직전 상자 측정(approach box_seed_s)이 남고, feedback이 끊겨도 인지가 꺼진 채로 남지 않음 (11-15)
class InferenceSchedule {
public:
  void configure(bool enabled, double navigation_hz, double near_m = 0.0, double feedback_timeout_s = 2.0) {
    if (!std::isfinite(navigation_hz) || navigation_hz < 0.0 || navigation_hz > 4.0)
      throw std::invalid_argument("navigation_infer_hz must be finite and in [0, 4] (0 = off while driving)");
    if (!std::isfinite(near_m) || near_m < 0.0 || !std::isfinite(feedback_timeout_s) || feedback_timeout_s <= 0.0)
      throw std::invalid_argument("navigation_full_near_m >= 0 and navigation_feedback_timeout_s > 0");
    enabled_ = enabled;
    navigation_interval_ = navigation_hz > 0.0 ? 1.0 / navigation_hz : std::numeric_limits<double>::infinity();
    near_m_ = near_m;
    feedback_timeout_s_ = feedback_timeout_s;
  }
  // action_msgs/GoalStatus: ACCEPTED, EXECUTING and CANCELING remain active.
  static bool active_status(int status) { return status >= 1 && status <= 3; }
  void navigation_active(bool active) {
    active_ = active;
    if (!active) remaining_t_ = -std::numeric_limits<double>::infinity();  // 다음 주행은 남은 거리를 다시 받을 때까지 켜 둠
  }
  // Nav2 NavigateToPose feedback distance_remaining [m] · 받은 시각 [s]
  void remaining(double distance_m, double now) {
    remaining_ = distance_m;
    remaining_t_ = now;
  }
  bool enabled() const { return enabled_; }
  bool throttled(double now) const {
    if (!enabled_ || !active_) return false;
    if (near_m_ <= 0.0) return true;                          // 예전 동작: 주행 중 내내
    if (now - remaining_t_ > feedback_timeout_s_) return false;  // 남은 거리를 모름(아직 · 끊김) → 평소대로
    return remaining_ > near_m_;
  }
  bool due(double now, double previous) const {
    // Retain the existing normal-mode gate; the 250ms timer caps it at 4Hz.
    if (!throttled(now)) return now - previous > 0.2;
    return std::isfinite(navigation_interval_) && now - previous >= navigation_interval_;
  }

private:
  bool enabled_ = false;
  bool active_ = false;
  double navigation_interval_ = 1.0;
  double near_m_ = 0.0, feedback_timeout_s_ = 2.0;
  double remaining_ = 0.0, remaining_t_ = -std::numeric_limits<double>::infinity();
};

}  // namespace perception
