#pragma once
// 진척 감시 — 남은 경로 거리(Nav2 NavigateToPose feedback distance_remaining)가 window_s 동안 min_gain_m만큼도
//   줄지 않으면 갇힌 것으로 봄. Nav2 progress checker는 회복 동작(후진·회전·대기)마다 다시 시작해 벽 앞에서
//   "진척 없음 → 회복 → 재계획"을 몇 분씩 반복했음 (11-10 시뮬). 이건 회복과 상관없이 목표까지 남은 거리만 봄.
//   재계획으로 남은 거리가 늘면(우회로) 기준만 새 값으로 — 시계는 그대로 (우회로는 곧 줄기 시작, 갇힌 로봇은 안 줄음)
#include <cmath>
#include <limits>

namespace skills {

class ProgressWatch {
public:
  ProgressWatch(double window_s = 25.0, double min_gain_m = 0.10) : window_s_(window_s), min_gain_m_(min_gain_m) {}
  // t [s] · remaining [m] → true면 갇힘 (window_s 동안 min_gain_m 못 줄임)
  bool update(double t, double remaining) {
    if (since_ < 0.0) { best_ = remaining; since_ = t; return false; }
    if (remaining <= best_ - min_gain_m_) { best_ = remaining; since_ = t; return false; }
    if (remaining > best_ + 0.3) best_ = remaining;  // 재계획 우회로 — 기준만 올림 (시계는 그대로)
    return t - since_ > window_s_;
  }
  double stalled_for(double t) const { return since_ < 0.0 ? 0.0 : t - since_; }

private:
  double window_s_, min_gain_m_;
  double best_ = std::numeric_limits<double>::infinity();
  double since_ = -1.0;
};

// 멈춤 감시 (11-16) — 로봇 자세가 window_s 동안 move_m · move_rad 넘게 안 바뀌면 멈춘 것으로 봄.
//   11-16 실기 nav_1116_062046: collision_monitor가 세운 뒤 8~11 s 그대로 서 있다가 ProgressWatch(12 s)로 끼임 풀기 —
//   남은 거리만 보면 제자리 회전(정상)과 구분이 안 돼 창을 줄일 수 없음 → 자세(위치 + 방향)로 따로 봄. 회전 중이면 안 걸림
class StillWatch {
public:
  StillWatch(double window_s = 3.0, double move_m = 0.02, double move_rad = 0.05)
  : window_s_(window_s), move_m_(move_m), move_rad_(move_rad) {}
  // t [s] · 자세 → true면 멈춤 (window_s ≤ 0이면 감시 안 함)
  bool update(double t, double x, double y, double yaw) {
    if (window_s_ <= 0.0) return false;
    const double dyaw = std::abs(std::remainder(yaw - yaw0_, 2 * M_PI));
    if (since_ < 0.0 || std::hypot(x - x0_, y - y0_) > move_m_ || dyaw > move_rad_) {
      x0_ = x; y0_ = y; yaw0_ = yaw; since_ = t;
      return false;
    }
    return t - since_ > window_s_;
  }

private:
  double window_s_, move_m_, move_rad_;
  double x0_ = 0.0, y0_ = 0.0, yaw0_ = 0.0;
  double since_ = -1.0;
};

}  // namespace skills
