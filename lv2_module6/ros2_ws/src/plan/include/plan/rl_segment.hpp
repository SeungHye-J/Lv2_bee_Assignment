#pragma once

// RL 정책 구간 — 학습(isaac/lab pick_env · place_env)과 데모(demo_cycle.py pick()·place())의 관절 시간표를 그대로.
//   잔차 정책은 이 시간표 위에서만 학습됨 (관측 = 계획 관절 q_above · q_down과 지금 기준 q_ref의 차 · 단계 · 단계 진행 시간).
//   단계: 0 이동+멈춤(hold) · 1 내려감 · 2 멈춤(settle) · 3 닫기(pick) / 놓기(place) · 4 올라감 · 5 기다림 · 6 끝
//   기준 q_ref(t): 이동 q0 → q_above (t_tr) · 내려감 q_above → q_down · 단계 3부터 q_down → q_above (t_ph[3]부터)
//   이동·내려감 시간 = 가장 많이 도는 관절 기준 사다리꼴 시간 (bee_common.trapezoid_duration, 팀 한계 × vel_scale), 곡선 = smooth step
#include <string>
#include <utility>
#include <vector>

namespace plan {

struct RlTimes {
  double hold = 0.3, settle = 0.3, act = 0.5, wait = 0.3;  // act = 닫기(pick 0.5 s) · 놓기(place 0.4 s)
  double vel_scale = 0.3;
};

struct RlTimeline {
  double t_tr = 0.0, t_mv = 0.0;
  std::vector<double> t_ph;  // 단계 끝 시각 [s] (누적, 6개) — 단계 = t_ph 중 지난 개수
};

double trapezoid_duration(double dist, double vel_scale);  // [rad] → [s] (VMAX 3 · AMAX 5 rad/s², 최소 0.1 s)
double move_s(double t, double dur);                       // 0..1 smooth step
RlTimeline rl_timeline(const std::vector<double> & q0, const std::vector<double> & q_above,
                       const std::vector<double> & q_down, const RlTimes & times);
std::vector<double> rl_ref(double t, const std::vector<double> & q0, const std::vector<double> & q_above,
                           const std::vector<double> & q_down, const RlTimeline & tl);
int rl_phase(double t, const RlTimeline & tl);
// 관절 경유점 pts[0] → … → pts[n]를 한 번의 움직임으로: 전체 관절 경로 길이(구간마다 가장 많이 도는 관절의 합)에 사다리꼴 시간
//   (trapezoid_duration · smooth step), 그 사이는 경유점 직선 보간 — right_env.arc_q와 같은 방식. dt마다 (시각, 관절값), 첫 점(t = 0) 제외
std::vector<std::pair<double, std::vector<double>>> joint_path(const std::vector<std::vector<double>> & pts, double vel_scale,
                                                               double dt = 0.02, double min_s = 0.5);
// 정책 노드(ml/rl_policy_node)에 주는 구간 정보 (JSON) — skill "" = 끝
std::string rl_context_json(const std::string & skill, double t0, const RlTimeline & tl, const std::vector<double> & q_above,
                            const std::vector<double> & q_down, int target_id, bool is_box, double tool_yaw,
                            const std::vector<double> & cell);

}  // namespace plan
