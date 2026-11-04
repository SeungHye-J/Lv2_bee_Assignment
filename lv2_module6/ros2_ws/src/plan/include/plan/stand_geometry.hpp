#pragma once

// 누운 퍽 세우기(stand)의 손끝 경로 — Isaac Lab right_env.plan_waypoints(mode "tilt")를 MoveIt 직교 경로용 자세로 옮긴 것
//   손은 내내 위에서 아래를 봄 (5축 팔은 상자 윗면 가까이에서 손을 수평으로 못 눕힘 — right_env 설명 R1).
//   퍽의 먼 쪽 끝 윗부분(상자: 손가락 중심 = 누운 퍽 윗면 높이 · 원기둥: 축 높이)을 살짝 잡고, 가까운 쪽 아래 모서리를 축으로
//   arc_end_deg까지 호를 그리며 tilt_over_m만큼 더 올림 (손이 앞서야 살짝 잡힌 퍽이 따라옴) → 잠깐 멈춤 →
//   같은 호로 90°(선 자세)까지 돌아와 set_dz_m 아래로 내려놓음 → (벌림) → lift_away_m 들어 올림
//   좌표: 퍽·로봇과 같은 좌표계(map). 손끝 = end_effector_link (손가락 닿는 가운데보다 ee_up 위)
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "perception/msg/tracked_object.hpp"

namespace plan {

struct StandParams {
  double puck_l = 0.060;          // [m] 퍽 길이 (누우면 긴 축)
  double puck_w = 0.030;          // [m] 퍽 폭 (누우면 높이)
  double contact_h_box = 0.030;   // [m] 사각기둥: 손가락 중심 높이 (상자 윗면 위) = 누운 퍽 윗면 → 위쪽 절반만 잡음
  double contact_h_cyl = 0.015;   // [m] 원기둥: 축 높이 (축에서 벗어나면 비스듬히 매달림)
  double pinch_from_end = 0.0;    // [m] 손가락 중심선 ~ 먼 쪽 끝면 (0 = 끝면 위 — 손가락 폭 절반만 걸침)
  double ee_up = 0.0106;          // [m] 손끝(end_effector_link) ~ 손가락 닿는 가운데 높이 차 (bee_common GRASP_Z − PUCK_H + GRIP_DEPTH/2)
  double above_m = 0.04;          // [m] 잡기 전 위에서 기다리는 높이
  double lift_away_m = 0.08;      // [m] 놓은 뒤 들어 올림 (선 퍽에서 손 빼기)
  double tilt_over_m = 0.010;     // [m] 호 끝까지 더 올림
  double arc_end_deg = 120.0;     // [deg] 선 자세(90°)를 넘겨 돌림 — 살짝 잡힌 퍽이 손보다 늦게 돎
  double set_dz_m = -0.010;       // [m] 내려놓는 점 (선 자세 잡은 점 기준, < 0 = 매달린 퍽을 바닥에 눌러 세움)
  double step_deg = 7.5;          // [deg] 호 점 간격 (관절 직선 보간이 원에서 0.5 mm 안)
  double drag_m = 0.0;            // [m] > 0이면 호 대신 잡은 채 로봇 쪽으로 이만큼 곧게 끌고 누르며 놓음 (퍽 밑면이 상자에 끌려 세워짐)
  double drag_lift_m = 0.0;       // [m] 끌면서 끝까지 서서히 올리는 양 (0 = 수평)
};

struct StandPath {
  double tool_yaw = 0.0;                     // 손 방향 ψ (손 = Rz(ψ)·Rx(π)) — 닫힘축이 퍽 긴 축에 직각
  geometry_msgs::msg::Pose above, pinch;     // 먼 쪽 끝 위 · 잡는 자세
  std::vector<geometry_msgs::msg::Pose> arc_up;    // 호 (0° 다음 점부터 arc_end까지)
  std::vector<geometry_msgs::msg::Pose> arc_back;  // 호를 90°까지 되돌아옴 + 내려놓는 점
  geometry_msgs::msg::Pose lift;                   // 벌린 뒤 들어 올림
};

// 위→아래 손 자세 Rz(ψ)·Rx(π) = (cos ψ/2, sin ψ/2, 0, 0) ↔ ψ. 그 꼴이 아니면 false (물체 yaw 쿼터니언 · 단위 등)
geometry_msgs::msg::Quaternion top_down_q(double psi);
bool top_down_yaw(const geometry_msgs::msg::Quaternion & q, double & psi);
// 퍽 방향 (perception) — details yaw_period가 있을 때만 앎. 누운 퍽 = 긴 축 (주기 180°)
bool object_yaw(const perception::msg::TrackedObject & o, double & yaw);
bool is_lying(const perception::msg::TrackedObject & o);
std::string detail(const perception::msg::TrackedObject & o, const std::string & key);  // 없으면 "" (vlm_task_planner.cpp에 정의)
// 긴 축 yaw(앞뒤 모름) → 로봇에서 멀어지는 쪽 (가까운 끝 → 먼 끝)
double far_axis_yaw(double axis_yaw, double cx, double cy, double rx, double ry);
// 퍽이 놓인 면의 높이 — 아래에 상자(start_box · location_box, 윗면 높이)가 있으면 그 윗면, 없으면 퍽 중심 − 반 높이
double support_top_z(const perception::msg::TrackedObject & puck, const std::vector<perception::msg::TrackedObject> & objects,
                     double puck_w);
// 받침 높이 맞추기 — 퍽은 바닥(0)이나 상자 윗면(0.07)에만 놓임(발제). 잰 값이 levels 중 가장 가까운 것에서 max_snap 안이면 그 값,
//   아니면 잰 값 그대로. 실기 11/03: 상자를 못 찾은 퍽은 perception z가 보이는 윗면 쪽이라 받침이 0.098로 2.8 cm 높게 나왔음
double snap_support(double z, const std::vector<double> & levels, double max_snap);
// 퍽 중심 (cx, cy) · 받침 윗면 높이 · 긴 축 yaw(가까운 끝 → 먼 끝) → 세우기 경로
StandPath stand_path(double cx, double cy, double support_z, double axis_yaw, bool is_box, const StandParams & p);

}  // namespace plan
