#pragma once

// hand_relative 계산 (ROS 없음, test/test_hand_geom.cpp) — scripts/hand_geom.py의 C++판 (11-16). 뜻·정의는 그 머리말과 같음:
//   시뮬 학습 정책이 보던 손 카메라 상대값 · 모두 base_link 좌표 [m]
//   puck : 퍽 가운데 − 손끝 · cell : 칸 가운데 − 손 안 퍽 바닥 · slot : 적재함 칸 가운데 − 손 안 퍽 바닥 · face_yaw : 사각 퍽 면 − 손 방향 (±45°)
//   손 안 퍽 바닥 = 손끝 + R_손 · (0, 0, GRASP_Z) — 손 안 오차는 0으로 둠
#include <array>
#include <cmath>
#include <optional>
#include <string>
#include <vector>

namespace perception::hand_geom {

using V3 = std::array<double, 3>;
using M3 = std::array<std::array<double, 3>, 3>;  // [행][열]

constexpr double GRASP_Z = 0.06 - 0.020 + 0.0206;  // 퍽 바닥이 손끝 아래 이만큼 (isaac/rl/bee_isaac/common.py GRASP_Z)
constexpr double PUCK_H = 0.06;

inline V3 add(const V3 & a, const V3 & b) { return {a[0] + b[0], a[1] + b[1], a[2] + b[2]}; }
inline V3 sub(const V3 & a, const V3 & b) { return {a[0] - b[0], a[1] - b[1], a[2] - b[2]}; }
inline V3 neg(const V3 & a) { return {-a[0], -a[1], -a[2]}; }
inline V3 mul(const M3 & R, const V3 & v) {
  return {R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2], R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2],
          R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2]};
}
inline V3 mul_t(const M3 & R, const V3 & v) {  // Rᵀ v
  return {R[0][0] * v[0] + R[1][0] * v[1] + R[2][0] * v[2], R[0][1] * v[0] + R[1][1] * v[1] + R[2][1] * v[2],
          R[0][2] * v[0] + R[1][2] * v[1] + R[2][2] * v[2]};
}

inline M3 quat_to_rot(double x, double y, double z, double w) {
  return {{{1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)},
           {2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)},
           {2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)}}};
}

// (a + period/2) mod period − period/2 — 파이썬 % (나머지 부호 = period 부호)와 같게
inline double wrap(double a, double period) {
  double m = std::fmod(a + period / 2, period);
  if (m != 0.0) {
    if ((period < 0) != (m < 0)) m += period;
  } else {
    m = std::copysign(0.0, period);
  }
  return m - period / 2;
}

// map 점 → base_link (base_t, base_R = map에서 본 base_link 위치·회전)
inline V3 to_base(const V3 & p_map, const V3 & base_t, const M3 & base_R) { return mul_t(base_R, sub(p_map, base_t)); }

// 손 방향 yaw (손끝 z가 아래일 때 손 x축의 바닥 위 방향) — 시뮬 rot = Rz(yaw) · DOWN 의 yaw
inline double tool_yaw(const M3 & ee_R) { return std::atan2(ee_R[1][0], ee_R[0][0]); }

// 손 안 퍽 바닥 (base_link) — 정상 파지
inline V3 bottom_in_hand(const V3 & ee_t, const M3 & ee_R) { return add(ee_t, mul(ee_R, {0.0, 0.0, GRASP_Z})); }

// 검출 위치 → 퍽 가운데 (반 높이). 손 카메라가 위에서 본 서 있는 퍽은 윗면 점이라 반 높이 내림
inline V3 puck_centre(V3 p, bool standing, bool seen_from_top = true) {
  if (standing && seen_from_top) p[2] -= PUCK_H / 2;
  return p;
}

// 사각 퍽 면 − 손 방향 (±45°) · 원기둥·방향 모름은 0
inline double face_yaw(std::optional<double> puck_yaw_b, double tool_yaw_b, bool is_box) {
  if (!is_box || !puck_yaw_b) return 0.0;
  return wrap(*puck_yaw_b - tool_yaw_b, M_PI / 2);
}

// 퍽 축 (아래 → 위, 누운 퍽은 가까운 끝 → 먼 끝) — 누운 퍽 yaw는 180° 대칭 → 로봇에서 먼 쪽
inline V3 puck_axis(bool standing, std::optional<double> yaw_b, const V3 & centre_b) {
  if (standing || !yaw_b) return {0.0, 0.0, 1.0};
  const V3 a{std::cos(*yaw_b), std::sin(*yaw_b), 0.0};
  return a[0] * centre_b[0] + a[1] * centre_b[1] >= 0 ? a : neg(a);
}

// 계획 때 칸 가운데 + (지금 본 상자 − 계획 때 상자) — 상자가 밀렸으면 따라감 (mission_solver tag_correction과 같은 한도)
inline V3 cell_now(const V3 & point_map, const std::optional<V3> & box_plan_map, const std::optional<V3> & box_seen_map,
                   double max_shift = 0.05) {
  if (!box_seen_map || !box_plan_map) return point_map;
  const double dx = (*box_seen_map)[0] - (*box_plan_map)[0], dy = (*box_seen_map)[1] - (*box_plan_map)[1];
  if (std::hypot(dx, dy) > max_shift) return point_map;
  return {point_map[0] + dx, point_map[1] + dy, point_map[2]};
}

struct Puck {
  int tag;
  V3 centre_b;  // base_link
};

// 목표 퍽 번호 (tag ≥ 0이면 그 번호, 없으면 손에 가장 가까운 것 — 같으면 앞 것) · 없으면 nullopt
inline std::optional<size_t> pick_target(const std::vector<Puck> & pucks, const V3 & ee_t, int tag = -1) {
  if (pucks.empty()) return std::nullopt;
  if (tag >= 0) {
    for (size_t k = 0; k < pucks.size(); ++k)
      if (pucks[k].tag == tag) return k;
    return std::nullopt;
  }
  size_t best = 0;
  double best_d = 0.0;
  for (size_t k = 0; k < pucks.size(); ++k) {
    const double d = std::hypot(pucks[k].centre_b[0] - ee_t[0], pucks[k].centre_b[1] - ee_t[1]);
    if (k == 0 || d < best_d) best = k, best_d = d;
  }
  return best;
}

// 목표 퍽 k 옆(같은 상자, radius 안) 가장 가까운 다른 퍽 − 목표 [m] · 없으면 nullopt
inline std::optional<V3> neighbour(const std::vector<Puck> & pucks, size_t k, double radius = 0.12) {
  std::optional<V3> best;
  for (size_t j = 0; j < pucks.size(); ++j) {
    if (j == k) continue;
    const V3 d = sub(pucks[j].centre_b, pucks[k].centre_b);
    const double r = std::hypot(d[0], d[1]);
    if (r < radius && (!best || r < std::hypot((*best)[0], (*best)[1]))) best = d;
  }
  return best;
}

// 마지막 검출(map) — kind "puck" (c_map = 퍽 가운데, standing, yaw_map 또는 없음, is_box) 또는 "cell" (c_map = 칸 점)
struct Held {
  std::string kind;
  V3 c_map{};
  bool standing = false;
  std::optional<double> yaw_map;
  bool is_box = false;
};

struct Relative {
  V3 rel;
  double face_yaw;
  V3 axis;
};

// 마지막 검출(map)을 지금 손에서 본 값 → (rel, face_yaw, axis) base_link — 시뮬 정책 입력 (isaac pick_env: last_m − 지금 손)
inline Relative held_relative(const Held & held, const V3 & base_t, const M3 & base_R, const V3 & ee_t, const M3 & ee_R) {
  const V3 c_b = to_base(held.c_map, base_t, base_R);
  if (held.kind == "cell") return {sub(c_b, bottom_in_hand(ee_t, ee_R)), 0.0, {0.0, 0.0, 1.0}};
  const double base_yaw = std::atan2(base_R[1][0], base_R[0][0]);
  std::optional<double> yaw_b;
  if (held.yaw_map) yaw_b = wrap(*held.yaw_map - base_yaw, 2 * M_PI);
  return {sub(c_b, ee_t), face_yaw(yaw_b, tool_yaw(ee_R), held.is_box), puck_axis(held.standing, yaw_b, c_b)};
}

// pick_skill 손끝 목표(서 있는 퍽을 위에서 집는 자세) → 계획이 겨냥한 퍽 가운데 (map) — 시뮬 pick_env tgt_insp_b
inline V3 aimed_centre(const V3 & grasp_ee_map) { return sub(grasp_ee_map, {0.0, 0.0, GRASP_Z - PUCK_H / 2}); }

// unload: 칸 안 퍽 공칭 가운데 (칸 바닥 + 반 높이) − 손끝 (base_link) — 시뮬 UnloadEnv slot_tf
inline V3 slot_puck_rel(const V3 & slot_floor_b, const V3 & ee_t) {
  return sub(add(slot_floor_b, {0.0, 0.0, PUCK_H / 2}), ee_t);
}

}  // namespace perception::hand_geom
