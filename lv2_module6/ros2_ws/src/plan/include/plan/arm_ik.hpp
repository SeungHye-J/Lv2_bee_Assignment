#pragma once

// 팔(joint1~5) 정기구학 · 위→아래 손 자세 IK — arm_base_link 기준 end_effector_link.
//   MoveIt IK(pick_ik)는 kinematics.yaml rotation_scale 0(5축이라 위치만)이라 손 방향을 보장하지 않는다 → 세우기처럼 손이 내내
//   아래를 봐야 하는 경로는 여기서 푼다 (학습 isaac/lab bee_common.ArmIK = pinocchio로 위치 + 회전 둘 다 맞춘 것과 같은 목표).
//   관절 원점·축 = urdf/default/urdf.xacro (joint1~5 · end_effector_joint). 관절 영점 offset은 0 (BeebotSystemHardware offset 기본).
//   풀이: 감쇠 최소제곱(Levenberg–Marquardt), 앞 점의 해를 씨앗으로 — 이웃 점끼리 해가 이어짐
#include <array>
#include <vector>

namespace plan {

using Mat4 = std::array<double, 16>;  // 행 우선 4×4

Mat4 arm_fk(const std::vector<double> & q);  // joint1~5 → arm_base_link ← end_effector_link
// 목표: 위치 p(arm_base_link) · 손 = Rz(psi)·Rx(π) (손 z축이 아래, 닫힘축 = ψ − 90°). 위치 오차 pos_tol [m] · 회전 오차 rot_tol [rad] 안이면 true
bool arm_ik_down(const std::array<double, 3> & p, double psi, const std::vector<double> & seed, std::vector<double> & q,
                 double pos_tol = 0.002, double rot_tol = 0.03);
// 회전 오차 [rad] — 지금 손 자세와 목표 Rz(psi)·Rx(π) 사이
double down_rot_err(const Mat4 & T, double psi);

}  // namespace plan
