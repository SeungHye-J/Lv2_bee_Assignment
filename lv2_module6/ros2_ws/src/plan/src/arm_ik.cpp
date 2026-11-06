// 팔 정기구학 · 위→아래 IK — 설명은 include/plan/arm_ik.hpp
#include "plan/arm_ik.hpp"

#include <algorithm>
#include <cmath>

namespace plan {

namespace {

struct Joint {
  double xyz[3], rpy[3], axis[3];  // 부모 → 이 관절 원점 (URDF origin) · 회전축 (그 원점 좌표)
};

// urdf/default/urdf.xacro — joint1~5, 마지막은 end_effector_joint (고정)
constexpr Joint ARM[6] = {
  {{0.0, 0.0, 0.0409}, {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}},
  {{0.0, -0.01875, 0.0416}, {1.5707963267945113, 0.0, 3.141592653375372}, {0.0, 0.0, 1.0}},
  {{-0.024, 0.128, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}},
  {{-0.124, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}},
  {{-0.06289999999598264, 0.0, 0.018750000013476882}, {-3.141592653589793, 1.570796326794886, 0.0}, {0.0, 0.0, 1.0}},
  {{0.0, 0.0, 0.109055}, {0.0, 0.0, 1.570796}, {0.0, 0.0, 0.0}},  // end_effector_joint (고정)
};
// URDF 관절 한계 (joint1~5)
constexpr double LO[5] = {-3.14159, -1.919862, -1.570796, -1.7, -3.141593};
constexpr double HI[5] = {3.14159, 1.919862, 1.396263, 1.97, 3.141593};

Mat4 eye() { return {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}; }

Mat4 mul(const Mat4 & a, const Mat4 & b) {
  Mat4 c{};
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j)
      for (int k = 0; k < 4; ++k) c[4 * i + j] += a[4 * i + k] * b[4 * k + j];
  return c;
}

// URDF origin: 이동 · 회전 R = Rz(yaw)·Ry(pitch)·Rx(roll)
Mat4 origin(const double xyz[3], const double rpy[3]) {
  const double cr = std::cos(rpy[0]), sr = std::sin(rpy[0]), cp = std::cos(rpy[1]), sp = std::sin(rpy[1]);
  const double cy = std::cos(rpy[2]), sy = std::sin(rpy[2]);
  return {cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, xyz[0],
          sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, xyz[1],
          -sp, cp * sr, cp * cr, xyz[2],
          0, 0, 0, 1};
}

Mat4 rot_z(double a) {
  const double c = std::cos(a), s = std::sin(a);
  return {c, -s, 0, 0, s, c, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
}

// 목표 손 회전 Rz(ψ)·Rx(π) — 열 = 손 x · y · z축 (arm_base_link 기준)
void down_rot(double psi, double R[9]) {
  const double c = std::cos(psi), s = std::sin(psi);
  const double r[9] = {c, s, 0, s, -c, 0, 0, 0, -1};
  std::copy(r, r + 9, R);
}

// 회전 오차 벡터 (목표 ← 지금, 축·각) — R_err = R_t · R_cᵀ 의 로그
void rot_err_vec(const Mat4 & T, const double Rt[9], double e[3]) {
  double Re[9];
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) {
      double v = 0;
      for (int k = 0; k < 3; ++k) v += Rt[3 * i + k] * T[4 * j + k];  // Rt · Rcᵀ
      Re[3 * i + j] = v;
    }
  const double tr = std::clamp((Re[0] + Re[4] + Re[8] - 1) / 2, -1.0, 1.0), ang = std::acos(tr);
  const double w[3] = {Re[7] - Re[5], Re[2] - Re[6], Re[3] - Re[1]};
  const double s = std::sin(ang);
  if (std::abs(s) < 1e-6) {
    if (ang < 1e-3) { e[0] = w[0] / 2; e[1] = w[1] / 2; e[2] = w[2] / 2; return; }
    // 180° 가까이 — 대각에서 축
    const double ax[3] = {std::sqrt(std::max(0.0, (Re[0] + 1) / 2)), std::sqrt(std::max(0.0, (Re[4] + 1) / 2)),
                          std::sqrt(std::max(0.0, (Re[8] + 1) / 2))};
    for (int i = 0; i < 3; ++i) e[i] = ax[i] * ang;
    return;
  }
  for (int i = 0; i < 3; ++i) e[i] = w[i] * ang / (2 * s);
}

// n×n 선형식 A x = b (가우스 소거, 부분 피벗) — n = 5
bool solve(std::vector<double> A, std::vector<double> b, int n, std::vector<double> & x) {
  for (int c = 0; c < n; ++c) {
    int p = c;
    for (int r = c + 1; r < n; ++r) if (std::abs(A[n * r + c]) > std::abs(A[n * p + c])) p = r;
    if (std::abs(A[n * p + c]) < 1e-12) return false;
    for (int k = 0; k < n; ++k) std::swap(A[n * c + k], A[n * p + k]);
    std::swap(b[c], b[p]);
    for (int r = c + 1; r < n; ++r) {
      const double f = A[n * r + c] / A[n * c + c];
      for (int k = c; k < n; ++k) A[n * r + k] -= f * A[n * c + k];
      b[r] -= f * b[c];
    }
  }
  x.assign(n, 0.0);
  for (int r = n - 1; r >= 0; --r) {
    double v = b[r];
    for (int k = r + 1; k < n; ++k) v -= A[n * r + k] * x[k];
    x[r] = v / A[n * r + r];
  }
  return true;
}

}  // namespace

Mat4 arm_fk(const std::vector<double> & q) {
  Mat4 T = eye();
  for (int j = 0; j < 6; ++j) {
    T = mul(T, origin(ARM[j].xyz, ARM[j].rpy));
    if (j < 5) T = mul(T, rot_z(q[j]));  // 축이 전부 원점 좌표 z (URDF axis 0 0 1)
  }
  return T;
}

double down_rot_err(const Mat4 & T, double psi) {
  double Rt[9], e[3];
  down_rot(psi, Rt);
  rot_err_vec(T, Rt, e);
  return std::sqrt(e[0] * e[0] + e[1] * e[1] + e[2] * e[2]);
}

namespace {
bool ik_from(const std::array<double, 3> & p, double psi, const std::vector<double> & seed, std::vector<double> & q,
             double pos_tol, double rot_tol);
}  // namespace

bool arm_ik_down(const std::array<double, 3> & p, double psi, const std::vector<double> & seed, std::vector<double> & q,
                 double pos_tol, double rot_tol) {
  // 주어진 씨앗(앞 점 · 지금 자세) → 안 되면 학습 ArmIK.solve와 같은 씨앗들 (팔 방향 hd, 팔꿈치 위 자세 · 손목 여러 각)
  if (ik_from(p, psi, seed, q, pos_tol, rot_tol)) return true;
  const double hd = std::atan2(p[1], p[0]);
  for (double s5 : {psi - hd, hd - psi, 0.0, M_PI / 2, -M_PI / 2}) {
    for (const auto & base : {std::vector<double>{hd, -0.2, 0.7, 1.0, s5}, std::vector<double>{hd, 0.4, -0.6, 1.6, s5}}) {
      auto sd = base;
      sd[4] = std::remainder(sd[4], 2 * M_PI);
      if (ik_from(p, psi, sd, q, pos_tol, rot_tol)) return true;
    }
  }
  return false;
}

namespace {
bool ik_from(const std::array<double, 3> & p, double psi, const std::vector<double> & seed, std::vector<double> & q,
             double pos_tol, double rot_tol) {
  constexpr double W_ROT = 0.1;  // 회전 오차 가중 [m/rad] — 위치 mm 단위와 비슷한 크기로
  double Rt[9];
  down_rot(psi, Rt);
  // ψ와 ψ+π는 손이 대칭이라 같은 파지 — 둘 다 시도해 오차 작은 쪽
  double best = 1e9;
  std::vector<double> best_q;
  for (double ps : {psi, psi + M_PI}) {
    down_rot(ps, Rt);
    std::vector<double> x(seed.begin(), seed.begin() + 5);
    double lambda = 1e-3;
    for (int it = 0; it < 300; ++it) {
      const Mat4 T = arm_fk(x);
      double e[6] = {p[0] - T[3], p[1] - T[7], p[2] - T[11], 0, 0, 0};
      rot_err_vec(T, Rt, e + 3);
      for (int i = 3; i < 6; ++i) e[i] *= W_ROT;
      const double err = std::sqrt(e[0] * e[0] + e[1] * e[1] + e[2] * e[2]) + std::sqrt(e[3] * e[3] + e[4] * e[4] + e[5] * e[5]);
      if (std::sqrt(e[0] * e[0] + e[1] * e[1] + e[2] * e[2]) < pos_tol * 0.25 &&
          std::sqrt(e[3] * e[3] + e[4] * e[4] + e[5] * e[5]) / W_ROT < rot_tol * 0.25) break;
      // 수치 야코비안 6×5
      double J[6][5];
      for (int k = 0; k < 5; ++k) {
        auto xk = x;
        xk[k] += 1e-6;
        const Mat4 Tk = arm_fk(xk);
        double ek[6] = {p[0] - Tk[3], p[1] - Tk[7], p[2] - Tk[11], 0, 0, 0};
        rot_err_vec(Tk, Rt, ek + 3);
        for (int i = 3; i < 6; ++i) ek[i] *= W_ROT;
        for (int i = 0; i < 6; ++i) J[i][k] = -(ek[i] - e[i]) / 1e-6;  // d(지금 − 목표)/dq
      }
      std::vector<double> A(25, 0.0), b(5, 0.0), dx;
      for (int r = 0; r < 5; ++r) {
        for (int c = 0; c < 5; ++c) for (int i = 0; i < 6; ++i) A[5 * r + c] += J[i][r] * J[i][c];
        A[5 * r + r] += lambda;
        for (int i = 0; i < 6; ++i) b[r] += J[i][r] * e[i];
      }
      if (!solve(A, b, 5, dx)) break;
      auto xn = x;
      for (int k = 0; k < 5; ++k) xn[k] = std::clamp(x[k] + dx[k], LO[k], HI[k]);
      // 나빠지면 감쇠 키움 (LM)
      const Mat4 Tn = arm_fk(xn);
      double en[6] = {p[0] - Tn[3], p[1] - Tn[7], p[2] - Tn[11], 0, 0, 0};
      rot_err_vec(Tn, Rt, en + 3);
      for (int i = 3; i < 6; ++i) en[i] *= W_ROT;
      const double errn = std::sqrt(en[0] * en[0] + en[1] * en[1] + en[2] * en[2]) + std::sqrt(en[3] * en[3] + en[4] * en[4] + en[5] * en[5]);
      if (errn < err) { x = xn; lambda = std::max(lambda * 0.3, 1e-6); }
      else lambda *= 10;
      if (lambda > 1e4) break;
    }
    const Mat4 T = arm_fk(x);
    const double pe = std::sqrt((p[0] - T[3]) * (p[0] - T[3]) + (p[1] - T[7]) * (p[1] - T[7]) + (p[2] - T[11]) * (p[2] - T[11]));
    const double re = down_rot_err(T, ps);
    if (pe < pos_tol && re < rot_tol) {
      double d = 0;  // 씨앗에 가까운 해를 고름
      for (int k = 0; k < 5; ++k) d += std::abs(x[k] - seed[k]);
      if (d < best) { best = d; best_q = x; }
    }
  }
  if (best_q.empty()) return false;
  q = best_q;
  return true;
}
}  // namespace

}  // namespace plan
