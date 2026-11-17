#pragma once
// 점 하나 판정 — CPU판(body_filter_core.cpp)과 GPU판(body_filter_cuda.cu)이 같은 코드를 씀 (결과 비트까지 같게)
//   계산 순서는 예전 tf2 코드 그대로: dot = (a0·b0 + a1·b1) + a2·b2 · 변환 = R·p + t · 구 = |p − sc|² > sr²
#include <cmath>
#include <cstdint>
#include <cstring>

#include "perception/body_filter_core.hpp"

#ifdef __CUDACC__
#define BF_HD __host__ __device__ __forceinline__
#else
#define BF_HD inline
#endif

namespace perception {
namespace bf {

BF_HD double dot3(const double * a, double x, double y, double z) { return a[0] * x + a[1] * y + a[2] * z; }

BF_HD float load_f32(const uint8_t * p) {
  float v;
  memcpy(&v, p, 4);
  return v;
}

// p = 점 바이트 시작 · 반환 = 판정표 값 (body_filter_core.hpp BODY_*)
BF_HD uint8_t classify_point(const uint8_t * p, int ox, int oy, int oz, const BodyBoxXf * boxes, int nbox,
                             const double * sc, double sr2, bool want_floor, const double * fr, double ft, double floor_z) {
  const double x = load_f32(p + ox), y = load_f32(p + oy), z = load_f32(p + oz);
  const double dx = x - sc[0], dy = y - sc[1], dz = z - sc[2];
  if (!(dx * dx + dy * dy + dz * dz > sr2)) {  // 몸 구 안(또는 NaN — 예전처럼 상자 검사로) → 상자마다
    for (int k = 0; k < nbox; ++k) {
      const BodyBoxXf & b = boxes[k];
      const double qx = dot3(b.r, x, y, z) + b.t[0];
      const double qy = dot3(b.r + 3, x, y, z) + b.t[1];
      const double qz = dot3(b.r + 6, x, y, z) + b.t[2];
      if (fabs(qx) <= b.half[0] && fabs(qy) <= b.half[1] && fabs(qz) <= b.half[2]) return 0;
    }
  }
  uint8_t m = BODY_KEEP;
  if (want_floor && dot3(fr, x, y, z) + ft > floor_z) m |= BODY_ABOVE_FLOOR;
  return m;
}

}  // namespace bf
}  // namespace perception
