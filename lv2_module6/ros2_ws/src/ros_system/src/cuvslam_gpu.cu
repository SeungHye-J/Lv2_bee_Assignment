// cuvslam_slam C++판 GPU 입력 (11-16) — 가까운 depth 마스크를 GPU에서 (slam_math.hpp near_mask와 같은 식)
#include "cuvslam_gpu.hpp"

namespace {

__global__ void near_mask_kernel(const uint8_t * depth, size_t dpitch, uint8_t * mask, size_t mpitch, int w, int h, uint16_t lim) {
  const int x = blockIdx.x * blockDim.x + threadIdx.x, y = blockIdx.y * blockDim.y + threadIdx.y;
  if (x >= w || y >= h) return;
  const uint16_t d = reinterpret_cast<const uint16_t *>(depth + y * dpitch)[x];
  mask[y * mpitch + x] = static_cast<uint16_t>(d - 1u) < lim;  // 0 < d ≤ lim → 1 (0은 65535로 넘어가 빠짐)
}

}  // namespace

cudaError_t near_mask_gpu(const void * depth, size_t dpitch, void * mask, size_t mpitch, int w, int h, uint16_t lim,
                          cudaStream_t s) {
  const dim3 b(32, 8), g((w + b.x - 1) / b.x, (h + b.y - 1) / b.y);
  near_mask_kernel<<<g, b, 0, s>>>(static_cast<const uint8_t *>(depth), dpitch, static_cast<uint8_t *>(mask), mpitch, w, h, lim);
  return cudaGetLastError();
}
