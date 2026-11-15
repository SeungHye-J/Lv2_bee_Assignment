// body_filter_core GPU판 — 설명은 include/perception/body_filter_core.hpp
//   점구름 바이트(뽑을 점만, cudaMemcpy2DAsync로 stride 건너뛰며) → GPU 버퍼 · 상자 → GPU · 점마다 한 스레드 판정 → 판정표 내려받음
//   젯슨: 관리 메모리·매핑 고정 메모리 안 씀 (cudaMalloc + 복사) · 기다릴 때 CPU를 돌리지 않게 BlockingSync
//   nvcc --fmad=false (CMake) — CPU판과 비트까지 같은 판정
#include <cuda_runtime.h>

#include "body_filter_point.hpp"

namespace perception {

namespace {
__global__ void classify_kernel(const uint8_t * __restrict__ cloud, uint32_t step, size_t m, int ox, int oy, int oz,
                                const BodyBoxXf * __restrict__ boxes, int nbox, double sc0, double sc1, double sc2, double sr2,
                                bool want_floor, double fr0, double fr1, double fr2, double ft, double floor_z,
                                uint8_t * __restrict__ mask) {
  const size_t j = static_cast<size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (j >= m) return;
  const double sc[3] = {sc0, sc1, sc2};
  const double fr[3] = {fr0, fr1, fr2};
  mask[j] = bf::classify_point(cloud + j * step, ox, oy, oz, boxes, nbox, sc, sr2, want_floor, fr, ft, floor_z);
}
}  // namespace

struct BodyFilterCuda::Impl {
  std::string error;
  bool ok{false};
  cudaStream_t stream{nullptr};
  uint8_t * d_cloud{nullptr};
  size_t cloud_cap{0};
  BodyBoxXf * d_boxes{nullptr};
  size_t box_cap{0};
  uint8_t * d_mask{nullptr};
  size_t mask_cap{0};

  bool fail(const char * what, cudaError_t e) {
    error = std::string(what) + ": " + cudaGetErrorString(e);
    return false;
  }
  template <class T>
  bool grow(T *& p, size_t & cap, size_t need, const char * what) {
    if (need <= cap) return true;
    if (p) cudaFree(p);
    p = nullptr;
    cap = 0;
    const size_t want = need + need / 4;  // 점 수가 조금씩 바뀔 때 매번 다시 잡지 않게
    if (auto e = cudaMalloc(reinterpret_cast<void **>(&p), want * sizeof(T)); e != cudaSuccess) return fail(what, e);
    cap = want;
    return true;
  }
};

BodyFilterCuda::BodyFilterCuda() : impl_(std::make_unique<Impl>()) {
  int n = 0;
  if (auto e = cudaGetDeviceCount(&n); e != cudaSuccess || n == 0) {
    impl_->error = e != cudaSuccess ? std::string("cudaGetDeviceCount: ") + cudaGetErrorString(e) : "GPU 없음";
    return;
  }
  // 기다리는 동안 CPU 한 코어를 돌리지 않게 — 이미 컨텍스트가 있으면(같은 프로세스의 다른 CUDA) 실패해도 그대로 씀
  cudaSetDeviceFlags(cudaDeviceScheduleBlockingSync);
  cudaGetLastError();
  if (auto e = cudaStreamCreateWithFlags(&impl_->stream, cudaStreamNonBlocking); e != cudaSuccess) {
    impl_->fail("cudaStreamCreate", e);
    return;
  }
  impl_->ok = true;
}

BodyFilterCuda::~BodyFilterCuda() {
  if (impl_->d_cloud) cudaFree(impl_->d_cloud);
  if (impl_->d_boxes) cudaFree(impl_->d_boxes);
  if (impl_->d_mask) cudaFree(impl_->d_mask);
  if (impl_->stream) cudaStreamDestroy(impl_->stream);
}

bool BodyFilterCuda::available() const { return impl_->ok; }
const std::string & BodyFilterCuda::error() const { return impl_->error; }

bool BodyFilterCuda::classify(const BodyCloudJob & job, std::vector<uint8_t> & mask) {
  auto & s = *impl_;
  if (!s.ok) return false;
  const size_t m = job.samples();
  mask.resize(m);
  if (m == 0) return true;
  const size_t step = job.point_step;
  const size_t nbox = job.boxes.size();
  if (!s.grow(s.d_cloud, s.cloud_cap, m * step, "cudaMalloc cloud") || !s.grow(s.d_mask, s.mask_cap, m, "cudaMalloc mask") ||
      (nbox && !s.grow(s.d_boxes, s.box_cap, nbox, "cudaMalloc boxes")))
    return false;
  // 뽑을 점만 빽빽하게 올림 (stride 4면 1/4 바이트) — 한 행 = 점 하나
  cudaError_t e;
  if (job.stride == 1) e = cudaMemcpyAsync(s.d_cloud, job.data, m * step, cudaMemcpyHostToDevice, s.stream);
  else e = cudaMemcpy2DAsync(s.d_cloud, step, job.data, step * job.stride, step, m, cudaMemcpyHostToDevice, s.stream);
  if (e != cudaSuccess) return s.fail("cudaMemcpy cloud", e);
  if (nbox && (e = cudaMemcpyAsync(s.d_boxes, job.boxes.data(), nbox * sizeof(BodyBoxXf), cudaMemcpyHostToDevice, s.stream)) != cudaSuccess)
    return s.fail("cudaMemcpy boxes", e);
  constexpr int T = 256;
  const unsigned blocks = static_cast<unsigned>((m + T - 1) / T);
  classify_kernel<<<blocks, T, 0, s.stream>>>(s.d_cloud, job.point_step, m, job.ox, job.oy, job.oz, s.d_boxes, static_cast<int>(nbox),
                                               job.sc[0], job.sc[1], job.sc[2], job.sr2, job.want_floor, job.fr[0], job.fr[1], job.fr[2],
                                               job.ft, job.floor_z, s.d_mask);
  if ((e = cudaGetLastError()) != cudaSuccess) return s.fail("kernel", e);
  if ((e = cudaMemcpyAsync(mask.data(), s.d_mask, m, cudaMemcpyDeviceToHost, s.stream)) != cudaSuccess) return s.fail("cudaMemcpy mask", e);
  if ((e = cudaStreamSynchronize(s.stream)) != cudaSuccess) return s.fail("cudaStreamSynchronize", e);
  return true;
}

}  // namespace perception
