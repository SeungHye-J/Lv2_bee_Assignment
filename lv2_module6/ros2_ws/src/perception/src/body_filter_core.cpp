// body_filter_core CPU판 (+ CUDA 없이 빌드할 때 GPU판 빈 껍데기) — 설명은 include/perception/body_filter_core.hpp
//   CMake가 이 파일만 -ffp-contract=off로 빌드 (GPU판 --fmad=false와 결과 비트까지 같게)
#include "body_filter_point.hpp"

namespace perception {

void classify_body_cpu(const BodyCloudJob & job, std::vector<uint8_t> & mask) {
  const size_t m = job.samples();
  mask.resize(m);
  const int nbox = static_cast<int>(job.boxes.size());
  for (size_t j = 0; j < m; ++j) {
    mask[j] = bf::classify_point(job.data + j * job.stride * job.point_step, job.ox, job.oy, job.oz, job.boxes.data(), nbox,
                                 job.sc, job.sr2, job.want_floor, job.fr, job.ft, job.floor_z);
  }
}

#ifndef PERCEPTION_BODY_CUDA
struct BodyFilterCuda::Impl {
  std::string error{"CUDA 없이 빌드됨 (nvcc 없음)"};
};
BodyFilterCuda::BodyFilterCuda() : impl_(std::make_unique<Impl>()) {}
BodyFilterCuda::~BodyFilterCuda() = default;
bool BodyFilterCuda::available() const { return false; }
const std::string & BodyFilterCuda::error() const { return impl_->error; }
bool BodyFilterCuda::classify(const BodyCloudJob &, std::vector<uint8_t> &) { return false; }
#endif

}  // namespace perception
