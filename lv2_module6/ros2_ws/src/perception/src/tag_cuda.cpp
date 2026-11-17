// tag_cuda 구현 — 설명·선언은 include/perception/tag_cuda.hpp
#include "perception/tag_cuda.hpp"

#include <mutex>

#ifdef PERCEPTION_CUAPRILTAGS
#include <cuda_runtime.h>

#include <opencv2/imgproc.hpp>

#include "cuAprilTags.h"
#endif

namespace perception {

#ifdef PERCEPTION_CUAPRILTAGS
struct CudaTagDetector::Impl {
  std::mutex mu;
  std::string error;
  bool gpu{false};
  cuAprilTagsHandle det{nullptr};
  cudaStream_t stream{nullptr};
  uchar3 * dev{nullptr};       // RGB 영상 (GPU)
  size_t pitch{0};
  unsigned char * host{nullptr};  // RGB 영상 (고정 메모리 — 비동기 복사)
  int w{0}, h{0};
  static constexpr uint32_t MAX_TAGS = 64;
  cuAprilTagsID_t tags[MAX_TAGS];

  void release() {
    if (det) cuAprilTagsDestroy(det);
    if (dev) cudaFree(dev);
    if (host) cudaFreeHost(host);
    det = nullptr; dev = nullptr; host = nullptr; w = h = 0;
  }
  bool fail(const std::string & what, cudaError_t e = cudaSuccess) {
    error = what + (e != cudaSuccess ? std::string(": ") + cudaGetErrorString(e) : std::string());
    release();
    return false;
  }
  bool ensure(int width, int height) {
    if (det && width == w && height == h) return true;
    release();
    if (auto e = cudaMallocPitch(reinterpret_cast<void **>(&dev), &pitch, static_cast<size_t>(width) * 3, height); e != cudaSuccess)
      return fail("cudaMallocPitch", e);
    if (auto e = cudaMallocHost(reinterpret_cast<void **>(&host), static_cast<size_t>(width) * height * 3); e != cudaSuccess)
      return fail("cudaMallocHost", e);
    // 내부 자세 계산용 값 — 쓰지 않음 (꼭짓점만 받고 크기별 자세는 tag_pnp가 계산 — 퍽 16 mm · 장소 48 mm)
    const cuAprilTagsCameraIntrinsics_t k{static_cast<float>(width), static_cast<float>(width), width / 2.0f, height / 2.0f};
    // tile_size 4 = isaac_ros_apriltag 기본 (적응 이진화 창)
    if (int r = nvCreateAprilTagsDetector(&det, width, height, 4, NVAT_TAG36H11, &k, 0.05f); r != 0) {
      det = nullptr;
      return fail("nvCreateAprilTagsDetector " + std::to_string(r));
    }
    w = width; h = height;
    return true;
  }
};

CudaTagDetector::CudaTagDetector() : impl_(std::make_unique<Impl>()) {
  int n = 0;
  const auto e = cudaGetDeviceCount(&n);
  impl_->gpu = e == cudaSuccess && n > 0;
  if (!impl_->gpu) impl_->error = e != cudaSuccess ? std::string("GPU 없음: ") + cudaGetErrorString(e) : "GPU 없음";
  else if (auto s = cudaStreamCreateWithFlags(&impl_->stream, cudaStreamNonBlocking); s != cudaSuccess) {
    impl_->gpu = false;
    impl_->error = std::string("cudaStreamCreate: ") + cudaGetErrorString(s);
  }
}

CudaTagDetector::~CudaTagDetector() {
  impl_->release();
  if (impl_->stream) cudaStreamDestroy(impl_->stream);
}

bool CudaTagDetector::available() const { return impl_->gpu; }

std::string CudaTagDetector::error() const {
  std::lock_guard<std::mutex> lk(impl_->mu);
  return impl_->error;
}

std::optional<std::vector<Tag>> CudaTagDetector::detect(const cv::Mat & bgr, std::optional<double> min_px) {
  auto & m = *impl_;
  std::lock_guard<std::mutex> lk(m.mu);
  if (!m.gpu || bgr.empty() || bgr.type() != CV_8UC3) return std::nullopt;
  if (!m.ensure(bgr.cols, bgr.rows)) return std::nullopt;
  cv::Mat rgb(bgr.rows, bgr.cols, CV_8UC3, m.host);
  cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
  if (auto e = cudaMemcpy2DAsync(m.dev, m.pitch, m.host, rgb.step, static_cast<size_t>(m.w) * 3, m.h, cudaMemcpyHostToDevice,
                                 m.stream); e != cudaSuccess) {
    m.fail("cudaMemcpy2DAsync", e);
    return std::nullopt;
  }
  const cuAprilTagsImageInput_t in{m.dev, m.pitch, static_cast<uint16_t>(m.w), static_cast<uint16_t>(m.h)};
  uint32_t n = 0;
  if (int r = cuAprilTagsDetect(m.det, &in, m.tags, &n, Impl::MAX_TAGS, m.stream); r != 0) {
    m.fail("cuAprilTagsDetect " + std::to_string(r));
    return std::nullopt;
  }
  if (auto e = cudaStreamSynchronize(m.stream); e != cudaSuccess) {
    m.fail("cudaStreamSynchronize", e);
    return std::nullopt;
  }
  std::vector<std::pair<int, std::array<cv::Point2d, 4>>> found;
  for (uint32_t i = 0; i < n; ++i) {
    std::array<cv::Point2d, 4> pts;
    for (int k = 0; k < 4; ++k) {  // aruco k번 = cuAprilTags (k + 2) % 4번
      const auto & c = m.tags[i].corners[(k + 2) % 4];
      pts[k] = {c.x, c.y};
    }
    if (min_px) {
      double per = 0.0;
      for (int k = 0; k < 4; ++k) per += cv::norm(pts[k] - pts[(k + 1) % 4]);
      if (per < *min_px) continue;
    }
    found.emplace_back(m.tags[i].id, pts);
  }
  return make_tags(found);  // cuAprilTags가 한 태그를 두 번 내는 일이 있음 (합성 영상 11/14) → 같은 번호·8 px 안은 하나로
}

#else  // cuAprilTags 없이 빌드 — 늘 OpenCV

struct CudaTagDetector::Impl {};
CudaTagDetector::CudaTagDetector() : impl_(std::make_unique<Impl>()) {}
CudaTagDetector::~CudaTagDetector() = default;
bool CudaTagDetector::available() const { return false; }
std::string CudaTagDetector::error() const { return "cuAprilTags 없이 빌드됨 (cuapriltags_vendor · CUDA 없음)"; }
std::optional<std::vector<Tag>> CudaTagDetector::detect(const cv::Mat &, std::optional<double>) { return std::nullopt; }

#endif

}  // namespace perception
