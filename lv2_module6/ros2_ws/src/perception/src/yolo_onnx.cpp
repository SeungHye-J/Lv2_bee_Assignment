// yolo_onnx 구현 — 설명·선언은 include/perception/yolo_onnx.hpp
//   숫자 규칙은 ultralytics 8.x detect(LetterBox · non_max_suppression · scale_boxes)를 그대로 옮김
#include "perception/yolo_onnx.hpp"

#include <onnxruntime_cxx_api.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <numeric>
#include <regex>
#include <stdexcept>

#include <opencv2/imgproc.hpp>

#ifdef PERCEPTION_TENSORRT
#include <NvInfer.h>
#include <cuda_runtime_api.h>
#endif

namespace perception {

namespace {

// 파이썬 round()와 같은 반올림 (0.5는 짝수 쪽 — 기본 반올림 모드 to-nearest-even)
double py_round(double v) { return std::nearbyint(v); }

// 두 네모 IoU (torchvision.ops.nms와 같은 넓이 계산 — +1 없음)
float iou_xyxy(const std::array<float, 4> & a, const std::array<float, 4> & b) {
  const float w = std::min(a[2], b[2]) - std::max(a[0], b[0]);
  const float h = std::min(a[3], b[3]) - std::max(a[1], b[1]);
  if (w <= 0.f || h <= 0.f) return 0.f;
  const float inter = w * h;
  const float ua = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter;
  return ua > 0.f ? inter / ua : 0.f;
}

// "{0: 'puck_red_box', 1: 'puck_red_cyl', ...}" → {0: puck_red_box, ...} (ultralytics가 onnx metadata에 넣는 형식)
std::map<int, std::string> parse_names(const std::string & s) {
  std::map<int, std::string> out;
  static const std::regex re(R"((\d+)\s*:\s*['"]([^'"]*)['"])");
  for (auto it = std::sregex_iterator(s.begin(), s.end(), re); it != std::sregex_iterator(); ++it)
    out[std::stoi((*it)[1].str())] = (*it)[2].str();
  return out;
}

#ifdef PERCEPTION_TENSORRT
// TensorRT 로그 → 마지막 경고·오류 한 줄만 남김 (엔진을 못 읽었을 때 이유로 error()에 붙임)
class TrtLogger : public nvinfer1::ILogger {
public:
  void log(Severity severity, const char * msg) noexcept override {
    if (severity <= Severity::kWARNING) last = msg;
  }
  std::string last;
};

// TensorRT 실행 묶음 — 엔진 · 실행 문맥 · CUDA 스트림 · 입출력 버퍼 (만들 때 한 번 잡고 계속 씀)
//   호스트 버퍼는 pinned(cudaMallocHost) — 복사가 빠르고 비동기 복사가 진짜 비동기로 됨
//   done 이벤트는 cudaEventBlockingSync — 기다리는 동안 CPU가 돌지 않고 잠듦 (CPU 절약의 핵심)
struct TrtRunner {
  TrtLogger logger;
  std::unique_ptr<nvinfer1::IRuntime> runtime;
  std::unique_ptr<nvinfer1::ICudaEngine> engine;  // runtime보다 먼저 지워야 함 (소멸자에서 순서대로 지움)
  std::unique_ptr<nvinfer1::IExecutionContext> ctx;
  cudaStream_t stream{nullptr};
  cudaEvent_t done{nullptr};
  void * d_in{nullptr};
  void * d_out{nullptr};
  float * h_in{nullptr};
  float * h_out{nullptr};
  size_t in_n{0}, out_n{0};        // 입력·출력 float 개수
  std::vector<int64_t> in_shape;   // (1, 3, H, W)
  std::vector<int64_t> out_shape;  // (1, 4 + 클래스 수, N)
  // CUDA 그래프 (11-16): enqueueV3는 커널 수백 개를 하나씩 띄움 — 실기처럼 CPU가 꽉 차면 띄우는 사이사이 스레드가 밀려
  //   GPU가 놀면서 기다림 (실기 GPU 점유 12~24 %인데 YOLO 17~91 ms). 한 번 잡아 둔 그래프는 cudaGraphLaunch 한 번으로 다 띄움.
  //   젯슨 실측 (실기 스택이 돌고 CPU 80~100 %일 때, 단독 프로그램 60회): 추론 중앙 14.2 → 4.0 ms · p90 31~41 → 6~7 ms ·
  //   그 스레드 CPU 4.2 → 0.4 ms/장. 같은 커널·같은 버퍼라 출력 같음 (test_yolo_trt_graph). 못 잡으면 enqueueV3 그대로
  cudaGraphExec_t graph{nullptr};
  std::string graph_error;  // 그래프를 못 잡은 이유 ('' = 잡음 또는 안 씀)

  ~TrtRunner() {
    if (graph) cudaGraphExecDestroy(graph);
    ctx.reset();
    engine.reset();
    runtime.reset();
    if (d_in) cudaFree(d_in);
    if (d_out) cudaFree(d_out);
    if (h_in) cudaFreeHost(h_in);
    if (h_out) cudaFreeHost(h_out);
    if (done) cudaEventDestroy(done);
    if (stream) cudaStreamDestroy(stream);
  }

  // 엔진 파일을 읽고 버퍼를 잡음 — 실패하면 이유를 돌려줌 ('' = 성공)
  std::string load(const std::string & path, int device_id) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return "엔진 파일을 못 엶: " + path;
    const std::vector<char> blob((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    if (cudaSetDevice(device_id) != cudaSuccess) return "cudaSetDevice 실패";
    runtime.reset(nvinfer1::createInferRuntime(logger));
    if (!runtime) return "TensorRT runtime 못 만듦";
    engine.reset(runtime->deserializeCudaEngine(blob.data(), blob.size()));
    if (!engine) return "엔진 읽기 실패 (다른 장치·TensorRT 버전에서 만든 엔진?) " + logger.last;
    // 입력 1개 · 출력 1개 · 둘 다 float32 (trtexec --fp16도 입출력은 float32 그대로 — 안쪽 계산만 FP16)
    if (engine->getNbIOTensors() != 2) return "입력 1개 · 출력 1개 엔진이 아님";
    const char * in_name = nullptr;
    const char * out_name = nullptr;
    for (int i = 0; i < engine->getNbIOTensors(); ++i) {
      const char * name = engine->getIOTensorName(i);
      if (engine->getTensorDataType(name) != nvinfer1::DataType::kFLOAT) return std::string("입출력이 float32가 아님: ") + name;
      if (engine->getTensorIOMode(name) == nvinfer1::TensorIOMode::kINPUT) in_name = name;
      else out_name = name;
    }
    if (!in_name || !out_name) return "입력 1개 · 출력 1개 엔진이 아님";
    const auto dims = [](const nvinfer1::Dims & d) { return std::vector<int64_t>(d.d, d.d + d.nbDims); };
    in_shape = dims(engine->getTensorShape(in_name));
    out_shape = dims(engine->getTensorShape(out_name));
    if (in_shape.size() != 4 || out_shape.size() != 3) return "입출력 모양이 YOLO가 아님";
    in_n = out_n = 1;
    for (auto d : in_shape) in_n *= static_cast<size_t>(std::max<int64_t>(d, 0));
    for (auto d : out_shape) out_n *= static_cast<size_t>(std::max<int64_t>(d, 0));
    if (!in_n || !out_n) return "고정 크기 엔진이 아님 (모양에 -1)";
    ctx.reset(engine->createExecutionContext());
    if (!ctx) return "실행 문맥 못 만듦 " + logger.last;
    if (cudaStreamCreate(&stream) != cudaSuccess ||
        cudaEventCreateWithFlags(&done, cudaEventBlockingSync | cudaEventDisableTiming) != cudaSuccess ||
        cudaMalloc(&d_in, in_n * sizeof(float)) != cudaSuccess || cudaMalloc(&d_out, out_n * sizeof(float)) != cudaSuccess ||
        cudaMallocHost(reinterpret_cast<void **>(&h_in), in_n * sizeof(float)) != cudaSuccess ||
        cudaMallocHost(reinterpret_cast<void **>(&h_out), out_n * sizeof(float)) != cudaSuccess)
      return "CUDA 버퍼 못 잡음";
    if (!ctx->setTensorAddress(in_name, d_in) || !ctx->setTensorAddress(out_name, d_out)) return "버퍼 연결 실패";
    return "";
  }

  // enqueueV3를 그래프로 잡음 (load 뒤 한 번) — 실패하면 graph 없이 (run이 enqueueV3로)
  void capture_graph() {
    // 첫 enqueueV3는 TensorRT가 안쪽 준비(메모리 · 전술 고르기)를 할 수 있어 그래프 밖에서 한 번 돌림
    if (!ctx->enqueueV3(stream) || cudaStreamSynchronize(stream) != cudaSuccess) {
      graph_error = "첫 enqueueV3 실패";
      return;
    }
    cudaGraph_t g = nullptr;
    if (cudaStreamBeginCapture(stream, cudaStreamCaptureModeThreadLocal) != cudaSuccess) {
      graph_error = "cudaStreamBeginCapture 실패";
      cudaGetLastError();
      return;
    }
    const bool enq = ctx->enqueueV3(stream);
    const cudaError_t end = cudaStreamEndCapture(stream, &g);
    if (!enq || end != cudaSuccess || !g) {
      graph_error = std::string("그래프로 못 잡음 (") + (enq ? cudaGetErrorString(end) : "enqueueV3") + ")";
      if (g) cudaGraphDestroy(g);
      cudaGetLastError();
      return;
    }
    const cudaError_t inst = cudaGraphInstantiate(&graph, g, 0);
    cudaGraphDestroy(g);
    if (inst != cudaSuccess) {
      graph = nullptr;
      graph_error = std::string("cudaGraphInstantiate: ") + cudaGetErrorString(inst);
      cudaGetLastError();
    }
  }

  // h_in(이미 채워 둠) → GPU → 추론 → h_out — 끝날 때까지 잠들어 기다림
  bool run() {
    if (cudaMemcpyAsync(d_in, h_in, in_n * sizeof(float), cudaMemcpyHostToDevice, stream) != cudaSuccess) return false;
    if (graph ? cudaGraphLaunch(graph, stream) != cudaSuccess : !ctx->enqueueV3(stream)) return false;
    if (cudaMemcpyAsync(h_out, d_out, out_n * sizeof(float), cudaMemcpyDeviceToHost, stream) != cudaSuccess) return false;
    if (cudaEventRecord(done, stream) != cudaSuccess) return false;
    return cudaEventSynchronize(done) == cudaSuccess;
  }
};
#endif

}  // namespace

struct YoloOnnx::Impl {
  Ort::Env env{ORT_LOGGING_LEVEL_WARNING, "perception_yolo"};  // 세션보다 먼저 만들고 늦게 지워야 함 (멤버 순서)
  std::unique_ptr<Ort::Session> session;
  std::string input_name, output_name, provider{"none"}, error;
  int in_h{0}, in_w{0};  // 모델 입력 크기 [px]
  std::map<int, std::string> names;
#ifdef PERCEPTION_TENSORRT
  std::unique_ptr<TrtRunner> trt;  // 있으면 추론은 이것으로 (그때 session은 비움)
#endif
};

YoloOnnx::YoloOnnx(const std::string & path, const std::string & device, int cpu_threads, int imgsz,
                   const std::string & trt_engine, bool trt_cuda_graph)
: impl_(std::make_shared<Impl>()) {
  auto & m = *impl_;
  // GPU를 원하나: "cuda" · "cuda:0" · "0"(GPU 번호, ultralytics device 표기)
  std::string dev = device;
  std::transform(dev.begin(), dev.end(), dev.begin(), [](unsigned char c) { return std::tolower(c); });
  const bool want_cuda = dev.rfind("cuda", 0) == 0 || (!dev.empty() && std::all_of(dev.begin(), dev.end(), ::isdigit));
  int device_id = 0;
  if (const auto c = dev.find(':'); c != std::string::npos) device_id = std::atoi(dev.c_str() + c + 1);
  else if (!dev.empty() && std::isdigit(static_cast<unsigned char>(dev[0]))) device_id = std::atoi(dev.c_str());

  const auto make_options = [&](bool cuda) {
    Ort::SessionOptions so;
    so.SetIntraOpNumThreads(std::max(1, cpu_threads));  // CPU 연산 스레드 (GPU여도 앞뒤 처리에 씀)
    so.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    if (cuda) {
      OrtCUDAProviderOptions cuda_options{};
      cuda_options.device_id = device_id;
      so.AppendExecutionProvider_CUDA(cuda_options);  // CUDA 빌드가 아니면 여기서 예외
    }
    return so;
  };
  // ---- TensorRT 엔진 먼저 (GPU를 원하고 · 엔진 경로가 있고 · TensorRT로 빌드됐을 때만)
  //   안 되면 m.error에 이유를 적고 아래 ONNX Runtime 그대로 (노드는 그 이유를 경고로 찍음)
  if (want_cuda && !trt_engine.empty()) {
#ifdef PERCEPTION_TENSORRT
    namespace fs = std::filesystem;
    const std::string make_cmd = "/usr/src/tensorrt/bin/trtexec --onnx=" + path + " --saveEngine=" + trt_engine + " --fp16";
    std::error_code ec;
    if (!fs::exists(trt_engine, ec)) {
      m.error = "TensorRT 엔진 없음 — ONNX Runtime으로 · 만들기(젯슨, 약 6분): " + make_cmd;
    } else if (fs::last_write_time(path, ec) > fs::last_write_time(trt_engine, ec)) {
      // .onnx가 바뀌었는데 엔진은 옛 모델 그대로일 수 있음 → 틀린 검출을 내느니 느린 쪽으로
      m.error = "TensorRT 엔진이 .onnx보다 오래됨 — ONNX Runtime으로 · 다시 만들기: " + make_cmd;
    } else {
      try {
        // 클래스 이름·입출력 모양은 .onnx에서 (엔진엔 metadata가 없음) — CPU 세션으로 잠깐 열고 닫음 (GPU 메모리 안 씀)
        Ort::SessionOptions so;
        so.SetIntraOpNumThreads(1);
        Ort::Session probe(m.env, path.c_str(), so);
        Ort::AllocatorWithDefaultOptions alloc;
        const auto ishape = probe.GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
        const auto oshape = probe.GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
        auto trt = std::make_unique<TrtRunner>();
        const std::string why = trt->load(trt_engine, device_id);
        if (!why.empty()) {
          m.error = "TensorRT 실패 — ONNX Runtime으로: " + why;
        } else if (trt->in_shape != std::vector<int64_t>(ishape.begin(), ishape.end()) ||
                   trt->out_shape != std::vector<int64_t>(oshape.begin(), oshape.end())) {
          m.error = "TensorRT 엔진 입출력 모양이 .onnx와 다름 (다른 모델의 엔진?) — ONNX Runtime으로 · 다시 만들기: " + make_cmd;
        } else {
          m.in_h = static_cast<int>(ishape[2]);
          m.in_w = static_cast<int>(ishape[3]);
          auto names = probe.GetModelMetadata().LookupCustomMetadataMapAllocated("names", alloc);
          if (names) m.names = parse_names(names.get());
          m.provider = "TensorRT";
          if (trt_cuda_graph) {
            trt->capture_graph();
            if (trt->graph) m.provider = "TensorRT (CUDA 그래프)";
            else m.error = "TensorRT CUDA 그래프 못 씀 — enqueueV3로: " + trt->graph_error;
          }
          m.trt = std::move(trt);
          return;  // ONNX Runtime 세션은 안 만듦
        }
      } catch (const Ort::Exception & e) {
        m.error = std::string("TensorRT 준비 중 .onnx 읽기 실패 — ONNX Runtime으로: ") + e.what();
      }
    }
#else
    m.error = "TensorRT 없이 빌드됨 (libnvinfer-dev 없음) — ONNX Runtime으로";
#endif
  }

  try {
    if (want_cuda) {
      const auto av = Ort::GetAvailableProviders();
      if (std::find(av.begin(), av.end(), "CUDAExecutionProvider") != av.end()) {
        try {
          m.session = std::make_unique<Ort::Session>(m.env, path.c_str(), make_options(true));
          m.provider = "CUDAExecutionProvider";
        } catch (const Ort::Exception & e) {
          m.error += (m.error.empty() ? "" : " · ") + std::string("CUDA 실패 — CPU로: ") + e.what();
        }
      } else {
        m.error += (m.error.empty() ? "" : " · ") + std::string("이 ONNX Runtime에 CUDAExecutionProvider 없음 — CPU로");
      }
    }
    if (!m.session) {
      m.session = std::make_unique<Ort::Session>(m.env, path.c_str(), make_options(false));
      m.provider = "CPUExecutionProvider";
    }
    Ort::AllocatorWithDefaultOptions alloc;
    m.input_name = m.session->GetInputNameAllocated(0, alloc).get();
    m.output_name = m.session->GetOutputNameAllocated(0, alloc).get();
    // 입력 크기: 고정이면 그 값 (1×3×H×W) · 동적(-1)이면 imgsz 정사각형
    const auto shape = m.session->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
    m.in_h = shape.size() == 4 && shape[2] > 0 ? static_cast<int>(shape[2]) : imgsz;
    m.in_w = shape.size() == 4 && shape[3] > 0 ? static_cast<int>(shape[3]) : imgsz;
    // 클래스 이름: ultralytics가 넣은 metadata "names"
    const auto meta = m.session->GetModelMetadata();
    auto names = meta.LookupCustomMetadataMapAllocated("names", alloc);
    if (names) m.names = parse_names(names.get());
  } catch (const Ort::Exception & e) {
    m.session.reset();
    m.provider = "none";
    m.error = std::string("모델 로드 실패: ") + e.what();
  }
}

bool YoloOnnx::ok() const {
#ifdef PERCEPTION_TENSORRT
  if (impl_->trt) return true;
#endif
  return impl_->session != nullptr;
}
std::string YoloOnnx::provider() const { return impl_->provider; }
std::string YoloOnnx::error() const { return impl_->error; }
const std::map<int, std::string> & YoloOnnx::names() const { return impl_->names; }

std::vector<YoloBox> YoloOnnx::predict(const cv::Mat & bgr, float conf, bool agnostic_nms, float iou, int max_det) {
  auto & m = *impl_;
  std::vector<YoloBox> out;
  if (!ok() || bgr.empty()) return out;
  const int h0 = bgr.rows, w0 = bgr.cols, H = m.in_h, W = m.in_w;

  // ---- LetterBox (ultralytics: auto=False · 가운데 · 여백 114) — 비율 유지해 줄이고(키우고) 남는 곳을 회색으로
  const double r = std::min(static_cast<double>(H) / h0, static_cast<double>(W) / w0);
  const int nw = static_cast<int>(py_round(w0 * r)), nh = static_cast<int>(py_round(h0 * r));
  const double dw = (W - nw) / 2.0, dh = (H - nh) / 2.0;
  cv::Mat img = bgr;
  if (nw != w0 || nh != h0) cv::resize(bgr, img, cv::Size(nw, nh), 0, 0, cv::INTER_LINEAR);
  const int top = static_cast<int>(py_round(dh - 0.1)), bottom = static_cast<int>(py_round(dh + 0.1));
  const int left = static_cast<int>(py_round(dw - 0.1)), right = static_cast<int>(py_round(dw + 0.1));
  cv::Mat lb;
  cv::copyMakeBorder(img, lb, top, bottom, left, right, cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));
  if (lb.rows != H || lb.cols != W) cv::resize(lb, lb, cv::Size(W, H));  // 반올림으로 1px 어긋나면 맞춤 (보통 안 일어남)

  // ---- BGR → RGB · 0~1 · NCHW — TensorRT면 GPU로 보낼 pinned 버퍼에 바로 씀 (복사 한 번 줄임)
  std::vector<float> input;
  float * in = nullptr;
#ifdef PERCEPTION_TENSORRT
  if (m.trt) in = m.trt->h_in;
#endif
  if (!in) {
    input.resize(static_cast<size_t>(3) * H * W);
    in = input.data();
  }
  for (int y = 0; y < H; ++y) {
    const auto * row = lb.ptr<cv::Vec3b>(y);
    for (int x = 0; x < W; ++x) {
      const size_t i = static_cast<size_t>(y) * W + x;
      in[i] = row[x][2] / 255.f;                               // R
      in[static_cast<size_t>(H) * W + i] = row[x][1] / 255.f;  // G
      in[static_cast<size_t>(2) * H * W + i] = row[x][0] / 255.f;  // B
    }
  }

  // ---- 추론 → 출력 p (1, 4 + 클래스 수, N) — TensorRT 또는 ONNX Runtime (아래 후처리는 같음)
  const float * p = nullptr;
  int nc = 0, n = 0;
  std::vector<Ort::Value> outputs;  // ONNX Runtime 출력 (p가 가리키므로 후처리 끝까지 살아 있어야 함)
#ifdef PERCEPTION_TENSORRT
  if (m.trt) {
    if (!m.trt->run()) throw std::runtime_error("TensorRT 추론 실패 (CUDA 오류)");  // on_infer가 잡아 경고
    p = m.trt->h_out;
    nc = static_cast<int>(m.trt->out_shape[1]) - 4;
    n = static_cast<int>(m.trt->out_shape[2]);
  }
#endif
  if (!p) {
    const std::array<int64_t, 4> in_shape{1, 3, H, W};
    auto mem = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    Ort::Value in_tensor = Ort::Value::CreateTensor<float>(mem, input.data(), input.size(), in_shape.data(), in_shape.size());
    const char * in_names[] = {m.input_name.c_str()};
    const char * out_names[] = {m.output_name.c_str()};
    outputs = m.session->Run(Ort::RunOptions{nullptr}, in_names, &in_tensor, 1, out_names, 1);
    const auto oshape = outputs[0].GetTensorTypeAndShapeInfo().GetShape();  // (1, 4 + 클래스 수, N)
    if (oshape.size() != 3) return out;
    nc = static_cast<int>(oshape[1]) - 4;
    n = static_cast<int>(oshape[2]);
    p = outputs[0].GetTensorData<float>();
  }

  // ---- 후보: 가장 높은 클래스 점수 > conf (ultralytics non_max_suppression, multi_label 끔)
  struct Cand { std::array<float, 4> b; float s; int c; };
  std::vector<Cand> cands;
  for (int i = 0; i < n; ++i) {
    int best = 0;
    float s = -1.f;
    for (int c = 0; c < nc; ++c) {
      const float v = p[static_cast<size_t>(4 + c) * n + i];
      if (v > s) { s = v; best = c; }
    }
    if (!(s > conf)) continue;
    const float cx = p[i], cy = p[n + i], bw = p[2 * n + i], bh = p[3 * n + i];
    cands.push_back({{cx - bw / 2.f, cy - bh / 2.f, cx + bw / 2.f, cy + bh / 2.f}, s, best});
  }
  // 점수 높은 순 (같으면 원래 순서) · 최대 30000개 (ultralytics max_nms)
  std::stable_sort(cands.begin(), cands.end(), [](const Cand & a, const Cand & b) { return a.s > b.s; });
  if (cands.size() > 30000) cands.resize(30000);

  // ---- NMS: 클래스마다 따로 (클래스 번호 × 7680만큼 옮겨 서로 안 겹치게 — agnostic이면 안 옮김) · IoU > iou면 지움
  const float max_wh = 7680.f;
  std::vector<std::array<float, 4>> shifted(cands.size());
  for (size_t i = 0; i < cands.size(); ++i) {
    const float off = agnostic_nms ? 0.f : cands[i].c * max_wh;
    shifted[i] = {cands[i].b[0] + off, cands[i].b[1] + off, cands[i].b[2] + off, cands[i].b[3] + off};
  }
  std::vector<char> removed(cands.size(), 0);
  std::vector<size_t> keep;
  for (size_t i = 0; i < cands.size() && static_cast<int>(keep.size()) < max_det; ++i) {
    if (removed[i]) continue;
    keep.push_back(i);
    for (size_t j = i + 1; j < cands.size(); ++j)
      if (!removed[j] && iou_xyxy(shifted[i], shifted[j]) > iou) removed[j] = 1;
  }

  // ---- 원본 좌표로 (scale_boxes) · 화면 안으로 자름 (clip_boxes)
  const double gain = std::min(static_cast<double>(H) / h0, static_cast<double>(W) / w0);
  const double pad_x = py_round((W - w0 * gain) / 2.0 - 0.1), pad_y = py_round((H - h0 * gain) / 2.0 - 0.1);
  for (size_t k : keep) {
    const auto & c = cands[k];
    const auto sx = [&](float v) { return std::clamp(static_cast<float>((v - pad_x) / gain), 0.f, static_cast<float>(w0)); };
    const auto sy = [&](float v) { return std::clamp(static_cast<float>((v - pad_y) / gain), 0.f, static_cast<float>(h0)); };
    out.push_back({sx(c.b[0]), sy(c.b[1]), sx(c.b[2]), sy(c.b[3]), c.s, c.c});
  }
  return out;
}

}  // namespace perception
