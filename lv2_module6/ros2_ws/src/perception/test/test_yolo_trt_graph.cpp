// YOLO TensorRT: CUDA 그래프로 띄운 결과가 enqueueV3와 같은지 (같은 엔진 · 같은 사진 → 네모·점수 비트까지)
//   엔진(~/.cache/beebot/trt/<모델>_fp16.engine)이 없거나 TensorRT 없이 빌드됐으면 건너뜀 (PC 도커 · CI)
//   .onnx는 임시 폴더에 복사해 엔진보다 오래된 것으로 (yolo_onnx가 .onnx가 엔진보다 새면 엔진을 안 씀 — 워크트리 checkout 시각)
#include <gtest/gtest.h>

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <random>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "perception/yolo_onnx.hpp"

using perception::YoloBox;
using perception::YoloOnnx;
namespace fs = std::filesystem;

TEST(YoloTrt, CudaGraphSameAsEnqueue) {
  const char * home = std::getenv("HOME");
  const char * model_dir = std::getenv("PERCEPTION_TEST_MODEL_DIR");
  if (!home || !model_dir) GTEST_SKIP() << "HOME · PERCEPTION_TEST_MODEL_DIR 없음";
  const std::string name = "bee_yolo11n_v6";
  const fs::path onnx = fs::path(model_dir) / (name + ".onnx");
  const fs::path engine = fs::path(home) / ".cache/beebot/trt" / (name + "_fp16.engine");
  std::error_code ec;
  if (!fs::exists(onnx, ec) || !fs::exists(engine, ec)) GTEST_SKIP() << "모델 또는 엔진 없음: " << engine;
  const fs::path tmp = fs::temp_directory_path() / ("yolo_trt_graph_" + std::to_string(::getpid()));
  fs::create_directories(tmp);
  const fs::path onnx_old = tmp / onnx.filename();
  fs::copy_file(onnx, onnx_old, fs::copy_options::overwrite_existing);
  fs::last_write_time(onnx_old, fs::last_write_time(engine) - std::chrono::hours(1));

  std::vector<cv::Mat> imgs;
  for (const char * f : {"barcode_mission_p2-01.jpg", "barcode_tight_p2-03.jpg", "big_tag7.png", "glare_tag0.png"}) {
    cv::Mat m = cv::imread(std::string(std::getenv("PERCEPTION_TEST_DATA")) + "/" + f);
    if (!m.empty()) imgs.push_back(m);
  }
  cv::Mat noise(360, 640, CV_8UC3);
  cv::randu(noise, 0, 255);
  imgs.push_back(noise);
  ASSERT_FALSE(imgs.empty());

  std::vector<std::vector<YoloBox>> res[2];
  for (int graph = 0; graph < 2; ++graph) {
    YoloOnnx y(onnx_old.string(), "cuda", 1, 640, engine.string(), graph == 1);
    if (!y.ok() || y.provider().rfind("TensorRT", 0) != 0) {
      fs::remove_all(tmp, ec);
      GTEST_SKIP() << "TensorRT 못 씀: " << y.provider() << " · " << y.error();
    }
    if (graph) EXPECT_EQ(y.provider(), "TensorRT (CUDA 그래프)") << y.error();
    else EXPECT_EQ(y.provider(), "TensorRT");
    for (int rep = 0; rep < 2; ++rep)  // 같은 그래프를 여러 번 띄워도 같게
      for (const auto & m : imgs) res[graph].push_back(y.predict(m, 0.25f, false));
  }
  fs::remove_all(tmp, ec);
  ASSERT_EQ(res[0].size(), res[1].size());
  size_t boxes = 0;
  for (size_t i = 0; i < res[0].size(); ++i) {
    ASSERT_EQ(res[0][i].size(), res[1][i].size()) << "사진 " << i;
    for (size_t k = 0; k < res[0][i].size(); ++k) {
      const auto & a = res[0][i][k];
      const auto & b = res[1][i][k];
      EXPECT_EQ(a.cls, b.cls);
      EXPECT_EQ(a.conf, b.conf);
      EXPECT_EQ(a.x0, b.x0); EXPECT_EQ(a.y0, b.y0); EXPECT_EQ(a.x1, b.x1); EXPECT_EQ(a.y1, b.y1);
      ++boxes;
    }
  }
  printf("[yolo trt graph] 사진 %zu장 · 네모 %zu개 같음\n", res[0].size(), boxes);
}
