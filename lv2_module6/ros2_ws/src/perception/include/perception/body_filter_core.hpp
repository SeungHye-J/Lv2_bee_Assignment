#pragma once

// body_filter_core — body_filter의 점마다 판정(몸 상자 안? · 바닥 위?)만 따로: CPU판과 GPU(CUDA)판이 같은 결과를 냄
//   왜 (11-16): body_filter는 15 Hz 점구름마다 점 × 상자(URDF 충돌 상자 77개) 변환을 함 — 팔이 화면에 들어와 점이 몸 구 안에
//     많을수록 비쌈. GPU판은 점구름(xyz가 든 바이트 그대로)을 GPU로 올려 점마다 한 스레드가 판정 → 판정표(바이트 1개/점)만 내려받음.
//     출력 점구름 만들기(남길 점 복사)는 두 판 모두 CPU (body_filter.cpp) — 판정표가 같으면 출력도 바이트까지 같음.
//   같은 결과: 두 판 모두 double · 같은 계산 순서 · 곱셈-덧셈 합치기(FMA) 끔 (CPU -ffp-contract=off · nvcc --fmad=false)
//     → 판정표가 비트까지 같음 (test_body_filter_cuda가 무작위 점구름으로 확인)
//   GPU판 빌드: CUDA 컴파일러(nvcc)가 있을 때만 (PERCEPTION_BODY_CUDA) · 없으면 BodyFilterCuda::available() = false → CPU판
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace perception {

// 카메라 → 상자 좌표 변환 (q = R·p + t) 과 반 크기(+padding) — tf2::Transform을 펼친 것
struct BodyBoxXf {
  double r[9];     // 행 우선 3×3
  double t[3];
  double half[3];
};

// 점구름 한 장 판정 입력 — data는 PointCloud2.data 그대로 (x·y·z는 FLOAT32, 오프셋 ox·oy·oz)
struct BodyCloudJob {
  const uint8_t * data{nullptr};
  size_t n{0};              // 점 수 (width·height)
  uint32_t point_step{0};
  int ox{0}, oy{4}, oz{8};
  size_t stride{1};         // i = 0, stride, 2·stride … 만 판정
  std::vector<BodyBoxXf> boxes;
  double sc[3]{0, 0, 0};    // 몸 전체를 감싸는 구 (카메라 좌표) — 밖이면 상자 검사 생략
  double sr2{0};
  bool want_floor{false};   // 바닥 판정도 (floor_frame z = fr·p + ft > floor_z)
  double fr[3]{0, 0, 1};    // to_floor 회전의 z 행
  double ft{0};
  double floor_z{0};
  size_t samples() const { return n == 0 ? 0 : (n - 1) / stride + 1; }
};

// 판정표 값 (점마다 1바이트) — 0 = 몸(뺌) · KEEP = 남김 · KEEP|ABOVE_FLOOR = 남김 + 바닥 위(no_floor 출력에도)
constexpr uint8_t BODY_KEEP = 1;
constexpr uint8_t BODY_ABOVE_FLOOR = 2;

// CPU판 — mask 크기 = job.samples()
void classify_body_cpu(const BodyCloudJob & job, std::vector<uint8_t> & mask);

// GPU판 — GPU 버퍼·스트림은 처음 쓸 때 만들고 점 수가 늘 때만 다시 잡음 (한 노드 콜백에서만 부름 — 스레드 안전 아님)
class BodyFilterCuda {
public:
  BodyFilterCuda();
  ~BodyFilterCuda();
  BodyFilterCuda(const BodyFilterCuda &) = delete;
  BodyFilterCuda & operator=(const BodyFilterCuda &) = delete;
  bool available() const;          // GPU판으로 빌드됐고 GPU가 보이나
  const std::string & error() const;  // 마지막 실패 이유
  bool classify(const BodyCloudJob & job, std::vector<uint8_t> & mask);  // false = 실패 (부른 쪽이 CPU판으로)

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace perception
