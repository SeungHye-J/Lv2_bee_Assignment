// body_filter_core — GPU판 판정표가 CPU판과 비트까지 같은지 + 예전 tf2 루프(11-16 전 body_filter.cpp)와 같은지 (무작위 합성 점구름)
//   GPU가 없거나 CUDA 없이 빌드됐으면 GPU 비교는 건너뜀. 끝에 걸린 시간(벽시계 · 프로세스 CPU)을 찍음 (BodyFilterCudaBench)
#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstring>
#include <ctime>
#include <limits>
#include <random>

#include "perception/body_filter_core.hpp"
#include "tf2/LinearMath/Transform.hpp"

using namespace perception;

namespace {

struct Scene {
  std::vector<uint8_t> data;
  BodyCloudJob job;
  std::vector<tf2::Transform> to_box;  // 예전 루프용 (같은 값)
  std::vector<tf2::Vector3> half;
  tf2::Transform to_floor;
};

// nbox개 상자(무작위 방향·크기)를 카메라 앞 1 m 둘레에 · 점의 inside_share는 상자 근처, 나머지는 멀리 · NaN 몇 개 · 상자 면 위 점 몇 개
Scene make_scene(uint32_t seed, size_t n, uint32_t step, int ox, int oy, int oz, size_t stride, int nbox, double inside_share, bool floor) {
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> u(-1.0, 1.0);
  Scene s;
  for (int k = 0; k < nbox; ++k) {
    tf2::Quaternion q(u(rng), u(rng), u(rng), u(rng) + 1.5);
    q.normalize();
    const tf2::Transform cam_box(q, tf2::Vector3(0.4 * u(rng), 0.3 * u(rng), 1.0 + 0.4 * u(rng)));
    s.to_box.push_back(cam_box.inverse());
    s.half.emplace_back(0.03 + 0.08 * std::abs(u(rng)), 0.03 + 0.08 * std::abs(u(rng)), 0.03 + 0.15 * std::abs(u(rng)));
  }
  // body_filter.cpp와 같은 구
  tf2::Vector3 sc(0, 0, 0);
  for (const auto & t : s.to_box) sc += t.inverse().getOrigin();
  if (!s.to_box.empty()) sc /= static_cast<double>(s.to_box.size());
  double sr = 0;
  for (size_t k = 0; k < s.to_box.size(); ++k) sr = std::max(sr, (s.to_box[k].inverse().getOrigin() - sc).length() + s.half[k].length());
  s.to_floor = tf2::Transform(tf2::Quaternion(tf2::Vector3(1, 0, 0), -2.0), tf2::Vector3(0.1, 0, 0.9));

  s.data.assign(n * step, 0xCD);
  for (size_t i = 0; i < n; ++i) {
    float p[3];
    const double r = std::uniform_real_distribution<double>(0, 1)(rng);
    if (r < inside_share && nbox > 0) {  // 상자 하나 근처 (상자 좌표에서 반 크기 × 1.3 안 → 안·밖 섞임)
      const int k = static_cast<int>(rng() % static_cast<uint32_t>(nbox));
      const tf2::Vector3 qb(1.3 * s.half[k].x() * u(rng), 1.3 * s.half[k].y() * u(rng), 1.3 * s.half[k].z() * u(rng));
      const tf2::Vector3 pc = s.to_box[k].inverse() * qb;
      p[0] = float(pc.x()); p[1] = float(pc.y()); p[2] = float(pc.z());
    } else {
      p[0] = float(3 * u(rng)); p[1] = float(2 * u(rng)); p[2] = float(0.2 + 4 * std::abs(u(rng)));
    }
    if (i % 997 == 5) p[1] = std::numeric_limits<float>::quiet_NaN();
    if (i % 1009 == 7) { p[0] = float(sc.x()); p[1] = float(sc.y()); p[2] = float(sc.z()); }  // 구 가운데
    std::memcpy(&s.data[i * step + ox], &p[0], 4);
    std::memcpy(&s.data[i * step + oy], &p[1], 4);
    std::memcpy(&s.data[i * step + oz], &p[2], 4);
  }
  auto & j = s.job;
  j.data = s.data.data();
  j.n = n;
  j.point_step = step;
  j.ox = ox; j.oy = oy; j.oz = oz;
  j.stride = stride;
  for (size_t k = 0; k < s.to_box.size(); ++k) {
    BodyBoxXf b;
    for (int r = 0; r < 3; ++r) {
      const tf2::Vector3 row = s.to_box[k].getBasis().getRow(r);
      b.r[r * 3] = row.x(); b.r[r * 3 + 1] = row.y(); b.r[r * 3 + 2] = row.z();
      b.t[r] = s.to_box[k].getOrigin()[r];
      b.half[r] = s.half[k][r];
    }
    j.boxes.push_back(b);
  }
  j.sc[0] = sc.x(); j.sc[1] = sc.y(); j.sc[2] = sc.z();
  j.sr2 = sr * sr;
  j.want_floor = floor;
  const tf2::Vector3 fz = s.to_floor.getBasis().getRow(2);
  j.fr[0] = fz.x(); j.fr[1] = fz.y(); j.fr[2] = fz.z();
  j.ft = s.to_floor.getOrigin().z();
  j.floor_z = 0.02;
  return s;
}

// 11-16 전 body_filter.cpp on_cloud 루프 그대로 (tf2 Vector3·Transform) → 판정표
std::vector<uint8_t> legacy_mask(const Scene & s) {
  const auto & j = s.job;
  const tf2::Vector3 sc(j.sc[0], j.sc[1], j.sc[2]);
  std::vector<uint8_t> mask;
  for (size_t i = 0; i < j.n; i += j.stride) {
    const uint8_t * p = &s.data[i * j.point_step];
    float x, y, z;
    std::memcpy(&x, p + j.ox, 4);
    std::memcpy(&y, p + j.oy, 4);
    std::memcpy(&z, p + j.oz, 4);
    const tf2::Vector3 pt(x, y, z);
    bool body = false;
    if (!((pt - sc).length2() > j.sr2)) {
      for (size_t k = 0; k < s.to_box.size() && !body; ++k) {
        const tf2::Vector3 q = s.to_box[k] * pt;
        body = std::abs(q.x()) <= s.half[k].x() && std::abs(q.y()) <= s.half[k].y() && std::abs(q.z()) <= s.half[k].z();
      }
    }
    if (body) { mask.push_back(0); continue; }
    mask.push_back(BODY_KEEP | ((j.want_floor && (s.to_floor * pt).z() > j.floor_z) ? BODY_ABOVE_FLOOR : 0));
  }
  return mask;
}

size_t count_diff(const std::vector<uint8_t> & a, const std::vector<uint8_t> & b) {
  if (a.size() != b.size()) return std::max(a.size(), b.size());
  size_t d = 0;
  for (size_t i = 0; i < a.size(); ++i) d += a[i] != b[i];
  return d;
}

double cpu_now() {
  timespec t;
  clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &t);
  return t.tv_sec + t.tv_nsec * 1e-9;
}

struct Case { uint32_t step; int ox, oy, oz; size_t stride; int nbox; double inside; bool floor; };
const Case CASES[] = {
  {16, 0, 4, 8, 4, 77, 0.3, true},    // 실기 형식 (realsense xyz + rgb 16 B · stride 4)
  {16, 0, 4, 8, 1, 77, 0.6, false},
  {32, 12, 4, 20, 3, 40, 0.5, true},  // 순서 뒤섞인 오프셋 · 홀수 stride
  {20, 8, 12, 16, 7, 5, 0.9, true},
  {16, 0, 4, 8, 1, 0, 0.0, true},     // 상자 없음 (URDF는 있는데 TF 전부 실패)
};

}  // namespace

TEST(BodyFilterCore, CpuMatchesLegacyTf2Loop) {
  uint32_t seed = 1;
  for (const auto & c : CASES) {
    const Scene s = make_scene(seed++, 60000, c.step, c.ox, c.oy, c.oz, c.stride, c.nbox, c.inside, c.floor);
    std::vector<uint8_t> cpu;
    classify_body_cpu(s.job, cpu);
    const auto legacy = legacy_mask(s);
    size_t body = 0;
    for (auto m : cpu) body += m == 0;
    if (c.nbox && c.inside > 0) EXPECT_GT(body, 0u);
    // 예전 루프(FMA 합치기 허용 빌드)와 다를 수 있는 건 상자 면에서 1e-15 m 안의 점뿐 — 무작위 점에서는 0개
    EXPECT_EQ(count_diff(cpu, legacy), 0u) << "step " << c.step << " stride " << c.stride;
  }
}

TEST(BodyFilterCore, GpuMatchesCpuBitExact) {
  BodyFilterCuda gpu;
  if (!gpu.available()) GTEST_SKIP() << "GPU판 없음: " << gpu.error();
  uint32_t seed = 100;
  for (int rep = 0; rep < 2; ++rep) {  // 두 번째는 버퍼 다시 씀 (크기 바뀜)
    for (const auto & c : CASES) {
      const Scene s = make_scene(seed++, rep ? 150000 : 30000, c.step, c.ox, c.oy, c.oz, c.stride, c.nbox, c.inside, c.floor);
      std::vector<uint8_t> cpu, g;
      classify_body_cpu(s.job, cpu);
      ASSERT_TRUE(gpu.classify(s.job, g)) << gpu.error();
      EXPECT_EQ(count_diff(cpu, g), 0u) << "step " << c.step << " stride " << c.stride << " nbox " << c.nbox;
    }
  }
  // 빈 점구름 · 점 1개
  std::vector<uint8_t> g;
  BodyCloudJob empty;
  EXPECT_TRUE(gpu.classify(empty, g));
  EXPECT_TRUE(g.empty());
  const Scene one = make_scene(7, 1, 16, 0, 4, 8, 4, 3, 1.0, true);
  std::vector<uint8_t> cpu1;
  classify_body_cpu(one.job, cpu1);
  ASSERT_TRUE(gpu.classify(one.job, g));
  EXPECT_EQ(g, cpu1);
}

// 걸린 시간 (참고용 — 실패 안 함): 실기 형식 48k 점 · stride 4 / 팔이 화면 가득 (점 대부분 몸 구 안) · stride 1
TEST(BodyFilterCudaBench, Timing) {
  BodyFilterCuda gpu;
  struct B { const char * name; size_t n; size_t stride; double inside; };
  const B benches[] = {{"실기 48k·stride4·근처30%", 48281, 4, 0.3}, {"팔 가득 48k·stride4·근처90%", 48281, 4, 0.9},
                       {"조밀 230k·stride1·근처60%", 230400, 1, 0.6}};
  for (const auto & b : benches) {
    const Scene s = make_scene(42, b.n, 16, 0, 4, 8, b.stride, 77, b.inside, true);
    std::vector<uint8_t> m;
    const int reps = 30;
    classify_body_cpu(s.job, m);
    auto w0 = std::chrono::steady_clock::now();
    double c0 = cpu_now();
    for (int r = 0; r < reps; ++r) classify_body_cpu(s.job, m);
    const double cpu_wall = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - w0).count() / reps;
    const double cpu_cpu = (cpu_now() - c0) * 1e3 / reps;
    double gpu_wall = -1, gpu_cpu = -1;
    if (gpu.available() && gpu.classify(s.job, m)) {
      w0 = std::chrono::steady_clock::now();
      c0 = cpu_now();
      for (int r = 0; r < reps; ++r) gpu.classify(s.job, m);
      gpu_wall = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - w0).count() / reps;
      gpu_cpu = (cpu_now() - c0) * 1e3 / reps;
    }
    printf("[bench] %s: CPU판 %.2f ms (CPU %.2f ms) · GPU판 %.2f ms (CPU %.2f ms)\n", b.name, cpu_wall, cpu_cpu, gpu_wall, gpu_cpu);
  }
}
