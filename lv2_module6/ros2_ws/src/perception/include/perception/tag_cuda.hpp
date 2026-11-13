#pragma once

// tag_cuda — AprilTag(tag36h11)를 GPU로: NVIDIA cuAprilTags (Isaac ROS isaac_ros_nitros/cuapriltags_vendor)
//   왜: 젯슨에서 perception CPU 대부분이 OpenCV aruco 3번 검사(원본·감마·반 크기)였음 (11/14 실기 top: perception_node 57 %).
//       cuAprilTags는 한 번 검사로 감마·반 크기가 찾던 태그(test/data glare_tag0 · big_tag7)도 찾고 640×360 한 장 ≈ 2 ms.
//   isaac_ros_apriltag 노드를 따로 띄우지 않고 라이브러리만 씀 — 그 노드는 rectify 영상·별도 토픽이라 YOLO 네모와 시각을 다시 맞춰야 함.
//   꼭짓점은 aruco 순서로 바꿔 돌려줌 (cuAprilTags = aruco를 두 칸 돌린 순서, 11/14 회전 4가지로 확인) → tag_pnp·tag_yaw 그대로.
//   빌드: cuapriltags_vendor(~/isaac_ros_ws)와 CUDA가 있을 때만 GPU판 (PERCEPTION_CUAPRILTAGS) · 없으면 available() = false
//         → perception_node가 OpenCV aruco로 (PC 도커 · CI 그대로)
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "perception/perception_core.hpp"

namespace perception {

class CudaTagDetector {
public:
  CudaTagDetector();
  ~CudaTagDetector();
  CudaTagDetector(const CudaTagDetector &) = delete;
  CudaTagDetector & operator=(const CudaTagDetector &) = delete;
  bool available() const;     // GPU판으로 빌드됐고 GPU가 보이나
  std::string error() const;  // 못 쓰면 이유 (마지막 실패)
  // bgr: 컬러 영상 (크기가 바뀌면 검출기를 다시 만듦) · min_px: 이보다 둘레가 짧은 태그는 버림 (OpenCV tag_min_perimeter_px와 같은 뜻)
  // 실패하면 없음 → 부른 쪽이 OpenCV로
  std::optional<std::vector<Tag>> detect(const cv::Mat & bgr, std::optional<double> min_px = std::nullopt);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace perception
