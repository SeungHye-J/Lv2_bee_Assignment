#pragma once

// 더미 RealSense: realsense2_camera(camera_namespace:='', camera_name:=<이름>) 대신 같은 토픽을 발행.
//   /<이름>/color/image_raw · color/camera_info · aligned_depth_to_color/image_raw (best_effort)
//   /<이름>/depth/color/points (PointCloud2, depth에서 cloud_step 픽셀 간격으로 만든 xyz — optical 프레임)
// 장면: 시뮬레이터(lv2_module6/simulator)가 로봇 카메라 시점으로 렌더해 쓰는 파일
//   <live_dir>/<이름>.png        컬러 (없으면 회색)
//   <live_dir>/<이름>_depth.png  16비트 depth [mm] (없으면 0.5m 평면). 0 = 측정 안 됨 (RealSense처럼)
//   파일이 바뀌면(mtime) 다음 프레임부터 새 장면 — module5 fake_camera와 같은 방식.
// 내부 파라미터는 hfov_deg(수평 화각)로 정한다 — 시뮬레이터도 같은 값으로 렌더해야 3D가 맞는다 (faker.yaml).
// launch에서 노드 이름을 pole_cam / gripper_cam 으로 띄운다 (~/ 토픽이 /pole_cam/... 이 됨).
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>

#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace faker {

using namespace sensor_msgs::msg;
using sensor_msgs::PointCloud2Iterator;
using sensor_msgs::PointCloud2Modifier;
namespace fs = std::filesystem;

class FakeCamera : public rclcpp::Node {
public:
  FakeCamera();

private:
  struct Live {
    cv::Mat mat;
    fs::file_time_type mtime{};
  };

  // 파일이 바뀌었을 때만 다시 읽는다. 없으면 빈 Mat. 크기는 width×height로 (depth는 보간 없이)
  cv::Mat load(const std::string & path, Live & live, int flags, int interp);

  void tick();

  int width_, height_, cloud_step_;
  double fx_;
  std::string frame_, color_path_, depth_path_;
  Live color_live_, depth_live_;
  rclcpp::Publisher<Image>::SharedPtr color_pub_, depth_pub_;
  rclcpp::Publisher<CameraInfo>::SharedPtr info_pub_;
  rclcpp::Publisher<PointCloud2>::SharedPtr points_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace faker
