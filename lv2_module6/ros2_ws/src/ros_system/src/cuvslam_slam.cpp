// cuvslam_slam C++판 (11-16) — scripts/cuvslam_slam.py와 같은 일 · 같은 노드 이름 · 같은 파라미터 · 같은 토픽
//   cuVSLAM C++ API(cuvslam2.h)를 바로 부르고, 영상은 메시지에서 GPU 장치 메모리로 바로 올려(cudaMemcpy2D) 가까운 depth
//   마스크도 GPU 커널로 만든 뒤 GPU 포인터로 넘김 (is_gpu_mem) — 버퍼는 두 벌을 번갈아 씀 (앞 프레임을 cuVSLAM이 아직 읽어도 안전)
//   파이썬판은 프레임마다 numpy 변환 · 마스크 · map→odom 계산이 인터프리터에서 돌아 CPU 몰림에 1 s씩 멈췄음
// 설명(MapGate · initialpose · 도면 보정 · BEE_CUVSLAM_LEVEL · BEE_MAP_FREEZE · BEE_CUVSLAM_VIS)은 파이썬판 머리말·주석과 같음
#include <cuda_runtime.h>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "cuvslam2.h"
#include "cuvslam_gpu.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "message_filters/subscriber.hpp"
#include "message_filters/sync_policies/approximate_time.hpp"
#include "message_filters/synchronizer.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros_system/slam_math.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

using ros_system::Pose2;
using Image = sensor_msgs::msg::Image;

namespace {

bool env_on(const char * k) {
  const char * v = std::getenv(k);
  return v && std::string(v) == "1";
}

Eigen::Matrix4d mat(const cuvslam::Pose & p) {
  return ros_system::mat(p.rotation[0], p.rotation[1], p.rotation[2], p.rotation[3], p.translation[0], p.translation[1],
                         p.translation[2]);
}

Eigen::Matrix4d mat(const geometry_msgs::msg::Transform & t) {
  return ros_system::mat(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w, t.translation.x, t.translation.y,
                         t.translation.z);
}

// 영상 버퍼 하나 — gpu면 장치 메모리(cudaMallocPitch, 메시지에서 cudaMemcpy2D로 바로 올림), 아니면 보통 메모리
//   (mapped pinned는 cuVSLAM이 GPU 포인터로 안 봄 — cudaPointerGetAttributes가 Host · Orin은 concurrentManagedAccess=0이라 managed도 안 씀)
struct Buf {
  void * p = nullptr;
  size_t pitch = 0;  // 행 바이트 (여백 포함)
  uint32_t w = 0, h = 0;
  size_t bpp = 0;
  bool gpu = false;
  std::vector<uint8_t> host;

  bool fits(uint32_t W, uint32_t H, size_t B, bool G) const { return p && w == W && h == H && bpp == B && gpu == G; }
  bool alloc(uint32_t W, uint32_t H, size_t B, bool G) {
    release();
    w = W;
    h = H;
    bpp = B;
    if (G && cudaMallocPitch(&p, &pitch, W * B, H) == cudaSuccess) {
      gpu = true;
      return true;
    }
    p = nullptr;
    gpu = false;
    pitch = W * B;
    host.assign(pitch * H, 0);
    p = host.data();
    return !G;
  }
  void release() {
    if (gpu && p) cudaFree(p);
    p = nullptr;
    gpu = false;
    host.clear();
  }
  ~Buf() { release(); }
};

}  // namespace

class CuvslamSlam : public rclcpp::Node {
public:
  CuvslamSlam() : Node("cuvslam_slam") {
    base_ = declare_parameter<std::string>("base_frame", "base_footprint");
    odom_ = declare_parameter<std::string>("odom_frame", "odom");
    map_ = declare_parameter<std::string>("map_frame", "map");
    publish_map_tf_ = declare_parameter<bool>("publish_map_tf", true);
    if (env_on("BEE_CUVSLAM_NO_MAP_TF")) publish_map_tf_ = false;
    map_tf_tolerance_ = rclcpp::Duration::from_seconds(declare_parameter<double>("map_tf_tolerance", 1.0));
    depth_scale_ = declare_parameter<double>("depth_scale_factor", 1000.0);
    min_depth_ = declare_parameter<double>("min_depth", 0.5);
    frame_skip_ = std::max<int64_t>(1, declare_parameter<int64_t>("frame_skip", 1));
    near_raw_ = ros_system::near_limit(min_depth_, depth_scale_);
    max_map_size_ = static_cast<uint32_t>(declare_parameter<int64_t>("max_map_size", 150));
    lin_var_ = declare_parameter<double>("linear_variance", 0.0025);
    ang_var_ = declare_parameter<double>("angular_variance", 0.0025);
    // 영상을 GPU 장치 메모리로 넘길지 (메시지 → cudaMemcpy2D · 마스크 커널) — false면 CPU 버퍼 (cuVSLAM이 안에서 복사)
    gpu_input_ = declare_parameter<bool>("gpu_input", true);
    if (gpu_input_ && cudaStreamCreateWithFlags(&stream_, cudaStreamNonBlocking) != cudaSuccess) {
      RCLCPP_WARN(get_logger(), "CUDA 스트림 못 만듦 — CPU 입력으로");
      gpu_input_ = false;
    }
    const double jm = declare_parameter<double>("gate_jump_m", 0.30), jd = declare_parameter<double>("gate_jump_deg", 15.0);
    const double rm = declare_parameter<double>("gate_rate_m", 0.05), rd = declare_parameter<double>("gate_rate_deg", 5.0);
    const double ss = declare_parameter<double>("gate_settle_s", 15.0);
    if (declare_parameter<bool>("map_gate", true)) gate_.emplace(jm, jd, rm, rd, ss);
    // [rad/s] odom 회전 속도가 이보다 크면 map→odom을 그대로 둠 (회전 중 보정 멈춤, 11-16) · 0 = 안 멈춤
    hold_wz_ = declare_parameter<double>("gate_hold_wz", 0.2);
    if (env_on("BEE_MAP_FREEZE")) {
      const double inf = std::numeric_limits<double>::infinity();
      gate_.emplace(inf, inf, 0.0, 0.0, 0.0);
      RCLCPP_WARN(get_logger(), "BEE_MAP_FREEZE=1 — map→odom 보정 끔 (출발 값에 고정)");
    }
    level_ = env_on("BEE_CUVSLAM_LEVEL");
    vis_ = env_on("BEE_CUVSLAM_VIS");

    tf_buf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buf_, this, true);  // 따로 스레드 — 콜백 안에서 TF를 읽어도 안 막힘
    tf_pub_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    // map_tf_relay (11-16): true면 map→odom을 TF 대신 토픽으로만 — map_tf_relay(C++)가 30 Hz로 계속 TF로 냄
    //   토픽은 파이썬판·map_tf_relay와 같은 /cuvslam/map_to_odom (예전 /cuvslam/map_odom이면 relay가 못 받아 map→odom TF가 없었음)
    relay_ = declare_parameter<bool>("map_tf_relay", true);
    map_odom_pub_ = create_publisher<geometry_msgs::msg::TransformStamped>(
      declare_parameter<std::string>("map_to_odom_topic", "/cuvslam/map_to_odom"), rclcpp::QoS(1).reliable().transient_local());
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(declare_parameter<std::string>("odom_topic", "/cuvslam/odom"), 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/cuvslam/path", 1);
    path_.header.frame_id = map_;

    init_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      declare_parameter<std::string>("initialpose_topic", "/initialpose"), 10,
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr m) { on_initialpose(*m); });
    corr_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      declare_parameter<std::string>("map_correction_topic", "/localization/map_correction"), 10,
      [this](std_msgs::msg::Float64MultiArray::ConstSharedPtr m) {
        if (m->data.size() < 3) return;
        map_offset_ = ros_system::apply_map_correction(map_offset_, {m->data[0], m->data[1], m->data[2]});
        RCLCPP_INFO(get_logger(), "도면 보정 받음 → map 기준 옮김 (%+.3f, %+.3f, %+.2f°)", m->data[0], m->data[1],
                    m->data[2] * 180.0 / M_PI);
      });
    const auto sensor = rclcpp::SensorDataQoS();
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      declare_parameter<std::string>("camera_info", "/pole_front_cam/color/camera_info"), sensor,
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr m) { info_ = m; });
    color_sub_ = std::make_shared<message_filters::Subscriber<Image>>(
      this, declare_parameter<std::string>("rgb", "/pole_front_cam/color/image_raw"), sensor);
    depth_sub_ = std::make_shared<message_filters::Subscriber<Image>>(
      this, declare_parameter<std::string>("depth", "/pole_front_cam/aligned_depth_to_color/image_raw"), sensor);
    sync_ = std::make_shared<Sync>(Policy(10), *color_sub_, *depth_sub_);
    sync_->getPolicy()->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.02));
    sync_->registerCallback(
      [this](const Image::ConstSharedPtr & c, const Image::ConstSharedPtr & d) { on_rgbd(*c, *d); });

    stats_ = create_wall_timer(std::chrono::seconds(10), [this]() {
      std::string s = "추적 " + std::to_string(ok_) + " · 잃음 " + std::to_string(lost_);
      if (publish_map_tf_) s += " · 경로 " + std::to_string(path_.poses.size());
      if (gate_)
        s += " · 보정 거름 " + std::to_string(gate_->rejected) + " · 회전 중 멈춤 " + std::to_string(gate_->held_rot) +
             (gate_->locked ? " · 잠금" : "");
      s += gpu_used_ ? " · GPU 입력" : " · CPU 입력";
      RCLCPP_INFO(get_logger(), "%s", s.c_str());
    });
    if (vis_) {
      vis_obs_ = create_publisher<Image>("/cuvslam/vis/observations_image", 1);
      vis_lm_ = create_publisher<sensor_msgs::msg::PointCloud2>("/cuvslam/vis/landmarks_cloud", 1);
      vis_map_ = create_publisher<sensor_msgs::msg::PointCloud2>("/cuvslam/vis/map_landmarks_cloud", 1);
      vis_loop_ = create_publisher<sensor_msgs::msg::PointCloud2>("/cuvslam/vis/loop_closure_cloud", 1);
      vis_graph_ = create_publisher<visualization_msgs::msg::MarkerArray>("/cuvslam/vis/pose_graph", 1);
      vis_timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / declare_parameter<double>("vis_rate_hz", 2.0)),
                                     [this]() { publish_vis(); });
      RCLCPP_INFO(get_logger(), "BEE_CUVSLAM_VIS=1 — /cuvslam/vis/* 냄 (특징점 영상 · 랜드마크 · 포즈 그래프)");
    }
  }

private:
  using Policy = message_filters::sync_policies::ApproximateTime<Image, Image>;
  using Sync = message_filters::Synchronizer<Policy>;

  // RViz 2D Pose Estimate — cuVSLAM은 리셋하지 않고 map 기준에 offset (추적은 계속)
  void on_initialpose(const geometry_msgs::msg::PoseWithCovarianceStamped & m) {
    if (!m.header.frame_id.empty() && m.header.frame_id != map_) {
      RCLCPP_WARN(get_logger(), "initialpose frame '%s' 무시 — %s만 지원", m.header.frame_id.c_str(), map_.c_str());
      return;
    }
    const auto & q = m.pose.pose.orientation;
    const Pose2 desired{m.pose.pose.position.x, m.pose.pose.position.y,
                        std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))};
    if (!last_base_) {
      pending_initialpose_ = desired;
      RCLCPP_INFO(get_logger(), "initialpose 대기 — 첫 추적 프레임 뒤 적용");
      return;
    }
    apply_initialpose(desired);
  }

  void apply_initialpose(const Pose2 & desired) {
    if (gate_) gate_->reset();  // 사람이 위치를 다시 맞춤 — 잠금 풀고 새 보정부터
    map_offset_ = ros_system::compose(desired, ros_system::inverse(*last_base_));
    path_.poses.clear();
    RCLCPP_WARN(get_logger(), "initialpose 적용: map (%+.2f, %+.2f, %+.1f°)", desired.x, desired.y, desired.yaw * 180.0 / M_PI);
  }

  // 첫 프레임: camera_info로 rig, URDF TF로 base_footprint←카메라. 단독 SLAM이면 루프 클로저(평면 제약) 켬
  void start(const std::string & frame) {
    geometry_msgs::msg::TransformStamped t;
    try {
      t = tf_buf_->lookupTransform(base_, frame, tf2::TimePointZero);
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "TF %s←%s 대기: %s", base_.c_str(), frame.c_str(), e.what());
      return;
    }
    base_from_cam_ = mat(t.transform);
    const auto & k = info_->k;
    cuvslam::Camera cam;
    cam.size = {static_cast<int32_t>(info_->width), static_cast<int32_t>(info_->height)};
    cam.principal = {static_cast<float>(k[2]), static_cast<float>(k[5])};
    cam.focal = {static_cast<float>(k[0]), static_cast<float>(k[4])};
    cam.distortion.model = cuvslam::Distortion::Model::Pinhole;
    // BEE_CUVSLAM_LEVEL=1: rig를 바닥과 수평(OpenCV 축)으로 두고 카메라는 그 안에서 숙인 것으로 — 이후 base_from_cam_ = base_from_rig
    if (level_) {
      Eigen::Matrix4d base_from_rig = Eigen::Matrix4d::Identity();
      base_from_rig.block<3, 3>(0, 0) << 0, 0, 1, -1, 0, 0, 0, -1, 0;
      base_from_rig.block<3, 1>(0, 3) = base_from_cam_.block<3, 1>(0, 3);
      const Eigen::Matrix4d rig_from_cam = base_from_rig.inverse() * base_from_cam_;
      const Eigen::Quaterniond q(Eigen::Matrix3d(rig_from_cam.block<3, 3>(0, 0)));
      cam.rig_from_camera.rotation = {static_cast<float>(q.x()), static_cast<float>(q.y()), static_cast<float>(q.z()),
                                      static_cast<float>(q.w())};
      cam.rig_from_camera.translation = {static_cast<float>(rig_from_cam(0, 3)), static_cast<float>(rig_from_cam(1, 3)),
                                         static_cast<float>(rig_from_cam(2, 3))};
      base_from_cam_ = base_from_rig;
      RCLCPP_WARN(get_logger(), "BEE_CUVSLAM_LEVEL=1 — rig 수평, 카메라는 rig 안에서 숙임 (rig_from_camera)");
    }
    cuvslam::Rig rig;
    rig.cameras.push_back(cam);
    cuvslam::Odometry::Config oc;
    oc.odometry_mode = cuvslam::Odometry::OdometryMode::RGBD;
    oc.async_sba = true;
    oc.rgbd_settings.depth_scale_factor = static_cast<float>(depth_scale_);
    oc.rgbd_settings.depth_camera_id = 0;
    oc.rgbd_settings.enable_depth_stereo_tracking = false;
    // SLAM은 Odometry 상태(관측·랜드마크)가 있어야 함 — 파이썬 Tracker도 SLAM이면 켬
    oc.enable_observations_export = oc.enable_landmarks_export = publish_map_tf_ || vis_;
    try {
      odom_trk_ = std::make_unique<cuvslam::Odometry>(rig, oc);
      if (publish_map_tf_) {
        cuvslam::Slam::Config sc;
        sc.planar_constraints = true;
        sc.max_map_size = max_map_size_;
        sc.enable_reading_internals = vis_;  // get_slam_landmarks · get_pose_graph
        slam_ = std::make_unique<cuvslam::Slam>(rig, odom_trk_->GetPrimaryCameras(), sc);
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "cuVSLAM 시작 실패: %s", e.what());
      odom_trk_.reset();
      slam_.reset();
      return;
    }
    const auto ver = cuvslam::GetVersion(nullptr, nullptr, nullptr);
    RCLCPP_INFO(get_logger(), "cuVSLAM %.*s 시작 (%s) · %ux%u · 카메라 %s%s", static_cast<int>(ver.size()), ver.data(),
                slam_ ? "단독 SLAM" : "시각 오도메트리", info_->width, info_->height, frame.c_str(),
                frame_skip_ > 1 ? (" · " + std::to_string(frame_skip_) + "장에 1장 추적").c_str() : "");
  }

  // 메시지 한 장 → 버퍼 (GPU면 cudaMemcpy2D가 행 여백까지 처리). bpp = 픽셀당 바이트
  bool copy_in(const Image & m, Buf & b, size_t bpp) {
    const size_t row = static_cast<size_t>(m.width) * bpp;
    if (m.height == 0 || m.width == 0 || m.step < row || m.data.size() < static_cast<size_t>(m.height) * m.step) return false;
    if (m.is_bigendian && bpp > 1) return false;
    if (!b.fits(m.width, m.height, bpp, gpu_input_) && !b.alloc(m.width, m.height, bpp, gpu_input_))
      RCLCPP_WARN(get_logger(), "GPU 메모리 못 잡음 — CPU 버퍼로 (cuVSLAM이 안에서 복사)");
    if (b.gpu) return cudaMemcpy2DAsync(b.p, b.pitch, m.data.data(), m.step, row, m.height, cudaMemcpyHostToDevice, stream_) ==
                      cudaSuccess;
    auto * dst = static_cast<uint8_t *>(b.p);
    for (uint32_t r = 0; r < m.height; ++r) std::memcpy(dst + r * b.pitch, m.data.data() + r * m.step, row);
    return true;
  }

  static cuvslam::Image as_image(const Buf & b, int ch, cuvslam::ImageData::DataType dt, int64_t ns) {
    cuvslam::Image im;
    im.pixels = b.p;
    im.width = static_cast<int32_t>(b.w);
    im.height = static_cast<int32_t>(b.h);
    im.pitch = static_cast<int32_t>(b.pitch);
    im.encoding = ch == 3 ? cuvslam::ImageData::Encoding::RGB : cuvslam::ImageData::Encoding::MONO;
    im.data_type = dt;
    im.is_gpu_mem = b.gpu;
    im.timestamp_ns = ns;
    im.camera_index = 0;
    return im;
  }

  void on_rgbd(const Image & c, const Image & d) {
    if (!info_) return;
    if (frame_skip_ > 1 && (++frame_n_ % frame_skip_)) return;
    if (!odom_trk_) {
      start(c.header.frame_id);
      if (!odom_trk_) return;
    }
    if (c.height != d.height || c.width != d.width) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "컬러 %ux%u ≠ depth %ux%u", c.width, c.height, d.width, d.height);
      return;
    }
    const int64_t ns = rclcpp::Time(c.header.stamp).nanoseconds();
    if (ns <= last_ns_) return;  // cuVSLAM은 시각이 늘 늘어나야 함
    last_ns_ = ns;
    const int ch = (c.encoding == "rgb8" || c.encoding == "bgr8") ? 3 : 1;
    if (ch == 1 && c.encoding != "mono8") {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "컬러 인코딩 %s 지원 안 함 (rgb8 · bgr8 · mono8)", c.encoding.c_str());
      return;
    }
    if (d.encoding != "16UC1" && d.encoding != "mono16") {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "depth 인코딩 %s 지원 안 함 (16UC1)", d.encoding.c_str());
      return;
    }
    auto & B = bufs_[flip_ ^= 1];  // 두 벌을 번갈아 — 앞 프레임 버퍼는 건드리지 않음
    if (!copy_in(c, B.rgb, ch) || !copy_in(d, B.depth, 2)) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "영상 크기·step이 맞지 않음 (잘린 버퍼 · big endian · GPU 복사 실패)");
      return;
    }
    // 마스크도 depth와 같은 쪽에서 — GPU면 커널, 아니면 CPU (cuVSLAM은 한 번에 같은 종류만 받음)
    const bool gpu = B.rgb.gpu && B.depth.gpu;
    if (!B.mask.fits(c.width, c.height, 1, gpu)) B.mask.alloc(c.width, c.height, 1, gpu);
    if (gpu && B.mask.gpu) {
      if (near_mask_gpu(B.depth.p, B.depth.pitch, B.mask.p, B.mask.pitch, c.width, c.height, near_raw_, stream_) != cudaSuccess ||
          cudaStreamSynchronize(stream_) != cudaSuccess) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "GPU 마스크 실패: %s", cudaGetErrorString(cudaGetLastError()));
        return;
      }
    } else {
      for (uint32_t r = 0; r < c.height; ++r)
        ros_system::near_mask(reinterpret_cast<const uint16_t *>(static_cast<const uint8_t *>(B.depth.p) + r * B.depth.pitch),
                              static_cast<uint8_t *>(B.mask.p) + r * B.mask.pitch, c.width, near_raw_);
    }
    gpu_used_ = gpu && B.mask.gpu;
    if (B.rgb.gpu != gpu_used_ || B.depth.gpu != gpu_used_) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "영상·depth·마스크 버퍼 종류가 섞임 (GPU/CPU) — 이 프레임 건너뜀");
      return;
    }
    using DT = cuvslam::ImageData::DataType;
    const cuvslam::Odometry::ImageSet imgs{as_image(B.rgb, ch, DT::UINT8, ns)};
    const cuvslam::Odometry::ImageSet masks{as_image(B.mask, 1, DT::UINT8, ns)};
    const cuvslam::Odometry::ImageSet depths{as_image(B.depth, 1, DT::UINT16, ns)};
    cuvslam::PoseEstimate est;
    std::optional<cuvslam::Pose> slam_pose;
    try {
      est = odom_trk_->Track(imgs, masks, depths);
      if (slam_ && est.world_from_rig) {
        cuvslam::Odometry::State st;
        odom_trk_->GetState(st);
        slam_->Track(st);
        slam_pose = slam_->GetPose();
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "cuVSLAM Track 실패: %s", e.what());
      return;
    }
    if (vis_) {
      last_vis_ = LastVis{c.header.stamp, ch, c.width, c.height, {}};
      copy_gray(c, ch, last_vis_->gray);
    }
    if (!est.world_from_rig) {
      ++lost_;
      prev_.reset();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "추적 잃음");
      return;
    }
    ++ok_;
    const auto & p = est.world_from_rig->pose;
    const Pose2 cur = ros_system::base_pose(base_from_cam_, mat(p));
    if (prev_) publish_odom(c.header.stamp, ros_system::body_twist(prev_->second, cur, (ns - prev_->first) * 1e-9));
    prev_ = std::make_pair(ns, cur);
    last_base_ = cur;
    if (pending_initialpose_) {
      apply_initialpose(*pending_initialpose_);
      pending_initialpose_.reset();
    }
    if (publish_map_tf_) publish_map(c.header.stamp, slam_pose ? *slam_pose : p);  // 루프 클로저 반영된 SLAM 위치 우선
  }

  // 속도만 (자세는 안 씀 — cuVSLAM 좌표는 추적을 잃으면 새로 시작해 자세를 EKF에 그대로 넣으면 튐)
  void publish_odom(const builtin_interfaces::msg::Time & stamp, const Pose2 & v) {
    nav_msgs::msg::Odometry o;
    o.header.stamp = stamp;
    o.header.frame_id = odom_;
    o.child_frame_id = base_;
    o.twist.twist.linear.x = v.x;
    o.twist.twist.linear.y = v.y;
    o.twist.twist.angular.z = v.yaw;
    o.twist.covariance[0] = o.twist.covariance[7] = lin_var_;
    o.twist.covariance[35] = ang_var_;
    o.twist.covariance[14] = o.twist.covariance[21] = o.twist.covariance[28] = 1e6;  // 안 쓰는 축
    odom_pub_->publish(o);
  }

  void publish_map(const builtin_interfaces::msg::Time & stamp, const cuvslam::Pose & p) {
    geometry_msgs::msg::TransformStamped o;
    try {
      o = tf_buf_->lookupTransform(odom_, base_, rclcpp::Time(stamp));
    } catch (const std::exception &) {  // 그 시각 odom이 없으면 이 프레임은 TF만 건너뜀 (추적은 계속)
      return;
    }
    const Pose2 odom = ros_system::planar(mat(o.transform));
    const Pose2 base_raw = ros_system::base_pose(base_from_cam_, mat(p));
    last_base_ = base_raw;
    Pose2 base = ros_system::compose(map_offset_, base_raw);
    Pose2 mo = ros_system::map_to_odom(base, odom);
    const double ts = rclcpp::Time(stamp).seconds();
    bool rotating = false;
    if (prev_odom_ && ts > prev_odom_->first) {
      const double wz = std::remainder(odom.yaw - prev_odom_->second.yaw, 2 * M_PI) / (ts - prev_odom_->first);
      rotating = hold_wz_ > 0.0 && std::abs(wz) > hold_wz_;
    }
    prev_odom_ = std::make_pair(ts, odom);
    if (gate_) {
      bool accepted = false;
      mo = gate_->update(mo, ts, odom, accepted, rotating);
      if (gate_->resettled) {
        RCLCPP_WARN(get_logger(), "map→odom 출발 정착 중 다시 맞춤 — 보정이 %.2f m · %.1f° 바뀜 (gate_settle_s 안이라 잠그지 않고 받음 · "
                                  "map = cuVSLAM 출발 자리 기준)", gate_->resettled->x, gate_->resettled->y);
        gate_->resettled.reset();
      }
      if (!accepted) {
        if (gate_->locked)
          RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                                "map→odom 보정 잠금 — cuVSLAM 위치가 크게 튐, 바퀴+IMU로만 이어 감 (/initialpose로 다시 맞추면 풀림)");
        base = ros_system::compose(mo, odom);  // 낸 보정 기준 로봇 자세 (경로 표시)
      }
    }
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = rclcpp::Time(stamp) + map_tf_tolerance_;
    tf.header.frame_id = map_;
    tf.child_frame_id = odom_;
    tf.transform.translation.x = mo.x;
    tf.transform.translation.y = mo.y;
    tf.transform.rotation.z = std::sin(mo.yaw / 2);
    tf.transform.rotation.w = std::cos(mo.yaw / 2);
    if (relay_) map_odom_pub_->publish(tf);
    else tf_pub_->sendTransform(tf);
    geometry_msgs::msg::PoseStamped ps;
    ps.header = tf.header;
    ps.pose.position.x = base.x;
    ps.pose.position.y = base.y;
    ps.pose.orientation.z = std::sin(base.yaw / 2);
    ps.pose.orientation.w = std::cos(base.yaw / 2);
    path_.header.stamp = stamp;
    path_.poses.push_back(ps);
    if (path_.poses.size() > 300) path_.poses.erase(path_.poses.begin(), path_.poses.end() - 300);  // 최근 300개만 (보기용)
    if (ok_ % 5 == 0 && path_pub_->get_subscription_count() > 0) path_pub_->publish(path_);
  }

  // BEE_CUVSLAM_VIS: 특징점(영상 위 점) · 그 프레임 랜드마크 · SLAM 지도 랜드마크 · 루프 클로저 랜드마크 · 포즈 그래프
  void publish_vis() {
    if (!odom_trk_ || !last_vis_) return;
    const auto & lv = *last_vis_;
    // 회색 영상(컬러면 G 채널) → bgr8 + 특징점 초록 점 (3x3)
    Image img;
    img.header.stamp = lv.stamp;
    img.header.frame_id = "pole_front_cam_color_optical_frame";
    img.width = lv.w;
    img.height = lv.h;
    img.encoding = "bgr8";
    img.step = lv.w * 3;
    img.data.resize(static_cast<size_t>(img.step) * lv.h);
    for (size_t i = 0; i < lv.gray.size(); ++i) img.data[i * 3] = img.data[i * 3 + 1] = img.data[i * 3 + 2] = lv.gray[i];
    try {
      for (const auto & o : odom_trk_->GetLastObservations(0))
        for (int dy = -1; dy <= 1; ++dy)
          for (int dx = -1; dx <= 1; ++dx) {
            const int u = static_cast<int>(o.u) + dx, v = static_cast<int>(o.v) + dy;
            if (u < 0 || v < 0 || u >= static_cast<int>(lv.w) || v >= static_cast<int>(lv.h)) continue;
            auto * px = &img.data[(static_cast<size_t>(v) * lv.w + u) * 3];
            px[0] = 0;
            px[1] = 255;
            px[2] = 0;
          }
    } catch (const std::exception &) {
    }
    vis_obs_->publish(img);
    std_msgs::msg::Header hdr;
    hdr.stamp = lv.stamp;
    hdr.frame_id = map_;
    auto cloud = [&](const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pub, const std::vector<Eigen::Vector3d> & pts) {
      if (pts.empty()) return;
      sensor_msgs::msg::PointCloud2 pc;
      pc.header = hdr;
      sensor_msgs::PointCloud2Modifier mod(pc);
      mod.setPointCloud2FieldsByString(1, "xyz");
      mod.resize(pts.size());
      sensor_msgs::PointCloud2Iterator<float> x(pc, "x"), y(pc, "y"), z(pc, "z");
      for (const auto & p : pts) {
        const Eigen::Vector3d q = base_from_cam_.block<3, 3>(0, 0) * p + base_from_cam_.block<3, 1>(0, 3);
        *x = static_cast<float>(q.x());
        *y = static_cast<float>(q.y());
        *z = static_cast<float>(q.z());
        ++x;
        ++y;
        ++z;
      }
      pub->publish(pc);
    };
    try {
      std::vector<Eigen::Vector3d> lm;
      for (const auto & l : odom_trk_->GetLastLandmarks()) lm.emplace_back(l.coords[0], l.coords[1], l.coords[2]);
      cloud(vis_lm_, lm);
    } catch (const std::exception &) {
    }
    if (!slam_) return;
    using L = cuvslam::Slam::DataLayer;
    for (auto [layer, pub] : {std::pair{L::Map, vis_map_}, std::pair{L::LoopClosure, vis_loop_}}) {
      slam_->EnableReadingData(layer, 100000);  // 파이썬 바인딩과 같게 — 읽을 때마다 켬
      if (const auto ls = slam_->ReadLandmarks(layer)) {
        std::vector<Eigen::Vector3d> pts;
        for (const auto & l : ls->landmarks) pts.emplace_back(l.coords[0], l.coords[1], l.coords[2]);
        cloud(pub, pts);
      }
    }
    slam_->EnableReadingData(L::PoseGraph, 1000);
    const auto g = slam_->ReadPoseGraph();
    if (!g) return;
    std::map<uint64_t, Pose2> xy;
    for (const auto & n : g->nodes) xy[n.id] = ros_system::base_pose(base_from_cam_, mat(n.node_pose));
    visualization_msgs::msg::Marker nodes, edges;
    for (auto [mk, ns, typ, sc, r, gg, b] : {std::tuple{&nodes, "nodes", visualization_msgs::msg::Marker::SPHERE_LIST, 0.04, 1.0, 0.6, 0.0},
                                             std::tuple{&edges, "edges", visualization_msgs::msg::Marker::LINE_LIST, 0.01, 0.2, 0.8, 1.0}}) {
      mk->header = hdr;
      mk->ns = ns;
      mk->type = typ;
      mk->action = visualization_msgs::msg::Marker::ADD;
      mk->scale.x = mk->scale.y = mk->scale.z = sc;
      mk->color.r = static_cast<float>(r);
      mk->color.g = static_cast<float>(gg);
      mk->color.b = static_cast<float>(b);
      mk->color.a = 1.0f;
      mk->pose.orientation.w = 1.0;
    }
    auto pt = [](const Pose2 & p) {
      geometry_msgs::msg::Point q;
      q.x = p.x;
      q.y = p.y;
      q.z = 0.05;
      return q;
    };
    for (const auto & [id, p] : xy) nodes.points.push_back(pt(p));
    for (const auto & e : g->edges) {
      const auto a = xy.find(e.node_from), b = xy.find(e.node_to);
      if (a == xy.end() || b == xy.end()) continue;
      edges.points.push_back(pt(a->second));
      edges.points.push_back(pt(b->second));
    }
    visualization_msgs::msg::MarkerArray ma;
    ma.markers = {nodes, edges};
    vis_graph_->publish(ma);
  }

  std::string base_, odom_, map_;
  bool publish_map_tf_ = true, relay_ = true, gpu_input_ = true, gpu_used_ = false, level_ = false, vis_ = false;
  rclcpp::Duration map_tf_tolerance_{0, 0};
  double hold_wz_ = 0.2;
  std::optional<std::pair<double, Pose2>> prev_odom_;  // 직전 프레임 odom 자세 (회전 속도)
  double depth_scale_ = 1000.0, min_depth_ = 0.5, lin_var_ = 0.0025, ang_var_ = 0.0025;
  int64_t frame_skip_ = 1, frame_n_ = 0, last_ns_ = 0;
  uint16_t near_raw_ = 0;
  uint32_t max_map_size_ = 150;
  long ok_ = 0, lost_ = 0;
  std::optional<ros_system::MapGate> gate_;
  Eigen::Matrix4d base_from_cam_ = Eigen::Matrix4d::Identity();
  Pose2 map_offset_;
  std::optional<Pose2> last_base_, pending_initialpose_;
  std::optional<std::pair<int64_t, Pose2>> prev_;  // 직전 오도메트리 자세 — 추적을 잃으면 비움
  std::unique_ptr<cuvslam::Odometry> odom_trk_;
  std::unique_ptr<cuvslam::Slam> slam_;
  struct Bufs {
    Buf rgb, depth, mask;
  };
  cudaStream_t stream_ = nullptr;
  std::array<Bufs, 2> bufs_;
  int flip_ = 0;
  struct LastVis {
    builtin_interfaces::msg::Time stamp;
    int ch;
    uint32_t w, h;
    std::vector<uint8_t> gray;  // 보기용 회색 (컬러면 G 채널) — GPU 버퍼는 CPU에서 못 읽어 메시지에서 따로 둠
  };
  static void copy_gray(const Image & c, int ch, std::vector<uint8_t> & g) {
    g.resize(static_cast<size_t>(c.width) * c.height);
    for (uint32_t y = 0; y < c.height; ++y)
      for (uint32_t x = 0; x < c.width; ++x) g[y * c.width + x] = c.data[y * c.step + x * ch + (ch == 3 ? 1 : 0)];
  }
  std::optional<LastVis> last_vis_;
  nav_msgs::msg::Path path_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr info_;
  std::shared_ptr<tf2_ros::Buffer> tf_buf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr map_odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr corr_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  std::shared_ptr<message_filters::Subscriber<Image>> color_sub_, depth_sub_;
  std::shared_ptr<Sync> sync_;
  rclcpp::TimerBase::SharedPtr stats_, vis_timer_;
  rclcpp::Publisher<Image>::SharedPtr vis_obs_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr vis_lm_, vis_map_, vis_loop_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vis_graph_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CuvslamSlam>());
  rclcpp::shutdown();
  return 0;
}
