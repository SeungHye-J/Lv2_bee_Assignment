// fake_camera 구현 — 설명·선언은 include/faker/fake_camera.hpp, main은 fake_camera_main.cpp
#include "faker/fake_camera.hpp"

namespace faker {

FakeCamera::FakeCamera()
: rclcpp::Node("fake_camera") {
  width_ = declare_parameter<int>("width", 640);
  height_ = declare_parameter<int>("height", 480);
  const double period = declare_parameter<double>("period_s", 1.0 / 15);
  const double hfov = declare_parameter<double>("hfov_deg", 69.0) * M_PI / 180.0;
  fx_ = width_ / 2.0 / std::tan(hfov / 2.0);
  cloud_step_ = declare_parameter<int>("cloud_step", 4);
  const auto dir = declare_parameter<std::string>("live_dir", "/ws/debug/input-live");
  color_path_ = dir + "/" + get_name() + ".png";
  depth_path_ = dir + "/" + get_name() + "_depth.png";
  frame_ = std::string(get_name()) + "_color_optical_frame";
  const auto qos = rclcpp::SensorDataQoS();
  color_pub_ = create_publisher<Image>("~/color/image_raw", qos);
  info_pub_ = create_publisher<CameraInfo>("~/color/camera_info", qos);
  depth_pub_ = create_publisher<Image>("~/aligned_depth_to_color/image_raw", qos);
  points_pub_ = create_publisher<PointCloud2>("~/depth/color/points", qos);
  timer_ = create_wall_timer(std::chrono::duration<double>(period), [this]() { tick(); });
}

cv::Mat FakeCamera::load(const std::string & path, Live & live, int flags, int interp) {
  std::error_code ec;
  const auto mtime = fs::last_write_time(path, ec);
  if (ec) { live.mat.release(); return live.mat; }
  if (mtime != live.mtime) {
    cv::Mat img = cv::imread(path, flags);
    if (!img.empty()) {  // 읽기 실패(교체 순간)면 직전 장면 유지
      if (img.cols != width_ || img.rows != height_) cv::resize(img, img, cv::Size(width_, height_), 0, 0, interp);
      live.mat = img;
      live.mtime = mtime;
    }
  }
  return live.mat;
}

void FakeCamera::tick() {
  std_msgs::msg::Header h;
  h.stamp = now();
  h.frame_id = frame_;

  Image color;
  color.header = h;
  color.width = width_;
  color.height = height_;
  color.encoding = "rgb8";
  color.step = width_ * 3;
  color.data.assign(color.step * height_, 128);
  const cv::Mat scene = load(color_path_, color_live_, cv::IMREAD_COLOR, cv::INTER_LINEAR);
  if (!scene.empty()) {
    cv::Mat rgb(height_, width_, CV_8UC3, color.data.data());  // 메시지 버퍼에 바로 씀
    cv::cvtColor(scene, rgb, cv::COLOR_BGR2RGB);
  }
  color_pub_->publish(color);

  Image depth;
  depth.header = h;
  depth.width = width_;
  depth.height = height_;
  depth.encoding = "16UC1";  // mm
  depth.step = width_ * 2;
  depth.data.resize(depth.step * height_);
  cv::Mat d(height_, width_, CV_16UC1, depth.data.data());
  const cv::Mat live_depth = load(depth_path_, depth_live_, cv::IMREAD_ANYDEPTH, cv::INTER_NEAREST);
  if (live_depth.type() == CV_16UC1) live_depth.copyTo(d);
  else d.setTo(500);
  depth_pub_->publish(depth);

  CameraInfo info;
  info.header = h;
  info.width = width_;
  info.height = height_;
  info.distortion_model = "plumb_bob";
  info.d = {0, 0, 0, 0, 0};
  const double cx = width_ / 2.0, cy = height_ / 2.0;
  info.k = {fx_, 0, cx, 0, fx_, cy, 0, 0, 1};
  info.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  info.p = {fx_, 0, cx, 0, 0, fx_, cy, 0, 0, 0, 1, 0};
  info_pub_->publish(info);

  // depth → 점구름 (optical 프레임: z 앞 · x 오른쪽 · y 아래). 0(측정 안 됨)은 뺀다
  PointCloud2 cloud;
  cloud.header = h;
  PointCloud2Modifier mod(cloud);
  mod.setPointCloud2FieldsByString(1, "xyz");
  mod.resize((width_ / cloud_step_) * (height_ / cloud_step_));
  size_t n = 0;
  {
    PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
    for (int v = 0; v + cloud_step_ <= height_; v += cloud_step_) {
      for (int u = 0; u + cloud_step_ <= width_; u += cloud_step_) {
        const uint16_t mm = d.at<uint16_t>(v, u);
        if (mm == 0) continue;
        const float zm = mm / 1000.0f;
        *x = static_cast<float>((u - cx) * zm / fx_);
        *y = static_cast<float>((v - cy) * zm / fx_);
        *z = zm;
        ++x, ++y, ++z, ++n;
      }
    }
  }
  mod.resize(n);
  points_pub_->publish(cloud);
}

}  // namespace faker
