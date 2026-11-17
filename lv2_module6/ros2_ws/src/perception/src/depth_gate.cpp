// depth_gate 구현 — 설명은 include/perception/depth_gate.hpp
#include "perception/depth_gate.hpp"

#include <cstring>

namespace perception {

namespace {
template <typename T>
long zero_rows(sensor_msgs::msg::Image & img, T limit) {
  long n = 0;
  for (uint32_t r = 0; r < img.height; ++r) {
    T * row = reinterpret_cast<T *>(img.data.data() + size_t(r) * img.step);
    for (uint32_t c = 0; c < img.width; ++c)
      if (row[c] > T(0) && row[c] < limit) { row[c] = T(0); ++n; }
  }
  return n;
}
}  // namespace

long zero_near_depth(sensor_msgs::msg::Image & img, double min_range_m) {
  if (img.data.size() < size_t(img.step) * img.height) return -1;
  // 파이썬판과 같은 비교: 16UC1은 mm 정수 < min_range·1000, 32FC1은 m < min_range (NaN은 비교가 거짓 → 그대로)
  if (img.encoding == "16UC1" && img.step >= img.width * 2) return zero_rows<uint16_t>(img, uint16_t(min_range_m * 1000.0));
  if (img.encoding == "32FC1" && img.step >= img.width * 4) return zero_rows<float>(img, float(min_range_m));
  return -1;
}

DepthGate::DepthGate(const rclcpp::NodeOptions & options) : Node("pole_depth_gate", options) {
  const auto in = declare_parameter<std::string>("depth_in", "/pole_front_cam/aligned_depth_to_color/image_raw");
  const auto out = declare_parameter<std::string>("depth_out", "/pole_front_cam/aligned_depth_to_color/image_nvblox");
  min_range_ = declare_parameter<double>("depth_min_range", 0.5);  // [m] 0이면 그대로 넘김
  pub_ = create_publisher<sensor_msgs::msg::Image>(out, rclcpp::SensorDataQoS());
  sub_ = create_subscription<sensor_msgs::msg::Image>(in, rclcpp::SensorDataQoS(),
                                                     [this](sensor_msgs::msg::Image::UniquePtr m) { on_depth(std::move(m)); });
  log_timer_ = create_wall_timer(std::chrono::seconds(10), [this]() {
    RCLCPP_INFO(get_logger(), "넘김 %lu · 0으로 바꾼 픽셀 %lu (min_range %.2f m)", static_cast<unsigned long>(passed_),
                static_cast<unsigned long>(zeroed_px_), min_range_);
  });
  RCLCPP_INFO(get_logger(), "%s → %s · %.2f m 안 픽셀을 0으로 (C++)", in.c_str(), out.c_str(), min_range_);
}

void DepthGate::on_depth(sensor_msgs::msg::Image::UniquePtr msg) {
  if (min_range_ > 0.0) {
    const long n = zero_near_depth(*msg, min_range_);
    if (n > 0) zeroed_px_ += static_cast<uint64_t>(n);
  }
  ++passed_;
  pub_->publish(std::move(msg));
}

}  // namespace perception
