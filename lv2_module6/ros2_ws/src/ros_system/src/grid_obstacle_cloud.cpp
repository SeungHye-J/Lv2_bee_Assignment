// grid_obstacle_cloud — 점유 격자(/arena/walls_only: 실측 벽만 — 상자는 approach가 붙어야 해서 뺌)에서 로봇 주변 radius_m 안의 막힌 칸을 점구름으로 (11-16)
//   collision_monitor의 두 번째 입력 — 앞 카메라는 옆·뒤를 못 봐서, 몸 옆이 벽 끝에 닿은 채 25 s 밀고 벽에 부딪혔음 (실기 11-16 02:49).
//   벽은 고정이라 도면 격자로 옆·뒤를 앎 (사람·새 물체는 카메라가 계속 봄). 출력 frame = base_footprint, z = z_m
#include <cmath>
#include <memory>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class GridObstacleCloud : public rclcpp::Node {
public:
  GridObstacleCloud() : Node("grid_obstacle_cloud"), buf_(get_clock()), tf_(buf_) {
    radius_ = declare_parameter<double>("radius_m", 1.0);
    z_ = declare_parameter<double>("z_m", 0.15);
    occ_ = declare_parameter<int>("occupied_min", 65);
    base_ = declare_parameter<std::string>("base_frame", "base_footprint");
    // 몸(base_footprint 기준 앞 0.069 · 뒤 −0.245 · 옆 ±0.151) + body_margin 안에 떨어진 도면 벽은 버림 — 로봇이 벽 안에 있을 수는 없으니
    //   위치 오차(실기 11-16: SLAM이 11 cm 틀어져 벽 끝이 몸 안 → collision_monitor가 모든 방향을 막아 approach_blocked)
    front_ = declare_parameter<double>("body_front_m", 0.069);
    rear_ = declare_parameter<double>("body_rear_m", -0.245);
    half_ = declare_parameter<double>("body_half_width_m", 0.151);
    margin_ = declare_parameter<double>("body_margin_m", 0.02);
    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(declare_parameter<std::string>("output_topic", "/arena/walls_cloud"),
                                                            rclcpp::SensorDataQoS().keep_last(1));
    sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      declare_parameter<std::string>("grid_topic", "/arena/walls_only"), rclcpp::QoS(1).reliable().transient_local(),
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m) { grid_ = m; });
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / declare_parameter<double>("rate_hz", 10.0)), [this]() { tick(); });
  }

private:
  void tick() {
    if (!grid_) return;
    geometry_msgs::msg::TransformStamped t;
    try {
      t = buf_.lookupTransform(base_, grid_->header.frame_id, tf2::TimePointZero);  // 격자 → 로봇
    } catch (const tf2::TransformException &) {
      return;
    }
    // 로봇 자리(격자 좌표) — 역변환
    geometry_msgs::msg::TransformStamped inv;
    try {
      inv = buf_.lookupTransform(grid_->header.frame_id, base_, tf2::TimePointZero);
    } catch (const tf2::TransformException &) {
      return;
    }
    const auto & g = *grid_;
    const double res = g.info.resolution, ox = g.info.origin.position.x, oy = g.info.origin.position.y;
    const double rx = inv.transform.translation.x, ry = inv.transform.translation.y;
    const auto & q = t.transform.rotation;  // 평면 yaw (tf2::getYaw는 tf2_geometry_msgs 링크가 필요해 직접)
    const double yaw = std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)), c = std::cos(yaw), s = std::sin(yaw);
    const double tx = t.transform.translation.x, ty = t.transform.translation.y;
    const int r = static_cast<int>(std::ceil(radius_ / res));
    const int ci = static_cast<int>(std::floor((rx - ox) / res)), cj = static_cast<int>(std::floor((ry - oy) / res));
    std::vector<std::pair<float, float>> pts;
    for (int j = cj - r; j <= cj + r; ++j) {
      if (j < 0 || j >= static_cast<int>(g.info.height)) continue;
      for (int i = ci - r; i <= ci + r; ++i) {
        if (i < 0 || i >= static_cast<int>(g.info.width)) continue;
        if (g.data[static_cast<size_t>(j) * g.info.width + i] < occ_) continue;
        const double gx = ox + (i + 0.5) * res, gy = oy + (j + 0.5) * res;
        if (std::hypot(gx - rx, gy - ry) > radius_) continue;
        const double bx = c * gx - s * gy + tx, by = s * gx + c * gy + ty;
        if (bx < front_ + margin_ && bx > rear_ - margin_ && std::abs(by) < half_ + margin_) { ++dropped_; continue; }  // 몸 안 = 위치 오차
        pts.emplace_back(static_cast<float>(bx), static_cast<float>(by));
      }
    }
    sensor_msgs::msg::PointCloud2 out;
    out.header.stamp = now();
    out.header.frame_id = base_;
    sensor_msgs::PointCloud2Modifier mod(out);
    mod.setPointCloud2FieldsByString(1, "xyz");
    mod.resize(pts.size());
    sensor_msgs::PointCloud2Iterator<float> x(out, "x"), y(out, "y"), z(out, "z");
    for (const auto & p : pts) { *x = p.first; *y = p.second; *z = static_cast<float>(z_); ++x; ++y; ++z; }
    pub_->publish(out);
    if (dropped_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "도면 벽 %zu칸이 몸 안 — 위치 오차로 보고 뺌 (안전 입력에서 제외)", dropped_);
      dropped_ = 0;
    }
  }

  tf2_ros::Buffer buf_;
  tf2_ros::TransformListener tf_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr grid_;
  double front_, rear_, half_, margin_;
  size_t dropped_ = 0;
  double radius_ = 1.0, z_ = 0.15;
  int occ_ = 65;
  std::string base_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GridObstacleCloud>());
  rclcpp::shutdown();
  return 0;
}
