// nvblox_map C++판 (11-16) — scripts/nvblox_map.py와 같음 (설명은 그 머리말). 격자 계산은 include/ros_system/grid_ops.hpp
//   불러오기(load_path)는 nvblox_msgs가 있을 때만 빌드 (CMake NVBLOX_MSGS) — 없으면 load_path를 주면 오류 로그만
#include <chrono>
#include <memory>
#include <optional>
#include <string>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros_system/grid_ops.hpp"
#include "ros_system/helpers.hpp"
#ifdef ROS_SYSTEM_NVBLOX_MSGS
#include "nvblox_msgs/srv/file_path.hpp"
#endif

namespace ros_system {

using nav_msgs::msg::OccupancyGrid;

class NvbloxMap : public rclcpp::Node {
public:
  explicit NvbloxMap(const rclcpp::NodeOptions & options) : Node("nvblox_map", options) {
    frame_ = declare_parameter<std::string>("map_frame", "map");
    load_path_ = declare_parameter<std::string>("load_path", "");
    const double rate = declare_parameter<double>("rate_hz", 1.0);
    speckle_ = declare_parameter<int>("speckle_min_cells", 0);  // 11-15 예전으로 (끔) — 시험할 때 8
    const auto latched = rclcpp::QoS(1).reliable().transient_local();
    pub_ = create_publisher<OccupancyGrid>(declare_parameter<std::string>("map_out", "/map"), latched);
    // best effort로 구독 — 발행 쪽이 reliable이든 best effort든 받음
    grid_sub_ = create_subscription<OccupancyGrid>(
      declare_parameter<std::string>("grid_in", "/nvblox_node/static_occupancy_grid"), rclcpp::SensorDataQoS(),
      [this](OccupancyGrid::SharedPtr m) { on_grid(std::move(m)); });
    walls_sub_ = create_subscription<OccupancyGrid>(
      declare_parameter<std::string>("walls_in", "/arena/walls"), latched, [this](OccupancyGrid::SharedPtr m) {
        walls_ = std::move(m);
        fresh_ = true;
        RCLCPP_INFO(get_logger(), "경기장 벽 격자 받음 %ux%u — /map에 겹침", walls_->info.width, walls_->info.height);
      });
    tick_timer_ = create_timer(std::chrono::duration<double>(1.0 / rate), [this]() { tick(); });
    log_timer_ = create_timer(std::chrono::seconds(10), [this]() {
      RCLCPP_INFO(get_logger(), "격자 받음 %lu · /map 냄 %lu · 지운 잡음 점 %ld칸 (마지막 격자)%s", n_in_, n_out_, n_speckle_,
                  grid_ ? "" : " — nvblox 격자 아직 없음");
    });
    if (!load_path_.empty()) load();
  }

private:
  void on_grid(OccupancyGrid::SharedPtr m) {
    if (!m->header.frame_id.empty() && m->header.frame_id != frame_) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000, "격자 프레임 %s ≠ %s — nvblox global_frame 확인",
                            m->header.frame_id.c_str(), frame_.c_str());
      return;
    }
    grid_ = std::move(m);
    fresh_ = true;
    ++n_in_;
  }

  void tick() {
    if (!fresh_) return;
    fresh_ = false;
    std::optional<OccupancyGrid> clean;
    const OccupancyGrid * grid = grid_.get();
    if (grid != nullptr && speckle_ > 0) {
      clean = despeckle(*grid, speckle_, n_speckle_);
      grid = &*clean;
    }
    auto out = std::make_unique<OccupancyGrid>(walls_ ? overlay(*walls_, grid) : *grid);  // walls 없으면 fresh는 격자가 왔을 때만
    out->header.frame_id = frame_;
    pub_->publish(std::move(out));
    ++n_out_;
  }

  // nvblox_node가 뜰 때까지 1 s마다 봄 (최대 60 s) — 생성자에서 기다리면 같은 프로세스의 다른 노드(cpp_helpers)까지 멈춤
  void load() {
#ifdef ROS_SYSTEM_NVBLOX_MSGS
    using FilePath = nvblox_msgs::srv::FilePath;
    auto cli = create_client<FilePath>("/nvblox_node/load_map");
    load_timer_ = create_timer(std::chrono::seconds(1), [this, cli]() {
      if (!cli->service_is_ready()) {
        if (++load_wait_ >= 60) {
          load_timer_->cancel();
          RCLCPP_ERROR(get_logger(), "nvblox load_map 서비스 없음 — %s 못 불러옴 (빈 지도로 시작)", load_path_.c_str());
        }
        return;
      }
      load_timer_->cancel();
      auto req = std::make_shared<FilePath::Request>();
      req->file_path = load_path_;
      cli->async_send_request(req, [this](rclcpp::Client<FilePath>::SharedFuture f) {
        auto res = f.get();
        RCLCPP_INFO(get_logger(), "지도 불러오기 %s: %s", load_path_.c_str(),
                    res && res->success ? "성공" : "실패 — 빈 지도로 시작");
      });
    });
#else
    RCLCPP_ERROR(get_logger(), "nvblox_msgs 없이 빌드됨 — %s 못 불러옴 (빈 지도로 시작) · isaac_ros_ws를 source하고 ros_system 다시 빌드",
                 load_path_.c_str());
#endif
  }

  std::string frame_, load_path_;
  int speckle_ = 0;
  long n_speckle_ = 0;
  unsigned long n_in_ = 0, n_out_ = 0;
  int load_wait_ = 0;
  bool fresh_ = false;
  OccupancyGrid::SharedPtr grid_, walls_;
  rclcpp::Publisher<OccupancyGrid>::SharedPtr pub_;
  rclcpp::Subscription<OccupancyGrid>::SharedPtr grid_sub_, walls_sub_;
  rclcpp::TimerBase::SharedPtr tick_timer_, log_timer_, load_timer_;
};

std::shared_ptr<rclcpp::Node> make_nvblox_map(const rclcpp::NodeOptions & options) {
  return std::make_shared<NvbloxMap>(options);
}

}  // namespace ros_system
