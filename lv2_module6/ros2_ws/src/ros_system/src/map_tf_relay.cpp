// map→odom TF 이어 내기 (11-16) — cuvslam_slam(파이썬)이 영상 한 장마다 계산한 map→odom(/cuvslam/map_to_odom)을 받아
//   rate_hz로 계속 TF를 냄. 표준(AMCL · slam_toolbox)처럼 보정 계산과 TF 발행을 나눔: 파이썬 노드가 CPU 몰림으로 1 s 넘게 멈춰도
//   TF가 끊기지 않음 (끊기면 Nav2 controller가 목표를 odom으로 못 옮겨 error 102). map→odom은 천천히 바뀌어 마지막 값을 그대로 냄.
//   멈춘 동안의 위치 보정은 늦어지고 그 사이는 odom(바퀴 + IMU)으로 이어 감.
// 시각 = 지금 + future_s — 방금 들어온 영상(nvblox depth)이 마지막 TF보다 한 주기(33 ms)쯤 새 시각일 수 있어 '미래 외삽'으로 버리지 않게
// 실시간 우선순위 (11-16): 시작할 때 SCHED_FIFO MAP_TF_RELAY_RT_PRIO(기본 50 · 0 = 안 함) — CPU가 몰려도 먼저 돎.
//   rclcpp::init 전에 올려 DDS 스레드도 물려받음. 계정 rtprio 한도가 필요 (/etc/security/limits.d/90-bee-rtprio.conf, 다시 로그인)
//   — 없으면 경고만 하고 보통 우선순위로 계속
#include <sched.h>

#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

class MapTfRelay : public rclcpp::Node {
public:
  MapTfRelay() : Node("map_tf_relay") {
    const double rate = declare_parameter<double>("rate_hz", 30.0);
    future_ = rclcpp::Duration::from_seconds(declare_parameter<double>("future_s", 0.1));
    tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    // transient_local — 이 노드가 나중에 떠도 마지막 값을 바로 받음
    sub_ = create_subscription<geometry_msgs::msg::TransformStamped>(
      declare_parameter<std::string>("topic", "/cuvslam/map_to_odom"), rclcpp::QoS(1).reliable().transient_local(),
      [this](geometry_msgs::msg::TransformStamped::ConstSharedPtr m) { last_ = m; });
    // 벽시계 타이머 — 시뮬 /clock이 잠깐 늦어도 계속 냄 (시각은 노드 시계 now())
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate), [this]() {
      if (!last_) return;
      auto tf = *last_;
      tf.header.stamp = now() + future_;
      tf_->sendTransform(tf);
    });
  }

private:
  rclcpp::Duration future_{0, 0};
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr sub_;
  geometry_msgs::msg::TransformStamped::ConstSharedPtr last_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  const char * env = std::getenv("MAP_TF_RELAY_RT_PRIO");
  const int prio = env ? std::atoi(env) : 50;
  std::string rt = "보통 우선순위 (MAP_TF_RELAY_RT_PRIO=0)";
  if (prio > 0) {
    sched_param sp{};
    sp.sched_priority = prio;
    rt = sched_setscheduler(0, SCHED_FIFO, &sp) == 0
           ? "SCHED_FIFO " + std::to_string(prio)
           : std::string("실시간 우선순위 못 올림 (") + std::strerror(errno) + ") — 보통 우선순위로 (계정 rtprio 한도 확인)";
  }
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MapTfRelay>();
  if (rt.rfind("SCHED_FIFO", 0) == 0) RCLCPP_INFO(node->get_logger(), "%s", rt.c_str());
  else RCLCPP_WARN(node->get_logger(), "%s", rt.c_str());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
