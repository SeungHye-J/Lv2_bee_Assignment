// cmd_vel_relay C++판 (11-16) — stand_test.launch.py 전용: twist_mux 출력 /cmd_vel_mux → diff_drive /cmd_vel 그대로 넘김
//   scripts/cmd_vel_relay.py와 같음 (노드 이름 · 토픽 · QoS 깊이 10). 전체 bringup은 그 사이에 Nav2 velocity_smoother ·
//   collision_monitor가 있는데, 가벼운 팔 테스트 실행엔 Nav2가 없어 수동 주행이 바퀴까지 안 갔다. 부드럽게 하기·충돌 감시는 없음 — 느리게만 몰 것
//   왜 C++: 파이썬 노드는 젯슨 CPU가 몰리면 ~1 s 멈춤 → 바퀴 명령이 끊김 (diff_drive cmd_vel_timeout)
#include <memory>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

class CmdVelRelay : public rclcpp::Node {
public:
  CmdVelRelay() : Node("cmd_vel_relay") {
    pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
    sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "/cmd_vel_mux", 10, [this](geometry_msgs::msg::TwistStamped::UniquePtr m) { pub_->publish(std::move(m)); });
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelRelay>());
  rclcpp::shutdown();
  return 0;
}
