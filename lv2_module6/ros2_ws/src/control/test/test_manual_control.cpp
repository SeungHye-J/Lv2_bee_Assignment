// control 테스트: manual_control은 수동 모드일 때만 /manual/cmd_vel을 /cmd_vel_manual로 넘기고 /operation_mode를 바꾼다
#include <gtest/gtest.h>

#include <chrono>
#include <thread>

#include "control/manual_control.hpp"

using namespace control;  // 노드 클래스·메시지 별칭 (include/control/*.hpp)

template <class Step, class Done>
bool spin_until(rclcpp::Executor & exec, Step step, Done done, double timeout_s = 5.0) {
  const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
  while (std::chrono::steady_clock::now() < end) {
    step();
    exec.spin_some(std::chrono::milliseconds(50));
    if (done()) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return false;
}

class RosTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
};

TEST_F(RosTest, ForwardsOnlyInManualMode) {
  auto node = std::make_shared<ManualControl>();
  auto probe = std::make_shared<rclcpp::Node>("probe");
  auto enable_pub = probe->create_publisher<std_msgs::msg::Bool>("/manual/enable", rclcpp::QoS(1).reliable());
  auto cmd_pub = probe->create_publisher<ManualControl::TwistStamped>("/manual/cmd_vel", 10);
  ManualControl::TwistStamped::SharedPtr out;
  std::string mode;
  auto out_sub = probe->create_subscription<ManualControl::TwistStamped>("/cmd_vel_manual", rclcpp::QoS(1).reliable(),
    [&](ManualControl::TwistStamped::SharedPtr m) { if (m->twist.linear.x > 0) out = m; });
  auto mode_sub = probe->create_subscription<std_msgs::msg::String>("/operation_mode", rclcpp::QoS(1).reliable().transient_local(),
    [&](std_msgs::msg::String::SharedPtr m) { mode = m->data; });
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(probe);

  ManualControl::TwistStamped cmd;
  cmd.twist.linear.x = 0.1;
  // 자동 모드: 수동 명령은 버린다
  ASSERT_TRUE(spin_until(exec, [&]() { cmd_pub->publish(cmd); }, [&]() { return mode == "AUTO"; }));
  spin_until(exec, [&]() { cmd_pub->publish(cmd); }, [] { return false; }, 0.5);
  EXPECT_EQ(out, nullptr);
  // 수동 모드: 넘긴다
  ASSERT_TRUE(spin_until(exec, [&]() { std_msgs::msg::Bool b; b.data = true; enable_pub->publish(b); cmd_pub->publish(cmd); },
    [&]() { return mode == "MANUAL" && out != nullptr; }));
  EXPECT_DOUBLE_EQ(out->twist.linear.x, 0.1);
}
