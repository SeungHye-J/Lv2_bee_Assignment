// ml 기본 테스트: rl_nav_correction은 보정 뒤 max_vel_x · max_vel_theta로 다시 자른다
#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <thread>

#include "ml/rl_nav_correction.hpp"
#include "std_srvs/srv/set_bool.hpp"

using namespace ml;  // 노드 클래스·메시지 별칭 (include/ml/*.hpp)

// 조건이 맞을 때까지 step()으로 입력을 넣으며 spin. discovery 전에 보낸 메시지는 사라질 수 있어서 반복해 넣는다
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

TEST_F(RosTest, ClipsToMaxVelocity) {
  auto node = std::make_shared<RlNavCorrection>();
  auto probe = std::make_shared<rclcpp::Node>("probe");
  auto odom_pub = probe->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
  auto cmd_pub = probe->create_publisher<TwistStamped>("/cmd_vel_nav", 10);
  TwistStamped::SharedPtr got;
  auto sub = probe->create_subscription<TwistStamped>("/cmd_vel_corrected", rclcpp::QoS(1).reliable(),
    [&](TwistStamped::SharedPtr m) { got = m; });
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(probe);

  ASSERT_TRUE(spin_until(exec,
    [&]() {
      odom_pub->publish(nav_msgs::msg::Odometry());
      TwistStamped cmd;
      cmd.twist.linear.x = 0.50;   // max_vel_x(0.26) 초과 → 잘림
      cmd.twist.angular.z = 2.50;  // max_vel_theta(1.82) 초과 → 잘림
      cmd_pub->publish(cmd);
    },
    [&]() { return got && std::abs(got->twist.linear.x - 0.26) < 1e-9; }));
  EXPECT_DOUBLE_EQ(got->twist.angular.z, 1.82);
}

// 정책 출력 스케일·클램프 (학습 env와 같은 약속)
TEST(RlNavCorrectionMath, ScaleAction) {
  const auto a = scale_action(0.5f, -0.5f, 0.05, 0.10);
  EXPECT_DOUBLE_EQ(a.first, 0.025);
  EXPECT_DOUBLE_EQ(a.second, -0.05);
  const auto b = scale_action(5.0f, -5.0f, 0.05, 0.10);  // [-1,1]로 자름
  EXPECT_DOUBLE_EQ(b.first, 0.05);
  EXPECT_DOUBLE_EQ(b.second, -0.10);
}

// 정지 명령에는 Δ를 주지 않는다 · 상한으로 자른다
TEST(RlNavCorrectionMath, ApplyDelta) {
  const auto zero = apply_delta({0.0, 0.0}, {0.05, 0.10}, 0.26, 1.82);
  EXPECT_DOUBLE_EQ(zero.first, 0.0);
  EXPECT_DOUBLE_EQ(zero.second, 0.0);
  const auto clip = apply_delta({0.25, 1.80}, {0.05, 0.10}, 0.26, 1.82);
  EXPECT_DOUBLE_EQ(clip.first, 0.26);
  EXPECT_DOUBLE_EQ(clip.second, 1.82);
  const auto add = apply_delta({0.10, 0.00}, {-0.02, 0.03}, 0.26, 1.82);
  EXPECT_DOUBLE_EQ(add.first, 0.08);
  EXPECT_DOUBLE_EQ(add.second, 0.03);
}

// ~/toggle 서비스로 ONNX 보정 켜기/끄기 (파라미터 enabled와 같은 값)
TEST_F(RosTest, ToggleService) {
  auto node = std::make_shared<RlNavCorrection>();
  auto probe = std::make_shared<rclcpp::Node>("probe_toggle");
  auto cli = probe->create_client<std_srvs::srv::SetBool>("/rl_nav_correction/toggle");
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(probe);
  ASSERT_TRUE(spin_until(exec, []() {}, [&]() { return cli->service_is_ready(); }));
  auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
  req->data = false;
  auto fut = cli->async_send_request(req);
  ASSERT_TRUE(spin_until(exec, []() {}, [&]() { return fut.wait_for(std::chrono::seconds(0)) == std::future_status::ready; }));
  EXPECT_FALSE(node->get_parameter("enabled").as_bool());
  EXPECT_TRUE(fut.get()->success);
}
