// perception 기본 테스트: pole_cam 영상이 들어오면 /tracked_objects에 물체 목록이 나온다
#include <gtest/gtest.h>

#include <chrono>
#include <thread>

#include "perception/perception_node.hpp"

using namespace perception;  // 노드 클래스·메시지 별칭 (include/perception/*.hpp)

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

TEST_F(RosTest, PublishesTrackedObjectsAfterImage) {
  auto node = std::make_shared<PerceptionNode>();
  auto probe = std::make_shared<rclcpp::Node>("probe");
  auto image_pub = probe->create_publisher<Image>("/pole_cam/color/image_raw", rclcpp::SensorDataQoS());
  TrackedObjectArray::SharedPtr got;
  auto sub = probe->create_subscription<TrackedObjectArray>("/tracked_objects", rclcpp::QoS(5).reliable(),
    [&](TrackedObjectArray::SharedPtr m) { got = m; });
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(probe);

  ASSERT_TRUE(spin_until(exec,
    [&]() { Image img; img.header.stamp = probe->now(); image_pub->publish(img); },
    [&]() { return got != nullptr; }));
  EXPECT_EQ(got->header.frame_id, "map");
  bool puck = false;
  for (const auto & o : got->objects) puck |= o.class_name == "puck";
  EXPECT_TRUE(puck);
}
