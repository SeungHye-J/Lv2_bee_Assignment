#include <chrono>
#include <memory>
#include <thread>

#include <gtest/gtest.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "skills/arm_state_cache.hpp"

TEST(ArmStateCache, SharesJointStateReaderWithinContext) {
  rclcpp::init(0, nullptr);

  auto first = std::make_shared<rclcpp::Node>("arm_cache_first");
  auto second = std::make_shared<rclcpp::Node>("arm_cache_second");
  auto first_cache = skills::ArmStateCache::for_node(*first);
  auto second_cache = skills::ArmStateCache::for_node(*second);
  EXPECT_EQ(first_cache.get(), second_cache.get());

  auto publisher = second->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(first);

  sensor_msgs::msg::JointState state;
  state.name = {"joint1", "joint2"};
  state.position = {0.25, -0.5};
  state.effort = {0.1, 0.2};
  bool received = false;
  double observed = 0.0;
  size_t reader_count = 0;
  for (int attempt = 0; attempt < 30 && !received; ++attempt) {
    publisher->publish(state);
    executor.spin_some();
    received = first_cache->position("joint1", observed);
    if (!received) std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  reader_count = publisher->get_subscription_count();
  EXPECT_EQ(reader_count, 1u);
  double position = 0.0;
  ASSERT_TRUE(first_cache->position("joint1", position));
  EXPECT_DOUBLE_EQ(position, 0.25);

  executor.remove_node(first);
  rclcpp::shutdown();
}
