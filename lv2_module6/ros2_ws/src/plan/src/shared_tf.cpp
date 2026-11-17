#include "plan/shared_tf.hpp"

#include <map>
#include <mutex>

namespace plan {
namespace {
struct Holder {
  std::shared_ptr<tf2_ros::Buffer> buffer;
  std::shared_ptr<tf2_ros::TransformListener> listener;  // buffer보다 먼저 사라짐 (선언 반대 순서)
};
std::mutex registry_mutex;
std::map<std::pair<const rclcpp::Context *, int>, std::weak_ptr<Holder>> registry;
}  // namespace

std::pair<std::shared_ptr<tf2_ros::Buffer>, std::shared_ptr<tf2_ros::TransformListener>> shared_tf(rclcpp::Node & node) {
  const auto context = node.get_node_base_interface()->get_context();
  const std::pair<const rclcpp::Context *, int> key{context.get(), static_cast<int>(node.get_clock()->get_clock_type())};
  std::lock_guard<std::mutex> lock(registry_mutex);
  auto holder = registry[key].lock();
  if (!holder) {
    holder = std::make_shared<Holder>();
    holder->buffer = std::make_shared<tf2_ros::Buffer>(node.get_clock());
    holder->listener = std::make_shared<tf2_ros::TransformListener>(*holder->buffer);
    registry[key] = holder;
  }
  // aliasing: 노드가 쥔 포인터 둘이 holder 하나를 살려 둠
  return {std::shared_ptr<tf2_ros::Buffer>(holder, holder->buffer.get()),
          std::shared_ptr<tf2_ros::TransformListener>(holder, holder->listener.get())};
}

}  // namespace plan
