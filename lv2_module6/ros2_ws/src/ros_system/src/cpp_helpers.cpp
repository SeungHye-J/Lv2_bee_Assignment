// cpp_helpers — ros_system 보조 노드 C++판을 한 프로세스에 (11-16 · py_helpers의 C++판)
//   cpp_helpers nvblox_map rewind   (인자 = 띄울 노드 이름 — ros_system.launch.py helpers_cpp · py_helpers_one_process)
//   노드마다 기본 콜백 그룹(서로 배타)이라 한 노드 안의 콜백 순서는 따로 띄울 때와 같고, 노드끼리는 스레드를 나눠 씀.
//   파라미터는 launch가 넘긴 것을 노드마다 자기가 선언한 이름만 읽음 (두 노드 파라미터 이름이 겹치지 않음)
//   노드 하나만 띄울 땐 launch가 name을 줘도 됨 (__node 리매핑이 그 하나에만 걸림)
#include <cstdio>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "ros_system/helpers.hpp"

int main(int argc, char ** argv) {
  auto args = rclcpp::init_and_remove_ros_arguments(argc, argv);
  const std::map<std::string, std::function<std::shared_ptr<rclcpp::Node>()>> make = {
    {"nvblox_map", [] { return ros_system::make_nvblox_map(); }},
    {"rewind", [] { return ros_system::make_rewind(); }}};
  std::vector<std::shared_ptr<rclcpp::Node>> nodes;
  for (size_t i = 1; i < args.size(); ++i) {
    auto it = make.find(args[i]);
    if (it == make.end()) {
      std::fprintf(stderr, "cpp_helpers: 모르는 노드 %s (nvblox_map · rewind)\n", args[i].c_str());
      rclcpp::shutdown();
      return 2;
    }
    nodes.push_back(it->second());
  }
  if (nodes.empty()) {
    std::fprintf(stderr, "사용: cpp_helpers <nvblox_map|rewind>…\n");
    rclcpp::shutdown();
    return 2;
  }
  std::unique_ptr<rclcpp::Executor> ex;  // 노드 하나면 SingleThreaded (MultiThreaded 1스레드는 경고)
  if (nodes.size() == 1)
    ex = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  else
    ex = std::make_unique<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions(), nodes.size());
  for (auto & n : nodes) ex->add_node(n);
  ex->spin();
  rclcpp::shutdown();
  return 0;
}
