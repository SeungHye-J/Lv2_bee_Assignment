// 같은 프로세스의 plan 노드(coordinator · mission_manager)가 TF 버퍼·리스너 하나를 같이 씀 (11-16 메모리)
//   리스너 하나 = /tf·/tf_static 구독 둘 + 10 s 버퍼 + 내부 노드·스레드 — mission_nodes 컨테이너(bringup core_composition)에
//   둘을 올리면 하나로. 따로 프로세스(실행 파일)면 지금처럼 노드마다 하나.
//   열쇠 = (context, 시계 종류) — 시뮬 시간 노드와 벽시계 노드가 버퍼를 섞지 않게. 마지막 노드가 내려놓으면 함께 사라짐
//   (skills ArmStateCache와 같은 방식)
#pragma once

#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace plan {

std::pair<std::shared_ptr<tf2_ros::Buffer>, std::shared_ptr<tf2_ros::TransformListener>> shared_tf(rclcpp::Node & node);

}  // namespace plan
