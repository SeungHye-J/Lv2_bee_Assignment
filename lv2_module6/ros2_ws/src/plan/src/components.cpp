// plan 노드를 컴포넌트로도 (11-16 메모리) — bringup core_composition:=true면 mission_nodes 컨테이너 한 프로세스에 올림
//   실행 파일(coordinator · mission_manager)은 그대로 — 같은 클래스를 main에서 spin
#include "rclcpp_components/register_node_macro.hpp"

#include "plan/coordinator.hpp"
#include "plan/mission_manager.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(plan::Coordinator)
RCLCPP_COMPONENTS_REGISTER_NODE(plan::MissionManager)
