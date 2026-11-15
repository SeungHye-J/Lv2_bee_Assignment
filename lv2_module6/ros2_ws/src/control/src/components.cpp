// control 노드를 컴포넌트로도 (11-16 메모리) — bringup core_composition:=true면 control_nodes 컨테이너 한 프로세스에 올림
//   실행 파일(arm_safety_monitor · manual_control)은 그대로 — 같은 클래스를 main에서 spin
#include "rclcpp_components/register_node_macro.hpp"

#include "control/arm_safety_monitor.hpp"
#include "control/manual_control.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(control::ArmSafetyMonitor)
RCLCPP_COMPONENTS_REGISTER_NODE(control::ManualControl)
