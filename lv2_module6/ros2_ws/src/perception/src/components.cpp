// perception C++ 보조 노드를 컴포넌트로도 (11-16 메모리) — bringup core_composition:=true면 body_filter · signal_decoder를
//   control_nodes 컨테이너에 올림. depth_gate · static_esdf_slice는 등록만 (ros_system은 아직 실행 파일로 띄움)
//   perception_node(YOLO)는 무거워(CPU·GPU, 콜백이 오래 걸림) 따로 프로세스 그대로
#include "rclcpp_components/register_node_macro.hpp"

#include "perception/body_filter.hpp"
#include "perception/depth_gate.hpp"
#include "perception/signal_decoder.hpp"
#include "perception/static_esdf_slice.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(perception::BodyFilter)
RCLCPP_COMPONENTS_REGISTER_NODE(perception::SignalDecoderNode)
RCLCPP_COMPONENTS_REGISTER_NODE(perception::DepthGate)
RCLCPP_COMPONENTS_REGISTER_NODE(perception::StaticEsdfSlice)
