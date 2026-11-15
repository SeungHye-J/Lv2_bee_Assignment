// ml 노드를 컴포넌트로도 (11-16 메모리) — bringup core_composition:=true면 mission_nodes 컨테이너 한 프로세스에 올림
//   실행 파일(grasp_scorer)은 그대로
#include "rclcpp_components/register_node_macro.hpp"

#include "ml/grasp_scorer.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(ml::GraspScorer)
