"""전체 실행: ros_system(📦 제공 패키지) + 🛠 직접 작성 노드.

  ros2 launch bringup bringup.launch.py
  ros2 launch bringup bringup.launch.py use_ros_system:=false            # 📦 대신 faker 등 다른 입력 (faker.launch.py가 씀)
  ros2 launch bringup bringup.launch.py learning_enabled:=false           # 학습 노드 Δ=0 · 프리셋 A 고정 (비교 기준)
  ros2 launch bringup bringup.launch.py command:="빨간 퍽을 목표 구역으로 옮겨"   # user_input에 텍스트 명령 한 번

🛠 노드 (패키지 · 담당):
  perception: perception (전승혜)
  plan:       user_input(외부 텍스트 입력), vlm_task_planner, coordinator (권형중)
  ml:         grasp_scorer, rl_nav_tuner, rl_nav_correction (송정혁)
              rl_arm_correction은 노드가 아니라 컨트롤러 플러그인(ml/RlArmCorrection) — ros_system의 controller_manager가 올린다
  control:    arm_safety_monitor, manual_control (김현하)
상태: 각 노드 /diagnostics → diagnostic_aggregator → /diagnostics_agg (config/diagnostics.yaml)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def node(package, executable, overrides=None):
    # 파라미터: 패키지 config/<패키지>.yaml → launch 인자(overrides)가 덮어씀
    config = PathJoinSubstitution([FindPackageShare(package), 'config', f'{package}.yaml'])
    return Node(package=package, executable=executable, output='screen', parameters=[config, overrides or {}])


def generate_launch_description():
    learning = {'enabled': LaunchConfiguration('learning_enabled')}
    return LaunchDescription([
        DeclareLaunchArgument('use_ros_system', default_value='true',
                              description='false면 ros_system(📦)을 띄우지 않음'),
        DeclareLaunchArgument('use_ml', default_value='true',
                              description='false면 ml 노드(grasp_scorer · rl_nav_tuner · rl_nav_correction)를 띄우지 않음 — faker ML 대역을 쓸 때'),
        DeclareLaunchArgument('learning_enabled', default_value='true',
                              description='false면 학습 노드가 Δ=0 · 기본 프리셋 (노드는 그대로 뜸)'),
        DeclareLaunchArgument('command', default_value='',
                              description='user_input이 시작 후 한 번 보낼 텍스트 명령'),

        # GroupAction: ros_system의 launch 인자가 이 파일 인자와 섞이지 않게 범위 분리
        GroupAction([IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('ros_system'), 'launch', 'ros_system.launch.py'])))],
            condition=IfCondition(LaunchConfiguration('use_ros_system'))),

        node('perception', 'perception_node'),
        node('plan', 'user_input', {'command': ParameterValue(LaunchConfiguration('command'), value_type=str)}),  # 공백·빈 값도 문자열로
        node('plan', 'vlm_task_planner'),
        node('plan', 'coordinator'),
        GroupAction([
            node('ml', 'grasp_scorer', learning),
            node('ml', 'rl_nav_tuner', learning),
            node('ml', 'rl_nav_correction', learning),
        ], condition=IfCondition(LaunchConfiguration('use_ml'))),
        node('control', 'arm_safety_monitor'),
        node('control', 'manual_control'),
        Node(package='diagnostic_aggregator', executable='aggregator_node', output='screen',
             parameters=[PathJoinSubstitution([FindPackageShare('bringup'), 'config', 'diagnostics.yaml'])]),
    ])
