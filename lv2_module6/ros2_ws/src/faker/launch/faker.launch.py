"""테스트베드 실행: bringup(🛠 노드) + ros_system 중 도커에서 도는 📦 + 도커에 없는 📦의 대역 + 호스트 파일 연동.

  ros2 launch faker faker.launch.py
  ros2 launch faker faker.launch.py pick_fail_count:=1      # 파지 1회 실패 → coordinator가 2순위 후보로 재시도
  ros2 launch faker faker.launch.py pick_fail_count:=2      # 후보 2개 실패 → FAILED → VLM 재계획 → 다시 집기
  ros2 launch faker faker.launch.py nav_fail_count:=2       # 이동 2회 실패 → 알리고 정지
  ros2 launch faker faker.launch.py mock_effort:=1.5        # 팔 전류 과다 → arm_safety_monitor 접촉 감지
  ros2 launch faker faker.launch.py mock_hardware:=false    # 시뮬레이터가 OpenCR 역할 (미끄럼·충돌·통신 끊김 — simulator/)
  ros2 launch faker faker.launch.py ml:=real                # ML 대역 대신 진짜 ml 노드·컨트롤러 (GPU·모델 없이도 돌 때만)

실제로 띄우는 📦 (ros_system, mock_hardware:=true):
  robot_state_publisher(URDF) · controller_manager + control/BeebotSystemHardware(mock) + 컨트롤러 · robot_localization EKF
대역 (도커 이미지에 없는 📦 · GPU·모델이 없는 ML):
  fake_ml          grasp_scorer · rl_nav_tuner · rl_nav_correction (같은 노드 이름·토픽 — ml:=fake 기본)
  PassthroughArmCorrection  rl_arm_correction 컨트롤러 자리 (Δ=0)
  fake_camera ×2   realsense2_camera (pole_cam, gripper_cam) — debug/input/hardware/camera/<이름>.png가 있으면 그 장면
  fake_nav2        Nav2 (노드 이름 controller_server — rl_nav_tuner의 set_parameters를 받음, /cmd_vel_corrected → /cmd_vel 중계)
  fake_moveit      move_group + MTC 실행
호스트 파일 연동 (통제실·시뮬레이션):
  monitor_manager  토픽 → debug/topic/<토픽>/message (JSON) · image.jpg
  input_manager    debug/input/topic/<토픽>/message (JSON) → 토픽
ml:=real이면 진짜 ml 노드를 띄운다 — 모델이 없거나 learning_enabled:=false면 기본 동작(Δ=0·규칙)으로 돌아야 한다.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def arg(name, default, description):
    return DeclareLaunchArgument(name, default_value=default, description=description)


def include(package, launch_file, args):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare(package), 'launch', launch_file])),
        launch_arguments=args.items())


def fake(executable, name, overrides=None, respawn=False):
    # 파라미터: config/faker.yaml(노드 이름별) → launch 인자(overrides)가 덮어씀
    config = PathJoinSubstitution([FindPackageShare('faker'), 'config', 'faker.yaml'])
    return Node(package='faker', executable=executable, name=name, output='screen', parameters=[config, overrides or {}],
                respawn=respawn, respawn_delay=1.0)


def num(name, t):
    return ParameterValue(LaunchConfiguration(name), value_type=t)


def generate_launch_description():
    return LaunchDescription([
        arg('command', '빨간 퍽을 목표 구역으로 옮겨', 'user_input 텍스트 명령 (빈 값이면 안 보냄)'),
        arg('ml', 'fake', 'fake = ML 대역(도커·CI 기본, GPU 불필요) · real = 진짜 ml 노드·컨트롤러'),
        arg('learning_enabled', 'true', '학습 노드 on/off (ml:=real일 때)'),
        arg('vlm_endpoint', '', '호스트 ai-service 주소 (비면 템플릿 고정)'),
        arg('pick_fail_count', '0', '처음 N번 MTC 실행 실패'),
        arg('nav_fail_count', '0', '처음 N번 NavigateToPose ABORTED'),
        arg('mock_hardware', 'true', 'false면 controller_manager가 가상 시리얼(/dev/ttyACM0)로 OpenCR과 통신 — 시뮬레이터가 OpenCR 역할'),
        arg('mock_effort', '0.0', '팔 관절 전류 [A] (arm_safety_monitor 기준 1.0)'),
        arg('debug_dir', '/debug', '호스트와 공유하는 폴더 (topic/ · input/ · output/)'),

        include('bringup', 'bringup.launch.py', {
            'use_ros_system': 'false',
            'command': LaunchConfiguration('command'),
            'learning_enabled': LaunchConfiguration('learning_enabled'),
            'vlm_endpoint': LaunchConfiguration('vlm_endpoint'),
            'use_ml': PythonExpression(["'", LaunchConfiguration('ml'), "' == 'real'"]),
        }),
        include('ros_system', 'ros_system.launch.py', {
            'use_ros2_control': 'true',
            'mock_hardware': LaunchConfiguration('mock_hardware'),
            'mock_effort': LaunchConfiguration('mock_effort'),
            'use_ekf': 'true',
            'use_cameras': 'false',
            'use_rtabmap': 'false',
            'use_nav2': 'false',
            'arm_correction_type': PythonExpression(
                ["'ml/RlArmCorrection' if '", LaunchConfiguration('ml'), "' == 'real' else 'faker/PassthroughArmCorrection'"]),
        }),

        fake('fake_camera', 'pole_cam', respawn=True),
        fake('fake_camera', 'gripper_cam', respawn=True),
        fake('fake_nav2', 'controller_server', {'fail_count': num('nav_fail_count', int)}),
        fake('fake_moveit', 'move_group', {'fail_count': num('pick_fail_count', int)}),
        Node(package='faker', executable='fake_ml', output='screen',  # 노드 3개라 name을 주지 않는다
             parameters=[PathJoinSubstitution([FindPackageShare('faker'), 'config', 'faker.yaml'])],
             condition=IfCondition(PythonExpression(["'", LaunchConfiguration('ml'), "' == 'fake'"]))),
        fake('monitor_manager', 'monitor_manager', {'debug_dir': LaunchConfiguration('debug_dir')}),
        fake('input_manager', 'input_manager', {'debug_dir': LaunchConfiguration('debug_dir')}),
        fake('service_action_monitor', 'service_action_monitor', {'debug_dir': LaunchConfiguration('debug_dir')}),
        fake('service_action_input', 'service_action_input', {'debug_dir': LaunchConfiguration('debug_dir')}),
    ])
