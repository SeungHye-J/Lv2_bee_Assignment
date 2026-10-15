"""📦 제공 패키지 기동 (실기용). 설계: documents/ros2-design/09-nodes.md "직접 작성 노드와 제공 노드"

  ros2 launch ros_system ros_system.launch.py
  ros2 launch ros_system ros_system.launch.py use_rtabmap:=false pole_cam_serial:=123456
  ros2 launch ros_system ros_system.launch.py use_nav2:=true nav2_params_file:=/path/nav2_params.yaml

  ros2 launch ros_system ros_system.launch.py mock_hardware:=true   # 실기 없이 controller_manager (명령 → 상태 되돌림)

기본으로 켜는 것: robot_state_publisher(URDF) + ros2_control(controller_manager + 컨트롤러),
                  RealSense ×2(pole_cam·gripper_cam), robot_localization EKF, RTAB-Map(위치추정 모드)
설정이 준비되면 켤 것 (기본 false):
  - Nav2: nav2_params.yaml (voxel layer에 pole_cam 점구름, velocity_smoother 입력 → /cmd_vel_mux (twist_mux 출력),
          enable_stamped_cmd_vel: true, behavior_server 출력 /cmd_vel_nav)
TODO(권형중) 아직 자리만 있는 것 — 설정 패키지가 필요:
  - move_group (+ MTC capability): MoveIt Setup Assistant 설정 패키지, pick_ik, octomap 센서(pole_cam 점구름)
테스트베드에서는 faker가 이 패키지들 대신 같은 토픽·액션을 낸다 (ros2 launch faker faker.launch.py).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

CONTROLLERS = ['joint_state_broadcaster', 'imu_sensor_broadcaster', 'diff_drive_controller',
               'rl_arm_correction', 'arm_controller',  # 체인은 뒤에서부터 (rl_arm_correction → arm_controller)
               'gripper_controller']


def flag(name, default, description):
    return DeclareLaunchArgument(name, default_value=default, description=description)


def camera(name, serial_arg, fps):
    # camera_namespace:='' + camera_name → /pole_cam/color/image_raw 처럼 설계의 토픽 이름에 맞춘다
    return Node(
        package='realsense2_camera', executable='realsense2_camera_node',
        namespace='', name=name, output='screen',
        condition=IfCondition(LaunchConfiguration('use_cameras')),
        parameters=[{
            'serial_no': LaunchConfiguration(serial_arg),
            'camera_name': name,
            'rgb_camera.color_profile': f'640x480x{fps}',
            'depth_module.depth_profile': f'640x480x{fps}',
            'align_depth.enable': True,
            'pointcloud.enable': True,
            'publish_tf': False,  # 카메라 프레임은 URDF(robot_state_publisher)가 낸다 — 하나만 남김
        }])


def share(*path):
    return PathJoinSubstitution([FindPackageShare('ros_system'), *path])


def generate_launch_description():
    robot_description = ParameterValue(Command([
        'xacro ', share('urdf', 'beebot.urdf.xacro'),
        ' mock_hardware:=', LaunchConfiguration('mock_hardware'),
        ' mock_effort:=', LaunchConfiguration('mock_effort'),
        ' port:=', LaunchConfiguration('port')]), value_type=str)
    ros2_control = IfCondition(LaunchConfiguration('use_ros2_control'))
    return LaunchDescription([
        flag('use_ros2_control', 'true', 'robot_state_publisher + controller_manager + 컨트롤러'),
        flag('mock_hardware', 'false', 'true면 BeebotSystemHardware가 실기 대신 명령을 상태로 되돌림'),
        flag('mock_effort', '0.0', 'mock일 때 팔 관절 effort [A]'),
        flag('port', '/dev/ttyACM0', 'OpenCR 포트'),
        flag('arm_correction_type', 'ml/RlArmCorrection', 'rl_arm_correction 컨트롤러 플러그인 (테스트베드 ML 대역: faker/PassthroughArmCorrection)'),
        flag('use_twist_mux', 'true', '바퀴 속도 명령 고르기 (자동 · 수동 · 비상정지 잠금)'),
        flag('use_cameras', 'true', 'RealSense 2대 (pole_cam·gripper_cam)'),
        flag('pole_cam_serial', "''", 'pole_cam serial_no (빈 값이면 첫 장치)'),
        flag('gripper_cam_serial', "''", 'gripper_cam serial_no'),
        flag('use_ekf', 'true', 'robot_localization EKF'),
        flag('use_rtabmap', 'true', 'RTAB-Map'),
        flag('localization', 'true', 'true=위치추정 모드(운영), false=매핑 모드(-d로 DB 새로)'),
        flag('use_nav2', 'false', 'Nav2 navigation (nav2_params_file 필요)'),
        flag('nav2_params_file', '', 'Nav2 파라미터 파일'),

        Node(
            package='robot_state_publisher', executable='robot_state_publisher', output='screen',
            condition=ros2_control, parameters=[{'robot_description': robot_description}]),
        # controller_manager는 /robot_description 토픽(robot_state_publisher)에서 URDF를 받는다
        Node(
            package='controller_manager', executable='ros2_control_node', output='screen',
            condition=ros2_control,
            parameters=[share('config', 'controllers.yaml'),
                        {'rl_arm_correction.type': LaunchConfiguration('arm_correction_type')}]),
        Node(
            package='controller_manager', executable='spawner', output='screen',
            condition=ros2_control,
            # 컨트롤러별 파라미터·리매핑은 spawner가 컨트롤러 노드에 넘긴다
            # (ros2_control_node에 준 파일·리매핑은 controller_manager 자신에게만 적용됨).
            # 리매핑은 모든 컨트롤러에 같이 가지만 ~/cmd_vel·~/odom은 diff_drive, ~/imu는 imu broadcaster에만 있다.
            arguments=CONTROLLERS + [
                '--controller-manager', '/controller_manager',
                '--controller-manager-timeout', '60',
                '--service-call-timeout', '30',  # 기동 직후 CPU가 몰리면 10초(기본) 안에 응답 못 할 때가 있다
                '--param-file', share('config', 'controllers.yaml'),
                '--controller-ros-args', '-r ~/cmd_vel:=/cmd_vel',
                '--controller-ros-args', '-r ~/odom:=/odom',
                '--controller-ros-args', '-r ~/imu:=/imu']),

        # 자동(/cmd_vel_corrected) · 수동(/cmd_vel_manual) 중 고르고 /e_stop이면 막는다 → /cmd_vel_mux → velocity_smoother
        Node(
            package='twist_mux', executable='twist_mux', output='screen',
            condition=IfCondition(LaunchConfiguration('use_twist_mux')),
            parameters=[share('config', 'twist_mux.yaml')],
            remappings=[('cmd_vel_out', '/cmd_vel_mux')]),

        camera('pole_cam', 'pole_cam_serial', 30),
        camera('gripper_cam', 'gripper_cam_serial', 15),

        Node(
            package='robot_localization', executable='ekf_node', name='ekf_filter_node', output='screen',
            condition=IfCondition(LaunchConfiguration('use_ekf')),
            parameters=[share('config', 'ekf.yaml')]),

        # 설계 3-1: 하향 카메라라 시각 오도메트리 끔, x·y·yaw만 (Reg/Force3DoF), 격자는 depth로 (Grid/Sensor 1)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('rtabmap_launch'), 'launch', 'rtabmap.launch.py'])),
            condition=IfCondition(LaunchConfiguration('use_rtabmap')),
            launch_arguments={
                'rgb_topic': '/pole_cam/color/image_raw',
                'depth_topic': '/pole_cam/aligned_depth_to_color/image_raw',
                'camera_info_topic': '/pole_cam/color/camera_info',
                'frame_id': 'base_footprint',
                'odom_topic': '/odometry/filtered',
                'visual_odometry': 'false',
                'approx_sync': 'true',
                'qos': '1',
                'localization': LaunchConfiguration('localization'),
                'rtabmap_args': '--Reg/Force3DoF true --Grid/Sensor 1',
                'rviz': 'false',
                'rtabmap_viz': 'false',
            }.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py'])),
            condition=IfCondition(LaunchConfiguration('use_nav2')),
            launch_arguments={'params_file': LaunchConfiguration('nav2_params_file')}.items()),
    ])
