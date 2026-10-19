"""🧪 Gazebo 카메라 테스트 — 팀 URDF 로봇을 미로에 세워 두고 RGB-D 카메라 2대의 토픽이 ROS로 넘어오는지 확인 (임시)

  ros2 launch lv2_module6/ros2_ws/src/bringup/launch/gazebo_cam_test.launch.py            # 소스 경로로 바로 (빌드 불필요)
  ros2 launch lv2_module6/ros2_ws/src/bringup/launch/gazebo_cam_test.launch.py gui:=false  # 화면 없이
  ros2 launch lv2_module6/ros2_ws/src/bringup/launch/gazebo_cam_test.launch.py urdf:=beebot_normal  # 다른 로봇 모델

  띄우는 것: Gazebo(미로 + 퍽 3개 + 상자) · robot_state_publisher(TF) · 로봇 스폰(static) · 카메라 브리지 · 깊이 mm 변환
  나오는 토픽 (documents/API.md와 같은 이름·형식):
    /pole_cam/color/image_raw (rgb8) · /pole_cam/color/camera_info · /pole_cam/aligned_depth_to_color/image_raw (16UC1, mm)
    /pole_cam/depth/color/points · gripper_cam도 같음
  - URDF는 수정하지 않는다: config/gazebo/beebot_gz.urdf.xacro가 lv2_module6/urdf/<urdf>/urdf.xacro를 불러와 센서만 덧붙임
    (ros_system과 같은 urdf:=<폴더 이름> 규칙 · 메시는 Gazebo가 읽도록 file:// 경로로 넘김)
  - 로봇은 static(고정) — 아직 gz_ros2_control이 없음. 움직여 보려면 Gazebo 화면에서 로봇을 끌어 옮긴다(이동 도구 T)
  - 같은 LAN의 다른 ROS와 섞이지 않게: export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
  - Gazebo는 NVIDIA GPU로 그린다 (gpu:=nvidia, 드라이버가 있으면 기본) — 노트북처럼 GPU가 둘이면 기본이 내장 그래픽이라
    카메라 렌더가 느림(메시 로봇에서 15Hz/8Hz). PRIME 오프로드 환경변수로 RTX에 넘긴다. 끄려면 gpu:=default
  - 영상을 보내는 브리지·깊이 변환에는 config/gazebo/fastdds_large_shm.xml(공유메모리 칸 32MB)을 적용 —
    기본 칸(512KB)보다 큰 영상이 UDP로 빠지며 대부분 손실되던 문제 해결. 받는 쪽 노드는 설정 불필요
  필요: ros-lyrical-ros-gz-sim · ros-lyrical-ros-gz-bridge · xacro (호스트, 테스트베드 도커에는 Gazebo 없음)
"""
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler,
                            SetEnvironmentVariable)
from launch.event_handlers import OnShutdown
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# 이 파일 기준으로 lv2_module6 찾기 (소스에서 바로 실행 · colcon --symlink-install 둘 다)
HERE = Path(__file__).resolve().parent
LV2 = HERE.parents[3]
CFG = HERE.parent / 'config' / 'gazebo'
URDF_DIR = LV2 / 'urdf'  # urdf/<이름>/urdf.xacro + urdf/<이름>/meshes/
COURSE = LV2 / 'course'
SPAWN = (-1.835, 2.757)  # course/twin_closed_maze/walls.json spawn_xy_m
NVIDIA_EGL = Path('/usr/share/glvnd/egl_vendor.d/10_nvidia.json')  # NVIDIA 드라이버가 깔려 있으면 있음
NVIDIA_ENV = {'__NV_PRIME_RENDER_OFFLOAD': '1', '__GLX_VENDOR_LIBRARY_NAME': 'nvidia',  # GLX(화면 있는 렌더)
              '__EGL_VENDOR_LIBRARY_FILENAMES': str(NVIDIA_EGL)}                         # EGL(gui:=false 렌더)


def gazebo(context, world):
    """gui:=true/false · gpu:=nvidia/default 에 맞춰 `gz sim` 한 개를 띄운다"""
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    gpu = LaunchConfiguration('gpu').perform(context)
    env = None
    if gpu == 'nvidia':
        if NVIDIA_EGL.exists():
            env = NVIDIA_ENV
        else:
            print('[gazebo_cam_test] NVIDIA 드라이버가 없어 기본 GPU로 렌더링합니다')
    cmd = ['gz', 'sim', '-r'] + ([] if gui else ['-s']) + [world]
    return [ExecuteProcess(cmd=cmd, output='screen', additional_env=env)]


def generate_launch_description():
    for p in (URDF_DIR, COURSE, CFG):
        if not p.exists():
            raise FileNotFoundError(f'{p} 없음 — 소스 경로의 launch 파일로 실행하세요 (위 사용법)')
    name = LaunchConfiguration('urdf')
    robot_description = ParameterValue(Command([
        'xacro ', str(CFG / 'beebot_gz.urdf.xacro'),
        ' urdf:=', PathJoinSubstitution([str(URDF_DIR), name, 'urdf.xacro']),
        ' mesh_uri:=file://', PathJoinSubstitution([str(URDF_DIR), name, 'meshes']),
        ' mock_hardware:=true']), value_type=str)
    world = str(CFG / 'cam_test_world.sdf')
    large_shm = {'FASTDDS_DEFAULT_PROFILES_FILE': str(CFG / 'fastdds_large_shm.xml')}
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='Gazebo 화면 띄우기'),
        DeclareLaunchArgument('gpu', default_value='nvidia',
                              description='nvidia = NVIDIA GPU로 렌더링(드라이버 없으면 자동으로 기본) · default = 시스템 기본 GPU'),
        DeclareLaunchArgument('urdf', default_value='default',
                              description='로봇 모델 = lv2_module6/urdf/<이 이름>/urdf.xacro (ros_system과 같은 규칙)'),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH',
                               os.pathsep.join(filter(None, [str(COURSE), os.environ.get('GZ_SIM_RESOURCE_PATH')]))),
        OpaqueFunction(function=gazebo, args=[world]),
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[{'robot_description': robot_description, 'use_sim_time': True}]),
        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-topic', 'robot_description', '-name', 'beebot',
                        '-x', str(SPAWN[0]), '-y', str(SPAWN[1]), '-z', '0.0']),
        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
             parameters=[{'config_file': str(CFG / 'gz_bridge_cameras.yaml')}], additional_env=large_shm),
        ExecuteProcess(cmd=['python3', str(CFG / 'depth_mm.py')], output='screen', additional_env=large_shm),
        # `gz sim`은 서버를 별도 프로세스로 띄워 launch 종료 때 남는다 → 이 월드의 서버·GUI를 같이 끈다
        RegisterEventHandler(OnShutdown(on_shutdown=[
            ExecuteProcess(cmd=['pkill', '-f', 'gz-sim-.*cam_test_world.sdf'])])),
    ])
