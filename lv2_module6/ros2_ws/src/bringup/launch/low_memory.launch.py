"""Opt-in Jetson operation profile; all robot nodes remain under normal bringup.

LAUNCH_FILE=low_memory.launch.py ./start.sh
The profile uses the saved scene_sim 2D map. To create a map, override with
map:=slam map_backend:=nvblox.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    defaults = {
        'map': 'saved',
        'map_name': 'scene_sim',
        'map_backend': 'static_2d',
        'navigation_throttle': 'true',
        'navigation_infer_hz': '0.0',      # 주행 중 인지 끔 (bringup 기본과 같음, 11-15) — 도착 1 m 전부터 켬
        'navigation_full_near_m': '1.0',
        'cv_threads': '1',
        'nvblox_params_file': PathJoinSubstitution([
            FindPackageShare('ros_system'), 'config', 'nvblox_low_memory.yaml']),
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()],
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('bringup'), 'launch', 'bringup.launch.py'])),
            launch_arguments={k: LaunchConfiguration(k) for k in defaults}.items()),
    ])
