"""Isaac Sim 팔 시연 공용 ROS — SLAM·Nav2 없이 bringup stand_test + Isaac 카메라 어댑터 (pick_load_demo.sh가 씀).

  ISAAC_ROS_LAUNCH=isaac_manipulator_demo.launch.py ISAAC_AUTOPLAY=1 isaac/scripts/standalone/isaac_sim.sh
"""
import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('bringup'), 'launch', 'stand_test.launch.py'])),
            launch_arguments={'port': '/tmp/opencr', 'use_cameras': 'false',
                              'grasp_min_effort': '0.0', 'miss_min_range': '0.1'}.items()),
        ExecuteProcess(cmd=['python3', os.path.join(os.path.dirname(os.path.abspath(__file__)), 'camera_adapter.py'),
                            '--ros-args', '-p', 'use_sim_time:=true'], output='screen'),
    ])
