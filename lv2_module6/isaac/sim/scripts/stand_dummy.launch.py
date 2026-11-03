"""ROS side of the stand-up + regrasp dummy (stand_dummy.sh): ros_system with ros2_control only
(controller_manager + arm_controller -> rl_arm_correction chain + gripper_controller) on the virtual OpenCR,
no SLAM / Nav2 / cameras / MoveIt / plan nodes (nothing else drives the arm). Sim time for every node.
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import SetParameter
from launch_ros.substitutions import FindPackageShare

ARGS = {'port': '/tmp/opencr', 'use_cameras': 'false', 'use_slam': 'false', 'use_nav2': 'false', 'use_moveit': 'false',
        'use_twist_mux': 'false', 'use_ekf': 'false'}


def generate_launch_description():
    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('ros_system'), 'launch', 'ros_system.launch.py'])),
            launch_arguments=ARGS.items()),
    ])
