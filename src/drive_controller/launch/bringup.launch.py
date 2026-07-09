import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    kinematics_node = Node(
        package='drive_controller',
        executable='kinematics_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
        }],
    )

    return LaunchDescription([
        kinematics_node,
    ])