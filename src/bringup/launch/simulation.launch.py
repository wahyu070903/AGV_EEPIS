import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    complete_sim = LaunchConfiguration('headless', default='false')

    DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Headless mode (true/false)'
    )

    robot_model = 