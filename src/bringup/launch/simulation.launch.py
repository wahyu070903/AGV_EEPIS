import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    headless_sim = LaunchConfiguration('headless', default='false')

    robot_model = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('model_description'), 'launch', 'agv_world.launch.py']
            )
        ),
        launch_arguments={'headless': headless_sim}.items() 
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='headless',
            default_value='false',
            description='Whether to execute gzclient'
        ),
        robot_model,
    ])