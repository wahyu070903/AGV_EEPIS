import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():
    mode = LaunchConfiguration('mode', default='mapping')
    
    # mapping = mapping with old database
    # mapping-new = mapping from zero
    # nav = navigating

    mode_declare = DeclareLaunchArgument(
        name='mode',
        default_value='mapping',
        description='simulation mode'
    )

    share_dir = get_package_share_directory('bringup')

    set_turtlebot_model = SetEnvironmentVariable(
        name='TURTLEBOT3_MODEL',
        value='waffle'
    )

    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('turtlebot3_gazebo'), 'launch', 'turtlebot3_world.launch.py']
            )
        )
    )

    return LaunchDescription([
        mode_declare,
        set_turtlebot_model,
        robot_description,
    ])
