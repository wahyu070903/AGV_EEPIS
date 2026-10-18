import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    camera_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('ascamera'), 'launch', 'hp60c.launch.py']
            )
        )
    )
    canbus_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('canbus'), 'launch', 'canbus.launch.py']
            )
        )
    )

    return LaunchDescription([
        canbus_node,
        camera_node,
        Node(
            package='linetracker',
            executable='tracker',
            name='tracker',
            output='screen'
        ),
        Node(
            package='linetracker',
            executable='follower',
            name='follower',
            output='screen',
            # arguments=['--ros-args', '--log-level', 'DEBUG']
        )
    ])