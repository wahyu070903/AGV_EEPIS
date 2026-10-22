import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    canbus_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('canbus'), 'launch', 'canbus.launch.py']
            )
        )
    )

    return LaunchDescription([
        canbus_node,
        Node(
            package='linetracker',
            executable='tracker_seg',
            name='tracker_seg',
            output='screen'
        ),
        Node(
            package='linetracker',
            executable='follower_seg',
            name='follower_seg',
            output='screen',
        )
    ])