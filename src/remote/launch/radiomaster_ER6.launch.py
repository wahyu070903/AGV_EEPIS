import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    GroupAction,
)
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    simulation = LaunchConfiguration('sim')
    return LaunchDescription([
        DeclareLaunchArgument(
            'sim',
            default_value='false',
            description='simulation mode',
        ),

        Node(
            package='remote',
            executable='radio_node',
            name='radio_receiver',
            parameters=[
                {
                    'sim' : simulation,
                }
            ],
            output='screen'
        )
    ])