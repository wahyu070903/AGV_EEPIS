import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

def generate_launch_description():
    # launching mode
    # nav = navigation
    # map = mapping
    # free = freedrive
    
    mode = LaunchConfiguration('mode', default='map')
    model = LaunchConfiguration('model', default='ddmr')

    remote_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('remote'), 'launch', 'radiomaster_ER6.launch.py']
            )
        ),
        launch_arguments={
            'sim' : 'true'
        }.items()
    )

    canbus_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('canbus'), 'launch','canbus.launch.py']
            )
        )
    )

    camera_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('ascamera'), 'launch', 'ascamera.launch.py']
            )
        )
    )

    display_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ddmr_description'),
                'launch',
                'display.launch.py'
            ])
        ),
        launch_arguments={
            'gui': 'false',
        }.items()
    )


    return LaunchDescription([
        display_node,
        canbus_node,
    ])

