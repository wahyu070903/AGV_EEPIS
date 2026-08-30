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
    headless_sim = LaunchConfiguration('headless', default='false')
    use_display = LaunchConfiguration('use_display', default='false')

    share_dir = get_package_share_directory('bringup')
    rviz_config_file = os.path.join(share_dir, 'rviz', 'simulation.rviz')

    robot_model = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('model_description'), 'launch', 'agv_world.launch.py']
            )
        ),
        launch_arguments={'headless': headless_sim}.items() 
    )
    
    # Only when using Ps4 for controller
    # remote_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         PathJoinSubstitution(
    #             [FindPackageShare('remote'), 'launch', 'ps4_remote.launch.py']
    #         )
    #     )
    # )

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

    kinematic_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('drive_controller'), 'launch', 'bringup.launch.py']
            )
        )
    )

    display_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_display)
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='headless',
            default_value='false',
            description='Whether to execute gzclient'
        ),
        robot_model,
        display_node,
        remote_launch,
        kinematic_launch,
    ])