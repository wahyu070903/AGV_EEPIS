import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution,PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.substitutions import EqualsSubstitution


def generate_launch_description():
    mode = LaunchConfiguration('mode')
    
    # map = mapping with old database
    # map-new = mapping from zero
    # nav = navigating

    mode_declare = DeclareLaunchArgument(
        name='mode',
        default_value='nav',
        description='simulation mode'
    )

    is_mapping = PythonExpression([
        "'", mode, "' in ['map', 'map-new']"
    ])
        
    share_dir = get_package_share_directory('bringup')
    rviz_config_file = os.path.join(share_dir, 'rviz', 'simulation.rviz')
    
    set_turtlebot_model = SetEnvironmentVariable(
        name='TURTLEBOT3_MODEL',
        value='waffle'
    )

    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('turtlebot3_gazebo'), 'launch', 'turtlebot3_world.launch.py']
            )
        ),
        launch_arguments={
            'sim' : 'true',
            'x_pose' : '-2.0',
            'y_pose' : '-0.5',
            'headless' : 'false'
        }.items()
    )

    remote_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('remote'),'launch', 'ps4_remote.launch.py']
            )
        ),
        condition=IfCondition(is_mapping)
    )

    slam_mapping_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('slam'), 'launch', 'mapping.launch.py']
            )
        ),
        launch_arguments={
            'mode': mode,
            'rgb_topic': '/camera/image_raw',
            'rgb_info': '/camera/camera_info',
            'depth_topic': '/camera/depth/image_raw'
        }.items(),
        condition=IfCondition(is_mapping)
    )

    slam_localization_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('slam'), 'launch', 'localization.launch.py']
            )
        ),
        launch_arguments={
            'user_sim_time': 'true',
            'rgb_topic': '/camera/image_raw',
            'rgb_info': '/camera/camera_info',
            'depth_topic': '/camera/depth/image_raw'
        }.items(),
        condition=IfCondition(EqualsSubstitution(mode, 'nav'))
    )

    slam_navigation_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('slam'), 'launch', 'navigation.launch.py']
            )
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'rgb_info': '/camera/camera_info',
            'depth_topic': '/camera/depth/image_raw'
        }.items(),
        condition=IfCondition(EqualsSubstitution(mode, 'nav'))
    )

    display_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(EqualsSubstitution(mode, 'nav'))
    )

    return LaunchDescription([
        mode_declare,
        set_turtlebot_model,
        robot_description,

        remote_node,

        slam_mapping_node,
        slam_localization_node,
        slam_navigation_node,

        display_node,
    ])
