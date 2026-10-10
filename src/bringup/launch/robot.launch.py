import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, EqualsSubstitution

def generate_launch_description():
    # launching mode
    # nav       = navigation
    # map       = mapping from existing database
    # map-new   = mapping from zero
        
    mode = LaunchConfiguration('mode')
    model = LaunchConfiguration('model')

    mode_declare = DeclareLaunchArgument(
        name='mode',
        default_value='nav',
        description="robot running mode"
    )

    model_declare = DeclareLaunchArgument(
        name='mode',
        default_value='ddmr',
        description='robot model ddmr/agv'
    )

    is_mapping = PythonExpression([
        "'", mode, "' in ['map', 'map-new']"
    ])

    share_dir = get_package_share_directory('bringup')
    rviz_config_file = os.path.join(share_dir, 'rviz', 'robot.rviz')
    
    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('ddmr_description'), 'launch', 'display.launch.py']
            )
        )
    )

    remote_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('remote'), 'launch', 'radiomaster_ER6.launch.py']
            )
        ),
        condition=IfCondition(is_mapping),
        launch_arguments={
            'sim' : 'true'
        }.items()
    )

    camera_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('ascamera'), 'launch','hp60c.launch.py']
            )
        )
    )

    canbus_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('canbus'), 'launch','canbus.launch.py']
            )
        )
    )

    slam_mapping_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('slam'), 'launch', 'mapping.launch.py']
            )
        ),
        launch_arguments={
            'mode': mode,
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
            'use_sim_time': 'false',
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
            'use_sim_time': 'false',
        }.items(),
        condition=IfCondition(EqualsSubstitution(mode, 'nav'))
    )

    display_node =  Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(EqualsSubstitution(mode, 'nav'))
    )

    kinematics_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('drive_controller'),
                'launch',
                'bringup.launch.py'
            ])
        )
    )

    camera_tf_link = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_camera_tf",
        arguments=[
            "--x", "0.0",
            "--y", "0.0",
            "--z", "0.0",
            "--roll", "1.57",
            "--pitch", "3.14",
            "--yaw", "1.57",
            "--frame-id", 'camera_link',
            "--child-frame-id", 'ascamera_hp60c_camera_link_0',
        ],
    )

    return LaunchDescription([
        mode_declare,
        model_declare,

        robot_description,
        canbus_node,
        remote_node,

        kinematics_node,
        

        camera_node,
        camera_tf_link,
        slam_mapping_node,
        slam_localization_node,
        slam_navigation_node,

        display_node,
    ])

