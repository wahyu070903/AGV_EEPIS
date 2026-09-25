import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    database_path = LaunchConfiguration('database_path')

    declare_database_path = DeclareLaunchArgument(
        'database_path',
        default_value='/home/orsted/AGV_EEPIS/maps/robot_map.db',
        description='RTAB-Map database used for localization'
    )
    
    localization_param = os.path.join(
        get_package_share_directory('slam'),
        'config',
        'rtabmap_localization.yaml'
    )

    camera_remappings = [
        (
            'rgb/image',
            '/ascamera_hp60c/camera_publisher/rgb0/image'
        ),
        (
            'rgb/camera_info',
            '/ascamera_hp60c/camera_publisher/rgb0/camera_info'
        ),
        (
            'depth/image',
            '/ascamera_hp60c/camera_publisher/depth0/image_raw'
        ),
        (
            'odom',
            '/odom'
        ),
    ]

    rgbd_sync = Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        name='rgbd_sync',
        output='screen',

        parameters=[{
            'approx_sync': True,
            'approx_sync_max_interval': 0.05,
        }],

        remappings=camera_remappings,

        arguments=[
            '--ros-args',
            '--log-level', 'ERROR'
        ],
    )

    rtabmap =  Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',

        parameters=[
            localization_param,
            {
                'database_path': database_path
            }
        ],

        arguments=[
            '--ros-args',
            '--log-level', 'ERROR'
        ]
    )

    return LaunchDescription([
        declare_database_path,
        rgbd_sync,
        rtabmap 
    ])