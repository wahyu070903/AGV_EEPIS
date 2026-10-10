import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    database_path = LaunchConfiguration('database_path')
    rgb_topic = LaunchConfiguration('rgb_topic')
    rgb_info = LaunchConfiguration('rgb_info')
    depth_topic = LaunchConfiguration('depth_topic')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    declare_database_path = DeclareLaunchArgument(
        'database_path',
        default_value='/home/orsted/AGV_EEPIS/maps/robot_map.db',
        description='RTAB-Map database used for localization'
    )
    
    declare_rgb_topic = DeclareLaunchArgument(
        'rgb_topic',
        default_value='/ascamera_hp60c/camera_publisher/rgb0/image'
    )

    declare_rgb_info = DeclareLaunchArgument(
        'rgb_info',
        default_value='/ascamera_hp60c/camera_publisher/rgb0/camera_info'
    )

    declare_depth_topic = DeclareLaunchArgument(
        'depth_topic',
        default_value='/ascamera_hp60c/camera_publisher/depth0/image_raw'
    )

    localization_param = os.path.join(
        get_package_share_directory('slam'),
        'config',
        'rtabmap_localization.yaml'
    )

    camera_remappings = [
        (
            'rgb/image',
            rgb_topic
        ),
        (
            'rgb/camera_info',
            rgb_info
        ),
        (
            'depth/image',
            depth_topic
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
            'use_sim_time': use_sim_time,
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
                'database_path': database_path,
                'use_sim_time': use_sim_time,
            }
        ],

        arguments=[
            '--ros-args',
            '--log-level', 'ERROR'
        ]
    )

    return LaunchDescription([
        declare_database_path,
        declare_rgb_topic,
        declare_rgb_info,
        declare_depth_topic,
        rgbd_sync,
        rtabmap 
    ])