import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    mapping_param = os.path.join(
        get_package_share_directory('slam'), 'config', 'rtabmap_mapping.yaml'
    )

    mode = LaunchConfiguration('mode')
    database_path = LaunchConfiguration('database_path')
    rgb_topic = LaunchConfiguration('rgb_topic')
    rgb_info = LaunchConfiguration('rgb_info')
    depth_topic = LaunchConfiguration('depth_topic')

    declare_mode = DeclareLaunchArgument(
        'mode',
        default_value='map',
        description='SLAM running mode: map or map-new'
    )

    declare_database_path = DeclareLaunchArgument(
        'database_path',
        default_value='/home/orsted/AGV_EEPIS/maps/robot_map.db',
        description='Path for RTAB-Map database'
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
            'approx_sync': True,
            'approx_sync_max_interval': 0.05,
        }],

        remappings=camera_remappings,

        arguments=[
            '--ros-args',
            '--log-level', 'WARN'
        ],
    )

    def create_rtabmap(context):

        selected_mode = context.perform_substitution(mode)

        rtabmap_args = [
            '--ros-args',
            '--log-level', 'WARN'
        ]

        if selected_mode == 'map-new':
            rtabmap_args.insert(0, '-d')

        print(f'[SLAM] Mode: {selected_mode}')
        print(f'[SLAM] Database: {context.perform_substitution(database_path)}')

        rtabmap = Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',

            parameters=[
                mapping_param,
                {
                    'database_path': database_path,
                }
            ],

            arguments=rtabmap_args,
        )

        return [rtabmap]

    rtabmap_viz = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',

        parameters=[{
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',

            'subscribe_rgbd': True,
            'subscribe_odom_info': False,

            'use_sim_time': False,
        }],

        arguments=[
            '--ros-args',
            '--log-level', 'WARN'
        ]
    )

    return LaunchDescription([
        declare_mode,
        declare_database_path,
        declare_rgb_topic,
        declare_rgb_info,
        declare_depth_topic,

        rgbd_sync,
        OpaqueFunction(function=create_rtabmap),
        rtabmap_viz,
    ])