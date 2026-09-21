from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

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
            '--log-level', 'WARN'
        ],
    )

    rtabmap = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',

        parameters=[{

            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'map_frame_id': 'map',

            'subscribe_rgbd': True,
            'subscribe_odom_info': False,

            'wait_for_transform': 0.2,

            'use_sim_time': False,
        }],

        arguments=[
            '-d',
            '--ros-args',
            '--log-level', 'WARN'
        ],
        
    )

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
        }],

        arguments=[
            '--ros-args',
            '--log-level', 'WARN'
        ]
    )

    return LaunchDescription([
        rgbd_sync,
        rtabmap,
        rtabmap_viz,
    ])