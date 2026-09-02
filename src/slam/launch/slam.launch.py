from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_static_tf',
        arguments=[
            '--x', '0.20',
            '--y', '0.0',
            '--z', '0.50',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'camera_link',
        ],
        output='screen',
    )

    camera_link_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_link_to_ascamera_tf',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.0',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'camera_link',
            '--child-frame-id', 'ascamera_hp60c_camera_link_0',
        ],
        output='screen',
    )

    rgbd_sync = Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        name='rgbd_sync',
        output='screen',

        parameters=[{
            'approx_sync': True,
        }],

        remappings=[
            ('rgb/image','/ascamera_hp60c/camera_publisher/rgb0/image'),
            ('depth/image', '/ascamera_hp60c/camera_publisher/depth0/image_raw'),
            ('rgb/camera_info', '/ascamera_hp60c/camera_publisher/rgb0/camera_info'),
            ('rgbd_image', 'rgbd_image'),
        ],
    )

    rgbd_odometry = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        output='screen',

        parameters=[{
            'subscribe_rgbd': True,
            'frame_id': 'base_link',
        }],
        remappings=[
            ('rgbd_image', 'rgbd_image'),
        ],
    )

    rtabmap = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',

        parameters=[{
            # =========================
            # TF
            # =========================
            'frame_id': 'base_link',

            # =========================
            # Input RGB-D
            # =========================
            'subscribe_depth': False,
            'subscribe_rgbd': True,

            # =========================
            # Synchronization
            # =========================
            'queue_size': 10,
            'approx_sync': False,

            # =========================
            # SLAM / Mapping
            # =========================
            'RGBD/AngularUpdate': '0.01',
            'RGBD/LinearUpdate': '0.01',
            'RGBD/OptimizeFromGraphEnd': 'false',

            # =========================
            # Map
            # =========================
            'Grid/FromDepth': 'true',

            # =========================
            # Memory
            # =========================
            'Mem/IncrementalMemory': 'true',
            'Mem/InitWMWithAllNodes': 'false',
        }],

        remappings=[
            ('odom', '/odom'),
            ('rgbd_image', '/rgbd_image'),
        ],

        arguments=[
            '--delete_db_on_start',
        ],
    )

    return LaunchDescription([
        camera_tf,
        camera_link_tf,
        rgbd_sync,
        rgbd_odometry,
        rtabmap,
    ])