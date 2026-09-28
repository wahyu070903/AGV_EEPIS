# import os
# from ament_index_python.packages import get_package_share_directory

# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription
# from launch.substitutions import LaunchConfiguration
# from launch.conditions import IfCondition, UnlessCondition
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch_ros.actions import Node

# def generate_launch_description():
#     share_dir = get_package_share_directory('turtlebot3_gazebo')
#     ps4_config = os.path.join(share_dir, 'params', 'ps4_teleop.yaml')
#     headless = LaunchConfiguration('headless', default='false')

#     world_launch = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(share_dir, 'launch', 'turtlebot3_world.launch.py')
#         ),
#         launch_arguments={'headless': headless}.items()
#     )

#     rviz_config_file = os.path.join(
#         share_dir,
#         'rviz',
#         'tb3_gazebo.rviz'
#     )

#     rviz2 = Node(
#         package='rviz2',
#         executable='rviz2',
#         name='rviz2',
#         output='screen',
#         arguments=['-d', rviz_config_file],
#         parameters=[
#             {
#                 'use_sim_time': True,
#             }
#         ],
#     )

#     joy_node = Node(
#         package='joy',
#         executable='joy_node',
#         name='joy_node',
#         output='screen'
#     )

#     teleop_node = Node(
#         package='teleop_twist_joy',
#         executable='teleop_node',
#         name='teleop_twist_joy',
#         parameters=[ps4_config],
#         output='screen'
#     )

#     rtabmap = Node(
#         package='rtabmap_slam',
#         executable='rtabmap',
#         name='rtabmap',
#         output='screen',
#         parameters=[{
#             'frame_id': 'base_link',
#             'odom_frame_id': 'odom',
#             'subscribe_depth': True,
#             'subscribe_rgb': True,
#             'approx_sync': True,
#             'use_sim_time': True,
#             'sync_queue_size': 30,
#             'topic_queue_size': 30,
#         }],
#         remappings=[
#             ('rgb/image', '/camera/image_raw'),
#             ('depth/image', '/camera/depth/image_raw'),
#             ('rgb/camera_info', '/camera/camera_info'),
#             ('odom', '/odom'),
#         ]
#     )

#     return LaunchDescription([
#         world_launch,
#         rviz2,
#         joy_node,
#         teleop_node,
#         # rtabmap,
#     ])


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

    # ─────────────────────────────────────────────
    # Directories
    # ─────────────────────────────────────────────
    share_dir = get_package_share_directory('turtlebot3_gazebo')
    ps4_config = os.path.join(share_dir, 'params', 'ps4_teleop.yaml')
    rviz_config_file = os.path.join(share_dir, 'rviz', 'tb3_gazebo.rviz')

    # ─────────────────────────────────────────────
    # Launch Arguments
    # ─────────────────────────────────────────────
    declare_headless = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo in headless mode (no GUI)'
    )

    declare_localization = DeclareLaunchArgument(
        'localization',
        default_value='false',
        description='true = localization mode (pakai peta lama), false = mapping mode (buat peta baru)'
    )

    declare_database_path = DeclareLaunchArgument(
        'database_path',
        default_value=os.path.expanduser('~/.ros/rtabmap.db'),
        description='Path ke file database RTAB-Map (.db)'
    )

    declare_use_rtabmap_viz = DeclareLaunchArgument(
        'use_rtabmap_viz',
        default_value='false',
        description='Jalankan rtabmap_viz (GUI visualisasi khusus RTAB-Map)'
    )

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Jalankan RViz2'
    )

    # ─────────────────────────────────────────────
    # LaunchConfiguration references
    # ─────────────────────────────────────────────
    headless          = LaunchConfiguration('headless')
    localization      = LaunchConfiguration('localization')
    database_path     = LaunchConfiguration('database_path')
    use_rtabmap_viz   = LaunchConfiguration('use_rtabmap_viz')
    use_rviz          = LaunchConfiguration('use_rviz')

    # ─────────────────────────────────────────────
    # Gazebo World
    # ─────────────────────────────────────────────
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share_dir, 'launch', 'turtlebot3_world.launch.py')
        ),
        launch_arguments={'headless': headless}.items()
    )

    # ─────────────────────────────────────────────
    # RViz2
    # ─────────────────────────────────────────────
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_rviz)
    )

    # ─────────────────────────────────────────────
    # Joystick / Teleop
    # ─────────────────────────────────────────────
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen'
    )

    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy',
        parameters=[ps4_config],
        output='screen'
    )

    # ─────────────────────────────────────────────
    # Parameter RTAB-Map (shared antara mapping & localization)
    # ─────────────────────────────────────────────
    rtabmap_common_params = {
        # ── Frame & sync ──────────────────────────
        'frame_id':               'base_link',
        'odom_frame_id':          'odom',
        'map_frame_id':           'map',
        'publish_tf':             True,         # RTAB-Map publish TF map→odom
        'use_sim_time':           True,

        # ── Sensor input ──────────────────────────
        'subscribe_depth':        True,
        'subscribe_rgb':          True,
        'subscribe_scan':         False,        # set True jika tambah LiDAR
        'approx_sync':            True,         # wajib True jika kamera & odom beda clock
        'sync_queue_size':        30,
        'topic_queue_size':       30,

        # ── Loop closure & optimasi ───────────────
        # Jarak/sudut minimum sebelum tambah keyframe baru
        'Mem/IncrementalMemory':  'true',       # dioverride saat localization
        'RGBD/LinearUpdate':      '0.05',       # meter
        'RGBD/AngularUpdate':     '0.05',       # radian (~3 derajat)
        'RGBD/NewMapOdomChangeDistance': '0.0',

        # Aktifkan loop closure berbasis penampakan visual
        'Kp/MaxFeatures':         '500',
        'Vis/MinInliers':         '15',         # minimal fitur cocok untuk loop closure
        'Rtabmap/TimeThr':        '0',          # 0 = tidak ada batas waktu proses

        # ── Occupancy grid (untuk nav2) ───────────
        'Grid/Sensor':            '1',          # 0=laser, 1=depth camera
        'Grid/CellSize':          '0.05',       # resolusi peta (meter/pixel)
        'Grid/RangeMax':          '3.5',        # jangkauan maksimum depth (meter)
        'Grid/3D':                'false',      # export 2D grid untuk nav2
        'Grid/NormalsSegmentation': 'false',    # lebih stabil untuk indoor flat floor

        # ── Database ──────────────────────────────
        'database_path':          database_path,
        'depth_frame_id':   'camera_depth_optical_frame',
        'rgb_frame_id':     'camera_rgb_optical_frame',
    }

    # ─────────────────────────────────────────────
    # Remapping topic (sesuaikan dengan output Gazebo TB3 Waffle)
    # ─────────────────────────────────────────────
    rtabmap_remappings = [
        ('rgb/image',       '/camera/image_raw'),
        ('depth/image',     '/camera/depth/image_raw'),
        ('rgb/camera_info', '/camera/camera_info'),
        ('odom',            '/odom'),
        # Output yang dipakai nav2:
        ('grid_map',        '/map'),
    ]

    # ─────────────────────────────────────────────
    # RTAB-Map — MODE MAPPING
    # Hapus DB lama dan mulai peta baru setiap start
    # ─────────────────────────────────────────────
    rtabmap_mapping = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        arguments=['--delete_db_on_start'],     # mulai dari peta kosong
        parameters=[{
            **rtabmap_common_params,
            'Mem/IncrementalMemory': 'true',    # tambah node baru ke peta
            'Mem/InitWMWithAllNodes': 'false',
        }],
        remappings=rtabmap_remappings,
        condition=UnlessCondition(localization)
    )

    # ─────────────────────────────────────────────
    # RTAB-Map — MODE LOCALIZATION
    # Load DB lama, tidak tambah keyframe baru
    # ─────────────────────────────────────────────
    rtabmap_localization = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            **rtabmap_common_params,
            'Mem/IncrementalMemory': 'false',   # jangan tambah node baru
            'Mem/InitWMWithAllNodes': 'true',   # load semua node dari DB ke working memory
            'Rtabmap/StartNewMapOnLoopClosure': 'false',
        }],
        remappings=rtabmap_remappings,
        condition=IfCondition(localization)
    )

    # ─────────────────────────────────────────────
    # rtabmap_viz (opsional, GUI khusus RTAB-Map)
    # Lebih informatif dari RViz untuk debug loop closure
    # ─────────────────────────────────────────────
    rtabmap_viz = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        parameters=[{
            'use_sim_time':      True,
            'subscribe_depth':   True,
            'subscribe_rgb':     True,
            'frame_id':          'base_link',
            'approx_sync':       True,
        }],
        remappings=rtabmap_remappings,
        condition=IfCondition(use_rtabmap_viz)
    )

    # ─────────────────────────────────────────────
    # LaunchDescription
    # ─────────────────────────────────────────────
    return LaunchDescription([
        # Declare args
        declare_headless,
        declare_localization,
        declare_database_path,
        declare_use_rtabmap_viz,
        declare_use_rviz,

        # Simulasi
        world_launch,

        # Visualisasi
        rviz2,
        rtabmap_viz,

        # Teleop
        joy_node,
        teleop_node,

        # RTAB-Map (salah satu aktif sesuai mode)
        rtabmap_mapping,
        rtabmap_localization,
    ])