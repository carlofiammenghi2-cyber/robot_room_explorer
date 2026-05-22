from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetParameter
from launch.launch_description_sources import FrontendLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('room_explorer')

    use_sim_time = LaunchConfiguration('use_sim_time')
    scan_topic = LaunchConfiguration('scan_topic')
    odom_frame = LaunchConfiguration('odom_frame')
    base_frame = LaunchConfiguration('base_frame')
    lidar_frame = LaunchConfiguration('lidar_frame')
    map_frame = LaunchConfiguration('map_frame')
    yaw_offset = LaunchConfiguration('yaw_offset')
    lidar_yaw_offset = LaunchConfiguration('lidar_yaw_offset')
    cmd_yaw_rate_scale = LaunchConfiguration('cmd_yaw_rate_scale')
    raw_yaw_rate_threshold = LaunchConfiguration('raw_yaw_rate_threshold')
    sync = LaunchConfiguration('sync')

    ep_tof_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'ep_tof.launch.xml')
        )
    )

    lidar_bridge_node = Node(
        package='room_explorer',
        executable='lidar_bridge',
        name='lidar_bridge',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[
            {'use_sim_time': use_sim_time},
            {'scan_topic': scan_topic},
            {'lidar_frame': lidar_frame},
        ]
    )

    slam_node = Node(
        package='slam_toolbox',
        executable=PythonExpression(["'sync_slam_toolbox_node' if '", sync, "' == 'true' else 'async_slam_toolbox_node'"]),
        name='slam_toolbox',
        output='log',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'scan_topic': scan_topic},
            {'odom_frame': odom_frame},
            {'base_frame': base_frame},
            {'map_frame': map_frame},
            {'max_laser_range': 6.0},
            {'minimum_travel_distance': 0.25},
            {'minimum_travel_heading': 0.05},
            {'resolution': 0.05},
            {'transform_timeout': 0.2},
            {'transform_publish_period': 0.02},
            {'do_loop_closing': True},
            {'use_scan_matching': True},
        ]
    )


    explorer_node = Node(
        package='room_explorer',
        executable='room_explorer_node',
        name='room_explorer_node',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    coppeliasim_odom_node = Node(
        package='room_explorer',
        executable='coppeliasim_odom',
        name='coppeliasim_odom',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[
            {'use_sim_time': use_sim_time},
            {'publish_tf': True},
            {'odom_frame': odom_frame},
            {'base_frame': base_frame},
            {'lidar_frame': lidar_frame},
            {'yaw_offset': yaw_offset},
            {'lidar_yaw_offset': lidar_yaw_offset},
            {'cmd_yaw_rate_scale': cmd_yaw_rate_scale},
            {'raw_yaw_rate_threshold': raw_yaw_rate_threshold},
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('scan_topic', default_value='/rm0/scan'),
        DeclareLaunchArgument('odom_frame', default_value='rm0/odom_truth'),
        DeclareLaunchArgument('base_frame', default_value='rm0/base_link_truth'),
        DeclareLaunchArgument('lidar_frame', default_value='rm0/lidar_link'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('yaw_offset', default_value='3.141592653589793'),
        DeclareLaunchArgument('lidar_yaw_offset', default_value='0.0'),
        DeclareLaunchArgument('cmd_yaw_rate_scale', default_value='1.0'),
        DeclareLaunchArgument('raw_yaw_rate_threshold', default_value='0.02'),
        DeclareLaunchArgument('sync', default_value='false'),
        ep_tof_launch,
        coppeliasim_odom_node,
        lidar_bridge_node,
        slam_node,
        # explorer_node,
        rviz_node,
    ])
