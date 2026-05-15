from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
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
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'scan_topic': scan_topic},
            {'odom_frame': odom_frame},
            {'base_frame': base_frame},
            {'map_frame': map_frame},
            {'minimum_travel_distance': 0.25},
            {'minimum_travel_heading': 0.05},
            {'map_update_interval': 1.5},
            {'max_laser_range': 6.0},
            {'do_loop_closing': True},
            {'use_scan_matching': True},
            {'use_scan_barycenter': True},
            {'scan_buffer_size': 30},
            {'scan_buffer_maximum_scan_distance': 10.0},
            {'link_match_minimum_response_fine': 0.1},
            {'link_scan_maximum_distance': 1.5},
            {'coarse_search_angle_offset': 0.349},
            {'coarse_angle_resolution': 0.01},
            {'fine_search_angle_offset': 0.001},
            {'angle_variance_penalty': 0.5},
            {'distance_variance_penalty': 0.1},
            {'loop_match_minimum_chain_size': 5},
            {'loop_search_maximum_distance': 3.0},
            {'loop_match_maximum_variance_coarse': 2.0},
            {'loop_match_minimum_response_coarse': 0.50},
            {'loop_match_minimum_response_fine': 0.60},
            {'loop_search_space_dimension': 8.0},
            {'loop_search_space_resolution': 0.05},
            {'loop_search_space_smear_deviation': 0.03},
            {'correlation_search_space_dimension': 0.5},
            {'correlation_search_space_resolution': 0.01},
            {'correlation_search_space_smear_deviation': 0.1},
            {'transform_timeout': 0.2},
            {'tf_buffer_duration': 30.0},
            {'transform_publish_period': 0.02},
            {'message_filter_queue_size': 100},
            {'resolution': 0.05},
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
            {'use_sim_time': False},
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
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('scan_topic', default_value='/rm0/scan'),
        DeclareLaunchArgument('odom_frame', default_value='rm0/odom_truth'),
        DeclareLaunchArgument('base_frame', default_value='rm0/base_link_truth'),
        DeclareLaunchArgument('lidar_frame', default_value='rm0/lidar_link'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('yaw_offset', default_value='3.141592653589793'),
        DeclareLaunchArgument('lidar_yaw_offset', default_value='0.0'),
        DeclareLaunchArgument('cmd_yaw_rate_scale', default_value='1.0'),
        DeclareLaunchArgument('raw_yaw_rate_threshold', default_value='0.02'),
        ep_tof_launch,
        coppeliasim_odom_node,
        lidar_bridge_node,
        slam_node,
        explorer_node,
        rviz_node,
    ])
