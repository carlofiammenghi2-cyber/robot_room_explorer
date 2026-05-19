import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap

def generate_launch_description():
    pkg_share = get_package_share_directory('room_explorer')
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    scan_topic = LaunchConfiguration('scan_topic')
    odom_frame = LaunchConfiguration('odom_frame')
    base_frame = LaunchConfiguration('base_frame')
    lidar_frame = LaunchConfiguration('lidar_frame')
    yaw_offset = LaunchConfiguration('yaw_offset')
    lidar_yaw_offset = LaunchConfiguration('lidar_yaw_offset')
    cmd_yaw_rate_scale = LaunchConfiguration('cmd_yaw_rate_scale')
    raw_yaw_rate_threshold = LaunchConfiguration('raw_yaw_rate_threshold')
    
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    ep_tof_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'ep_tof.launch.xml'))
    )

    lidar_bridge_node = Node(
        package='room_explorer', executable='lidar_bridge', name='lidar_bridge', output='log',
        respawn=True, respawn_delay=2.0,
        parameters=[{'use_sim_time': use_sim_time}, {'scan_topic': scan_topic}, {'lidar_frame': lidar_frame}]
    )

    coppeliasim_odom_node = Node(
        package='room_explorer', executable='coppeliasim_odom', name='coppeliasim_odom', output='log',
        respawn=True, respawn_delay=2.0,
        parameters=[
            {'use_sim_time': use_sim_time}, {'publish_tf': True}, {'odom_frame': odom_frame},
            {'base_frame': base_frame}, {'lidar_frame': lidar_frame}, {'yaw_offset': yaw_offset},
            {'lidar_yaw_offset': lidar_yaw_offset}, {'cmd_yaw_rate_scale': cmd_yaw_rate_scale},
            {'raw_yaw_rate_threshold': raw_yaw_rate_threshold},
        ]
    )

    # Nav2 bringup within a group to apply remapping without extra packages
    nav2_bringup_group = GroupAction(
        actions=[
            SetRemap(src='/cmd_vel', dst='/rm0/cmd_vel'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
                launch_arguments={
                    'map': map_yaml_file,
                    'use_sim_time': use_sim_time,
                    'params_file': params_file,
                    'use_composition': 'True',
                    'container_name': 'nav2_container'}.items(),
            ),
        ]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('scan_topic', default_value='/rm0/scan'),
        DeclareLaunchArgument('odom_frame', default_value='rm0/odom_truth'),
        DeclareLaunchArgument('base_frame', default_value='rm0/base_link_truth'),
        DeclareLaunchArgument('lidar_frame', default_value='rm0/lidar_link'),
        DeclareLaunchArgument('yaw_offset', default_value='3.141592653589793'),
        DeclareLaunchArgument('lidar_yaw_offset', default_value='0.0'),
        DeclareLaunchArgument('cmd_yaw_rate_scale', default_value='1.0'),
        DeclareLaunchArgument('raw_yaw_rate_threshold', default_value='0.02'),

        DeclareLaunchArgument('map', default_value=os.path.join(pkg_share, 'config', 'mappa_casa.yaml')),
        DeclareLaunchArgument('params_file', default_value=os.path.join(pkg_share, 'config', 'nav2_params.yaml')),

        ep_tof_launch,
        coppeliasim_odom_node,
        lidar_bridge_node,
        nav2_bringup_group,
        
        Node(
            package='rviz2', executable='rviz2', name='rviz2', output='log',
            arguments=['-d', os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
