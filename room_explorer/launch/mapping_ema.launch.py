import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    project_share = get_package_share_directory('project')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_frame = LaunchConfiguration('map_frame')
    odom_frame = LaunchConfiguration('odom_frame')
    base_frame = LaunchConfiguration('base_frame')
    laser_frame = LaunchConfiguration('laser_frame')
    input_scan_topic = LaunchConfiguration('input_scan_topic')
    scan_topic = LaunchConfiguration('scan_topic')
    slam_params = os.path.join(project_share, 'config', 'slam_toolbox.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('odom_frame', default_value='rm0/odom'),
        DeclareLaunchArgument('base_frame', default_value='rm0/base_link'),
        DeclareLaunchArgument('laser_frame', default_value='a'),
        DeclareLaunchArgument('input_scan_topic', default_value='/hokuyo'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),

        SetParameter(name='use_sim_time', value=use_sim_time),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_hokuyo_tf',
            arguments=[
                '--x', '0.20',
                '--y', '0.0',
                '--z', '0.15',
                '--roll', '0.0',
                '--pitch', '0.0',
                '--yaw', '0.0',
                '--frame-id', base_frame,
                '--child-frame-id', laser_frame,
            ],
        ),

        Node(
            package='project',
            executable='scan_filter',
            name='scan_filter',
            output='screen',
            remappings=[
                ('/hokuyo', input_scan_topic),
                ('/scan', scan_topic),
            ],
        ),

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                slam_params,
                {
                    'map_frame': map_frame,
                    'odom_frame': odom_frame,
                    'base_frame': base_frame,
                    'scan_topic': scan_topic,
                    'use_sim_time': use_sim_time,
                },
            ],
            remappings=[
                ('/scan', scan_topic),
                ('scan', scan_topic),
            ],
        ),
    ])
