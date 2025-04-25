#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Launch configuration
    channel_type = LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='1000000')
    frame_id = LaunchConfiguration('frame_id', default='laser')

    # Paths
    rviz_config = os.path.join(
        get_package_share_directory('sllidar_ros2'),
        'rviz',
        'sllidar_ros2.rviz'
    )

    cartographer_config = os.path.join(
        get_package_share_directory('motor_control_pkg'),
        'config'
    )

    return LaunchDescription([
        # Lidar node
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{
                'channel_type': channel_type,
                'serial_port': serial_port,
                'serial_baudrate': serial_baudrate,
                'frame_id': frame_id,
            }],
            output='screen'
        ),

        # Relay scan topic
        Node(
            package='topic_tools',
            executable='relay',
            name='relay_scan_topic',
            arguments=['/sllidar_node/scan', '/scan'],
            output='screen'
        ),

        # Scan filter node
        Node(
            package='motor_control_pkg',
            executable='scan_filter_node',
            name='scan_filter_node',
            output='screen'
        ),

        # Updated motor control node
        Node(
            package='motor_control_pkg',
            executable='lidar_motor_control',
            name='motor_control_node',
            output='screen'
        ),

        # Static TF: base_link -> laser
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='laser_to_base_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser'],
            output='screen'
        ),

        # Cartographer SLAM
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            parameters=[{
                'use_sim_time': False
            }],
            arguments=[
                '-configuration_directory', cartographer_config,
                '-configuration_basename', 'backpack_2d.lua'
            ],
            remappings=[
                ('scan', '/scan')
            ],
            output='screen'
        ),

        # Map generation
        Node(
            package='cartographer_ros',
            executable='occupancy_grid_node',
            name='occupancy_grid_node',
            parameters=[{
                'use_sim_time': False,
                'resolution': 0.05
            }],
            remappings=[
                ('submap_list', '/submap_list'),
                ('map', '/map')
            ],
            output='screen'
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        ),
    ])
