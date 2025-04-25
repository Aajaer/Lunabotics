from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    return LaunchDescription([

        # Delay TF: odom → base_link
        TimerAction(
            period=1.0,
            actions=[
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='static_tf_pub_odom_to_base',
                    arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
                    output='screen'
                )
            ]
        ),

        # Delay TF: base_link → laser
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='static_tf_pub_base_to_laser',
                    arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser'],
                    output='screen'
                )
            ]
        ),

        # RPLidar S2L node
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[
                {'serial_port': '/dev/ttyUSB0'},
                {'frame_id': 'laser'},
                {'angle_compensate': True},
                {'scan_mode': 'Standard'}
            ],
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

        # Delay SLAM Toolbox
        TimerAction(
            period=6.0,
            actions=[
                Node(
                    package='slam_toolbox',
                    executable='sync_slam_toolbox_node',
                    name='slam_toolbox',
                    parameters=[
                        {'use_sim_time': False},
                        {'odom_frame': 'odom'},
                        {'base_frame': 'base_link'},
                        {'map_frame': 'map'}
                    ],
                    remappings=[
                        ('scan', '/scan')
                    ],
                    output='screen'
                )
            ]
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
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
    ])
