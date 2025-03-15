from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[
                {'serial_port': '/dev/ttyUSB0'},
                {'frame_id': 'laser'},
                {'angle_compensate': True}
            ],
            output='screen'
        ),
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[
                {'use_sim_time': False, 'queue_size':100},
                {'map_file_name': '/tmp/my_map'}
            ],
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', '/path/to/your/rviz_config.rviz'],
            output='screen'
        )
    ])
