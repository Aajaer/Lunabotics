import rclpy
from rclpy.node import Node
from launch_ros.actions import Node as LaunchNode
from launch import LaunchDescription, LaunchService
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import numpy as np
import threading

class LidarSlamNode(Node):
    def __init__(self):
        super().__init__('lidar_slam_node')

        self.start_lidar_node()

        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        #Publish map
        self.map_publisher = self.create_publisher(OccupancyGrid, '/map', 10)

        self.get_logger().info("Lidar SLAM Node Started")

    def start_lidar_node(self):
	#Retrieve package from SLLIDAR package from github
        def run_launch():
            launch_description = LaunchDescription([
                LaunchNode(
                    package='sllidar_ros2',
                    executable='sllidar_node',
                    name='sllidar_node',
                    parameters=[
                        {'serial_port': '/dev/ttyUSB0'},
                        {'frame_id': 'laser'},
                        {'angle_compensate': True}
                    ],
                    output='screen'
                )
            ])
            launch_service = LaunchService()
            launch_service.include_launch_description(launch_description)
            launch_service.run()

        #Run launch
        lidar_thread = threading.Thread(target=run_launch, daemon=True)
        lidar_thread.start()
        self.get_logger().info("Started SLLidar Node")

	#scan the ranges
    def scan_callback(self, scan):
        ranges = np.array(scan.ranges)
        ranges[ranges == float('inf')] = 0 
        self.get_logger().info(f"Received Lidar scan with {len(ranges)} points")
        grid_msg = OccupancyGrid()
        grid_msg.header.frame_id = "map"
        self.map_publisher.publish(grid_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LidarSlamNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
