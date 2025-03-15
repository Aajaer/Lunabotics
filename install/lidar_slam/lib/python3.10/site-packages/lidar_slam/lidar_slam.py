import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R

class LidarSlamNode(Node):
    def __init__(self):
        super().__init__('lidar_slam_node')
        
        # Map parameters
        self.map_size = 500  # Grid map size in pixels
        self.resolution = 0.05  # Map resolution (meters per pixel)
        self.map = np.zeros((self.map_size, self.map_size), dtype=np.int8)  # Empty map
        self.pose = np.array([self.map_size // 2, self.map_size // 2, 0])  # Initial pose

        # Subscribe to Lidar scan topic
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        # Publish the occupancy grid map
        self.map_publisher = self.create_publisher(OccupancyGrid, '/map', 10)

        self.prev_points = None  # Store previous scan points

    def scan_to_points(self, scan):
        """ Convert Lidar scan (ranges) to (x, y) coordinates in robot frame """
        angles = np.linspace(scan.angle_min, scan.angle_max, len(scan.ranges))
        ranges = np.array(scan.ranges)
        ranges[ranges == float('inf')] = 0  # Remove infinite values

        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        return np.vstack((x, y))

    def icp_scan_matching(self, prev_points, curr_points):
        """ Use ICP to align two Lidar scans and estimate movement """
        if prev_points is None:
            return np.eye(3)  # No transformation on first scan

        transform_matrix, _ = cv2.estimateAffinePartial2D(prev_points.T, curr_points.T)
        if transform_matrix is None:
            return np.eye(3)  # Return identity if ICP fails

        affine_mat = np.eye(3)
        affine_mat[:2, :2] = transform_matrix[:, :2]
        affine_mat[:2, 2] = transform_matrix[:, 2]
        return affine_mat

    def update_pose(self, transform):
        """ Update robot pose using estimated transformation """
        new_pose = np.dot(transform, np.array([self.pose[0], self.pose[1], 1]))
        self.pose[:2] = new_pose[:2]
        self.pose[2] += np.arctan2(transform[1, 0], transform[0, 0])

    def add_scan_to_map(self, points):
        """ Convert scan points to map coordinates and update occupancy grid """
        for x, y in points.T:
            mx, my = int(self.pose[0] + x / self.resolution), int(self.pose[1] + y / self.resolution)
            if 0 <= mx < self.map_size and 0 <= my < self.map_size:
                self.map[my, mx] = 100  # Mark as occupied

    def publish_map(self):
        """ Publish the generated occupancy grid map """
        grid_msg = OccupancyGrid()
        grid_msg.header.frame_id = "map"
        grid_msg.info.resolution = self.resolution
        grid_msg.info.width = self.map_size
        grid_msg.info.height = self.map_size
        grid_msg.data = self.map.flatten().tolist()  # Convert to list format
        
        self.map_publisher.publish(grid_msg)
        self.get_logger().info("Published map")

    def scan_callback(self, scan):
        """ Process incoming Lidar scans """
        curr_points = self.scan_to_points(scan)
        transform = self.icp_scan_matching(self.prev_points, curr_points)
        self.update_pose(transform)
        self.add_scan_to_map(curr_points)
        self.prev_points = curr_points  # Store last scan for ICP
        self.publish_map()

def main(args=None):
    rclpy.init(args=args)
    node = LidarSlamNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
