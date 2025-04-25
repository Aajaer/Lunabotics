import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class ScanFilterNode(Node):
    def __init__(self):
        super().__init__('scan_filter_node')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',  # Raw LiDAR data
            self.scan_callback,
            10
        )
        self.publisher = self.create_publisher(
            LaserScan,
            '/scan/front_filtered',  # Filtered output
            10
        )

    def scan_callback(self, msg: LaserScan):
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        angle_max = msg.angle_max
        self.get_logger().info(f"[DEBUG] angle_min: {math.degrees(angle_min):.2f}°, angle_max: {math.degrees(angle_max):.2f}°")

        num_readings = len(msg.ranges)

        # Log LiDAR angle range
        self.get_logger().info(
            f"angle_min: {math.degrees(angle_min):.2f}°, "
            f"angle_max: {math.degrees(msg.angle_max):.2f}°, "
            f"total points: {num_readings}"
        )

        def is_front_angle(angle_rad):
            angle_deg = math.degrees(angle_rad)
            angle_deg = (angle_deg + 360) % 360  # Normalize to [0, 360)
            return (angle_deg >= 315 or angle_deg <= 45)  # Keep ±45° in front

        filtered_ranges = list(msg.ranges)

        for i in range(num_readings):
            angle = angle_min + i * angle_increment
            if not is_front_angle(angle):
                filtered_ranges[i] = float('inf')  # Ignore rear data

        # Republish filtered scan
        filtered_scan = LaserScan()
        filtered_scan.header = msg.header
        filtered_scan.angle_min = msg.angle_min
        filtered_scan.angle_max = msg.angle_max
        filtered_scan.angle_increment = msg.angle_increment
        filtered_scan.time_increment = msg.time_increment
        filtered_scan.scan_time = msg.scan_time
        filtered_scan.range_min = msg.range_min
        filtered_scan.range_max = msg.range_max
        filtered_scan.ranges = filtered_ranges
        filtered_scan.intensities = msg.intensities

        self.publisher.publish(filtered_scan)

def main(args=None):
    rclpy.init(args=args)
    node = ScanFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
