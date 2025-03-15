import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.motor_command_publisher = self.create_publisher(Int32, '/motor_command', 10)
        self.obstacle_distance_threshold = 100000000  # Distance threshold in meters

    def scan_callback(self, msg: LaserScan):
        # Check if an obstacle is detected
        obstacle_detected = any(
            distance < self.obstacle_distance_threshold for distance in msg.ranges if distance > 0.0
        )

        # Publish a motor command based on detection
        command_msg = Int32()
        command_msg.data = 0 if obstacle_detected else 1  # 0 = stop, 1 = move forward
        self.motor_command_publisher.publish(command_msg)

def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorController()
    rclpy.spin(motor_controller)
    motor_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
