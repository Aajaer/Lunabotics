import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32
import math

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan/front_filtered',  # <- Uses filtered topic
            self.scan_callback,
            10
        )
        self.motor_command_publisher = self.create_publisher(Int32, '/motor_command', 10)
        self.obstacle_distance_threshold = 1.0  # meters

    def scan_callback(self, msg: LaserScan):
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment

        front_min_angle = math.radians(-90)
        front_max_angle = math.radians(90)

        obstacle_detected = False

        for i, distance in enumerate(msg.ranges):
            angle = angle_min + i * angle_increment
            if front_min_angle <= angle <= front_max_angle and 0.0 < distance < self.obstacle_distance_threshold:
                obstacle_detected = True
                break

        command_msg = Int32()
        command_msg.data = 0 if obstacle_detected else 1
        self.motor_command_publisher.publish(command_msg)

def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorController()
    rclpy.spin(motor_controller)
    motor_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
