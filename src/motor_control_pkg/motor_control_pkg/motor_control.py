#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')

        # Publishers for PWM and direction
        self.pwm_pub = self.create_publisher(Int32, 'motor_pwm', 10)
        self.dir_pub = self.create_publisher(Int32, 'motor_dir', 10)

        # Subscribe to the distance sensor topic (just for logging)
        self.create_subscription(
            Int32,
            'distance_mm',
            self.distance_callback,
            10
        )

        # Initial motor commands
        self.current_pwm = 40      # speed: 0–255

        # Internal counter to track timer ticks
        self._tick = 0

        # Timer: publish commands at 1 Hz
        self.timer = self.create_timer(1.0, self.publish_motor_commands)

    def publish_motor_commands(self):
        # Decide direction based on tick count:
        #   ticks 0–1 → forward (0)
        #   ticks 2–3 → backward (1)
        # then wrap every 4 ticks → 4 s cycle
        dir_val = 0 if (self._tick % 4) < 2 else 1
        self._tick += 1

        # Publish PWM
        pwm_msg = Int32(data=self.current_pwm)
        self.pwm_pub.publish(pwm_msg)
        self.get_logger().info(f'▶ motor_pwm: {pwm_msg.data}')

        # Publish direction
        dir_msg = Int32(data=dir_val)
        self.dir_pub.publish(dir_msg)
        self.get_logger().info(f'▶ motor_dir: {dir_msg.data} ' +
                               f'({"FORWARD" if dir_val==0 else "BACKWARD"})')

    def distance_callback(self, msg: Int32):
        # Just log distance; no effect on direction
        self.get_logger().info(f'📏 distance_mm: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
