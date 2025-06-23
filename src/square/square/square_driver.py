#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import time
import math


class SquareDriver(Node):
    def __init__(self):
        super().__init__('square_driver')
        self.publisher = self.create_publisher(TwistStamped, '/robot2/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.update_motion)

        self.state = "forward"
        self.last_switch_time = time.time()
        self.side_count = 0

    def update_motion(self):
        now = time.time()
        elapsed = now - self.last_switch_time

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        if self.state == "forward":
            msg.twist.linear.x = 0.1  # 0.1 m/s forward
            msg.twist.angular.z = 0.0
            if elapsed > 5.0:  # Go straight for 5 seconds (~0.5m)
                self.state = "turn"
                self.last_switch_time = now
                self.get_logger().info(f"🟩 Completed side {self.side_count + 1}, turning...")

        elif self.state == "turn":
            msg.twist.linear.x = 0.0
            msg.twist.angular.z = math.pi / 2  # ≈1.57 rad/s → 90°/s left turn
            if elapsed > 1.0:  # Turn for 1 second (90°)
                self.state = "forward"
                self.side_count += 1
                self.last_switch_time = now
                if self.side_count >= 4:
                    self.get_logger().info("🔁 Square complete, looping again.")
                    self.side_count = 0

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SquareDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 SquareDriver stopped.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
