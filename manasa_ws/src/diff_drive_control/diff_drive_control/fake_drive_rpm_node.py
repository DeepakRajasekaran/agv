#!/usr/bin/env python3
"""
fake_drive_rpm_node
--------------------
TEST-ONLY helper. Simulates a perfect hardware response by echoing
whatever is published on /cmd_rpm straight back out as /drive_rpm.

Use this ONLY while you don't have real motor controllers / encoders
publishing /drive_rpm yet, so you can verify the full pipeline:

    keyboard -> /cmd_vel -> diff_drive_controller_node -> /cmd_rpm
             -> fake_drive_rpm_node -> /drive_rpm
             -> diff_drive_controller_node -> /odom

Stop this node and let your real firmware/driver publish /drive_rpm
once actual hardware is connected.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class FakeDriveRpmNode(Node):

    def __init__(self):
        super().__init__('fake_drive_rpm_node')
        self.sub = self.create_subscription(
            Float32MultiArray, '/cmd_rpm', self.callback, 10)
        self.pub = self.create_publisher(Float32MultiArray, '/drive_rpm', 10)
        self.get_logger().warn(
            'fake_drive_rpm_node is running: simulating perfect hardware feedback. '
            'This is for testing only — do not run alongside a real motor driver.'
        )

    def callback(self, msg: Float32MultiArray):
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeDriveRpmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
