#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class AverageTrackNode(Node):
    def __init__(self):
        super().__init__('average_track_node')
        self.left_sub = self.create_subscription(Float32, '/sensor/left_track_position', self.left_cb, 10)
        self.right_sub = self.create_subscription(Float32, '/sensor/right_track_position', self.right_cb, 10)
        self.pub = self.create_publisher(Float32, '/sensor/track_position_test', 10)
        
        self.left_val = 0.0
        self.right_val = 0.0
        self.timer = self.create_timer(0.02, self.timer_cb) # 50Hz publish rate

    def left_cb(self, msg):
        self.left_val = msg.data

    def right_cb(self, msg):
        self.right_val = msg.data

    def timer_cb(self):
        avg = (self.left_val + self.right_val) / 2.0
        msg = Float32()
        msg.data = float(avg)
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = AverageTrackNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
