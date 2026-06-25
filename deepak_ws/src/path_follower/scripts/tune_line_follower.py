#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class SimulateInputsNode(Node):
    def __init__(self):
        super().__init__('simulate_inputs_node')
        
        # Subscribe to actual hardware topics
        self.left_sub = self.create_subscription(Float32, '/sensor/left_track_position', self.left_cb, 10)
        self.right_sub = self.create_subscription(Float32, '/sensor/right_track_position', self.right_cb, 10)
        
        # Publish the "simulated" average to the controller's parameterized topic
        self.track_pub = self.create_publisher(Float32, '/test/track_position', 10)
        
        self.left_val = 0.0
        self.right_val = 0.0
        
        # 50Hz continuous publisher
        self.timer = self.create_timer(0.02, self.publish_average) 

    def left_cb(self, msg):
        self.left_val = msg.data

    def right_cb(self, msg):
        self.right_val = msg.data

    def publish_average(self):
        avg_pos = (self.left_val + self.right_val) / 2.0
        
        msg = Float32()
        msg.data = avg_pos
        self.track_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SimulateInputsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
