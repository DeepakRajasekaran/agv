#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import TwistStamped

class TuneLineFollowerNode(Node):
    def __init__(self):
        super().__init__('tune_line_follower_node')
        
        # ==========================================
        #           TUNING PARAMETERS
        # ==========================================
        self.Kp = 3.0      # Proportional gain (Steer strength)
        self.Ki = 0.0      # Integral gain (Eliminates steady drift)
        self.Kd = 0.1      # Derivative gain (Dampens oscillations)
        
        self.nominal_speed = 0.25 # Linear speed in m/s (approx 25% max speed)
        self.max_steer = 1.5      # Max angular velocity in rad/s
        # ==========================================

        self.left_sub = self.create_subscription(Float32, '/sensor/left_track_position', self.left_cb, 10)
        self.right_sub = self.create_subscription(Float32, '/sensor/right_track_position', self.right_cb, 10)
        
        # Publish directly to the diff_drive_controller
        self.cmd_pub = self.create_publisher(TwistStamped, '/diff_drive_controller/cmd_vel', 10)
        
        self.left_val = 0.0
        self.right_val = 0.0
        
        self.prev_error = 0.0
        self.integral = 0.0

        # Run control loop at 50Hz (0.02s)
        self.timer = self.create_timer(0.02, self.control_loop) 

    def left_cb(self, msg):
        self.left_val = msg.data

    def right_cb(self, msg):
        self.right_val = msg.data

    def control_loop(self):
        # Average track position as error (Meters)
        error = (self.left_val + self.right_val) / 2.0
        
        # PID calculation
        self.integral += error * 0.02
        derivative = (error - self.prev_error) / 0.02
        
        steer = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)
        self.prev_error = error
        
        # Clamp steering
        steer = max(-self.max_steer, min(self.max_steer, steer))
        
        # Speed scaling: slightly reduce linear velocity when steering sharply
        speed_scale = 1.0 - (0.5 * (abs(steer) / self.max_steer))
        linear_vel = self.nominal_speed * speed_scale
        
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = float(linear_vel)
        msg.twist.angular.z = float(steer)
        
        self.cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TuneLineFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Stop the robot when Ctrl+C is pressed
        msg = TwistStamped()
        msg.header.stamp = node.get_clock().now().to_msg()
        node.cmd_pub.publish(msg)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
