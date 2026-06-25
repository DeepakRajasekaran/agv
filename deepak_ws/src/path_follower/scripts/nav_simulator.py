#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from custom_interfaces.msg import ControllerState

from std_srvs.srv import Trigger

class NavSimulator(Node):
    def __init__(self):
        super().__init__('nav_simulator')
        
        self.declare_parameter('nominal_speed', 0.2)
        self.nominal_speed = self.get_parameter('nominal_speed').value
        
        self.current_state = ControllerState.IDLE
        self.junction_count = 0
        self.active_turn_cmd = 0.0
        self.last_left_marker = False
        
        self.pub_cmd_vel = self.create_publisher(Twist, '/nav/cmd_vel', 10)
        
        self.sub_state = self.create_subscription(
            ControllerState, '/controller_state', self.state_callback, 10)
            
        self.sub_left_marker = self.create_subscription(
            Bool, '/sensor/left_marker', self.marker_callback, 10)
            
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('Nav Simulator started. First command defaults to STRAIGHT.')
        
        self.start_client = self.create_client(Trigger, '/path_follower_node/start')
        self.start_called = False
        
    def state_callback(self, msg: ControllerState):
        prev_state = self.current_state
        self.current_state = msg.state
        
        # Reset turn command when we successfully transition back to normal line following
        if prev_state in [ControllerState.EXECUTE_TURN, ControllerState.JUNCTION_DETECTED] and self.current_state == ControllerState.FOLLOW_LINE:
            self.get_logger().info('Resumed standard tracking. Resetting command to STRAIGHT.')
            self.active_turn_cmd = 0.0
            
    def marker_callback(self, msg: Bool):
        # Detect rising edge of left marker
        if msg.data and not self.last_left_marker:
            self.get_logger().info('Left marker detected!')
            
            # Check if this marker signifies a junction
            if self.current_state == ControllerState.JUNCTION_DETECTED:
                self.junction_count += 1
                if self.junction_count % 2 != 0:
                    self.get_logger().info(f'Junction #{self.junction_count} (Odd) -> Commanding LEFT')
                    self.active_turn_cmd = 0.5
                else:
                    self.get_logger().info(f'Junction #{self.junction_count} (Even) -> Commanding RIGHT')
                    self.active_turn_cmd = -0.5
            else:
                self.get_logger().info('State is not JUNCTION_DETECTED. Following the natural turn.')
                self.active_turn_cmd = 0.0
                
        self.last_left_marker = msg.data

    def timer_callback(self):
        # Auto-start if IDLE
        if self.current_state == ControllerState.IDLE and not self.start_called:
            if self.start_client.wait_for_service(timeout_sec=0.1):
                self.get_logger().info('Calling /path_follower_node/start...')
                req = Trigger.Request()
                self.start_client.call_async(req)
                self.start_called = True
        
        msg = Twist()
        
        # If tracking is active, provide velocity
        if self.current_state in [ControllerState.FOLLOW_LINE, ControllerState.JUNCTION_DETECTED, ControllerState.EXECUTE_TURN, ControllerState.RESUME_TRACKING]:
            msg.linear.x = self.nominal_speed
            msg.angular.z = self.active_turn_cmd
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            
        self.get_logger().info(f'Tick | State: {self.current_state} | cmd_vel: v={msg.linear.x:.2f}, w={msg.angular.z:.2f}')
        
        self.pub_cmd_vel.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = NavSimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
