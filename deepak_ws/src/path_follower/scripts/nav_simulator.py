#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Twist
from custom_interfaces.msg import ControllerState
from custom_interfaces.srv import SelectTrack

from std_srvs.srv import Trigger

class NavSimulator(Node):
    def __init__(self):
        super().__init__('nav_simulator')
        
        self.declare_parameter('nominal_speed', 0.2)
        self.nominal_speed = self.get_parameter('nominal_speed').value
        
        self.add_on_set_parameters_callback(self.parameters_callback)
        
        self.current_state = ControllerState.IDLE
        self.junction_count = 0
        self.pub_cmd_vel = self.create_publisher(Twist, '/nav/cmd_vel', 10)
        
        self.sub_state = self.create_subscription(
            ControllerState, '/controller_state', self.state_callback, 10)
            
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('Nav Simulator started. First command defaults to STRAIGHT.')
        
        self.start_client = self.create_client(Trigger, '/path_follower_node/start')
        self.select_track_client = self.create_client(SelectTrack, '/path_follower_node/select_track')
        self.start_called = False
        self.error_start_time = None
        
    def parameters_callback(self, params):
        for param in params:
            if param.name == 'nominal_speed':
                self.nominal_speed = param.value
                self.get_logger().info(f"Updated nominal_speed to {self.nominal_speed}")
        return SetParametersResult(successful=True)
        
    def state_callback(self, msg: ControllerState):
        prev_state = self.current_state
        self.current_state = msg.state
        
        # Detect transition into JUNCTION_DETECTED
        if prev_state != ControllerState.JUNCTION_DETECTED and self.current_state == ControllerState.JUNCTION_DETECTED:
            self.junction_count += 1
            req = SelectTrack.Request()
            
            if self.junction_count % 2 != 0:
                self.get_logger().info(f'Junction #{self.junction_count} (Odd) -> Calling service to track LEFT (1)')
                req.track_id = 1
            else:
                self.get_logger().info(f'Junction #{self.junction_count} (Even) -> Calling service to track RIGHT (2)')
                req.track_id = 2
                
            if self.select_track_client.wait_for_service(timeout_sec=0.5):
                self.select_track_client.call_async(req)
            else:
                self.get_logger().error('Service /path_follower_node/select_track not available!')
                
        # Logging standard track resumption
        if prev_state == ControllerState.JUNCTION_DETECTED and self.current_state == ControllerState.FOLLOW_LINE:
            self.get_logger().info('Resumed standard tracking (Controller auto-reset track to AVERAGE).')

    def timer_callback(self):
        # Auto-start if IDLE
        if self.current_state == ControllerState.IDLE and not self.start_called:
            if self.start_client.wait_for_service(timeout_sec=0.1):
                self.get_logger().info('Calling /path_follower_node/start...')
                req = Trigger.Request()
                self.start_client.call_async(req)
                self.start_called = True
        
        # Auto-recovery if ERROR
        if self.current_state == ControllerState.ERROR:
            if self.error_start_time is None:
                self.error_start_time = self.get_clock().now()
            else:
                elapsed = (self.get_clock().now() - self.error_start_time).nanoseconds / 1e9
                if elapsed > 2.0:
                    self.get_logger().warn('Auto-recovering from ERROR state...')
                    if self.start_client.wait_for_service(timeout_sec=0.1):
                        req = Trigger.Request()
                        self.start_client.call_async(req)
                        self.error_start_time = None
        else:
            self.error_start_time = None
        
        msg = Twist()
        
        # If tracking is active, provide velocity
        if self.current_state in [ControllerState.FOLLOW_LINE, ControllerState.JUNCTION_DETECTED, ControllerState.RESUME_TRACKING]:
            msg.linear.x = self.nominal_speed
            msg.angular.z = 0.0
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
