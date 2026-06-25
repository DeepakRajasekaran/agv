#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from custom_interfaces.msg import ControllerState

class NavSimulator(Node):
    def __init__(self):
        super().__init__('nav_simulator')
        
        self.declare_parameter('turn_sequence', ['left', 'right', 'straight'])
        self.declare_parameter('loop_sequence', True)
        self.declare_parameter('nominal_speed', 0.2)
        
        self.turn_sequence = self.get_parameter('turn_sequence').value
        self.loop_sequence = self.get_parameter('loop_sequence').value
        self.nominal_speed = self.get_parameter('nominal_speed').value
        
        self.turn_index = 0
        self.current_state = ControllerState.IDLE
        
        self.pub_cmd_vel = self.create_publisher(Twist, '/nav/cmd_vel', 10)
        
        self.sub_state = self.create_subscription(
            ControllerState, '/controller_state', self.state_callback, 10)
            
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info(f'Nav Simulator started with sequence: {self.turn_sequence}')
        
    def state_callback(self, msg: ControllerState):
        prev_state = self.current_state
        self.current_state = msg.state
        
        # When we transition from JUNCTION_DETECTED to EXECUTE_TURN, we advance the sequence
        if prev_state == ControllerState.JUNCTION_DETECTED and self.current_state == ControllerState.EXECUTE_TURN:
            self.advance_sequence()
            
    def advance_sequence(self):
        if len(self.turn_sequence) == 0:
            return
            
        self.turn_index += 1
        
        if self.turn_index >= len(self.turn_sequence):
            if self.loop_sequence:
                self.turn_index = 0
                self.get_logger().info('Turn sequence finished. Looping back to start.')
            else:
                self.turn_index = -1
                self.get_logger().info('Turn sequence finished. Stopping navigation.')

    def timer_callback(self):
        msg = Twist()
        
        # If tracking is active, provide velocity
        if self.current_state in [ControllerState.FOLLOW_LINE, ControllerState.JUNCTION_DETECTED, ControllerState.EXECUTE_TURN]:
            
            # Sequence finished and no looping
            if self.turn_index == -1:
                msg.linear.x = 0.0
                msg.angular.z = 0.0
            else:
                msg.linear.x = self.nominal_speed
                
                # Apply angular bias based on current requested turn
                if len(self.turn_sequence) > 0:
                    current_turn = self.turn_sequence[self.turn_index]
                    if current_turn == 'left':
                        msg.angular.z = 0.5
                    elif current_turn == 'right':
                        msg.angular.z = -0.5
                    elif current_turn == 'straight':
                        msg.angular.z = 0.0
        
        self.pub_cmd_vel.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = NavSimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
