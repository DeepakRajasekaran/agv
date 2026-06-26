#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from std_srvs.srv import Trigger
from custom_interfaces.msg import ControllerState
from custom_interfaces.srv import SelectTrack

class JunctionManager(Node):
    def __init__(self):
        super().__init__('junction_manager')
        
        # State tracking
        self.current_state = ControllerState.IDLE
        self.nav_angular_z = 0.0
        
        self.track_pos = 0.0
        self.left_track_pos = 0.0
        self.right_track_pos = 0.0
        
        self.current_track_selection = -1
        
        # Divergence limit in meters (default 0.15m)
        self.declare_parameter('divergence_limit', 0.150)
        self.divergence_limit = self.get_parameter('divergence_limit').get_parameter_value().double_value
        
        # Side drift tolerance in meters (default 0.02m)
        self.declare_parameter('drift_tolerance', 0.020)
        self.drift_tolerance = self.get_parameter('drift_tolerance').get_parameter_value().double_value
        
        # Subscribers
        self.sub_state = self.create_subscription(
            ControllerState, '/controller_state', self.state_callback, 10)
            
        self.sub_cmd_vel = self.create_subscription(
            Twist, '/nav/cmd_vel', self.cmd_vel_callback, 10)
            
        self.sub_track = self.create_subscription(
            Float32, '/sensor/track_position', self.track_pos_callback, 10)
            
        self.sub_left = self.create_subscription(
            Float32, '/sensor/left_track_position', self.left_track_callback, 10)
            
        self.sub_right = self.create_subscription(
            Float32, '/sensor/right_track_position', self.right_track_callback, 10)
            
        # Service Clients to MGS1600
        self.cli_fork_left = self.create_client(Trigger, '/sensor/follow_left')
        self.cli_fork_right = self.create_client(Trigger, '/sensor/follow_right')
        self.cli_fork_clear = self.create_client(Trigger, '/sensor/clear_follow')
        
        # Service Client to PidController
        self.cli_select_track = self.create_client(SelectTrack, '/path_follower_node/select_track')
        
        # Fast timer to process divergence limits and drift
        self.timer = self.create_timer(0.05, self.process_track_logic)
        
        self.get_logger().info('Junction Manager Middleware Initialized.')
        
    def state_callback(self, msg: ControllerState):
        prev_state = self.current_state
        self.current_state = msg.state
            
        # Reset decision flag and clear sensor tracking when merging back
        if self.current_state == ControllerState.RESUME_TRACKING and prev_state != ControllerState.RESUME_TRACKING:
            self.call_service(self.cli_fork_clear, 'clear_follow')
            self.set_controller_track(0)
            
    def cmd_vel_callback(self, msg: Twist):
        self.nav_angular_z = msg.angular.z
        
    def track_pos_callback(self, msg: Float32):
        self.track_pos = msg.data
            
    def left_track_callback(self, msg: Float32):
        self.left_track_pos = msg.data
        
    def right_track_callback(self, msg: Float32):
        self.right_track_pos = msg.data

    def process_track_logic(self):
        # Calculate track divergence
        divergence = abs(self.left_track_pos - self.right_track_pos)
        
        if divergence >= self.divergence_limit:
            # When tracks diverge, use the side drift to pick the track
            if self.track_pos < -self.drift_tolerance:
                self.update_tracking(2, self.cli_fork_right, 'follow_right')
            elif self.track_pos > self.drift_tolerance:
                self.update_tracking(1, self.cli_fork_left, 'follow_left')
        else:
            # Just do averaging until it exceeds divergence
            self.update_tracking(0, self.cli_fork_clear, 'clear_follow')
                
    def update_tracking(self, track_id, client, srv_name):
        if self.current_track_selection != track_id:
            self.get_logger().info(f'Switching track: divergence={abs(self.left_track_pos - self.right_track_pos):.3f}, track_pos={self.track_pos:.3f} -> selecting track {track_id}')
            self.current_track_selection = track_id
            self.set_controller_track(track_id)
            self.call_service(client, srv_name)

    def set_controller_track(self, track_id):
        if not self.cli_select_track.wait_for_service(timeout_sec=0.1):
            return
        req = SelectTrack.Request()
        req.track_id = track_id
        self.cli_select_track.call_async(req)

    def call_service(self, client, srv_name):
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f'Service {srv_name} not available!')
            return
            
        req = Trigger.Request()
        future = client.call_async(req)
        future.add_done_callback(
            lambda fut: self.service_callback(fut, srv_name))
            
    def service_callback(self, future, srv_name):
        try:
            res = future.result()
            if res.success:
                self.get_logger().info(f'Successfully called {srv_name}: {res.message}')
            else:
                self.get_logger().error(f'Failed to call {srv_name}: {res.message}')
        except Exception as e:
            self.get_logger().error(f'Service call {srv_name} failed: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = JunctionManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
