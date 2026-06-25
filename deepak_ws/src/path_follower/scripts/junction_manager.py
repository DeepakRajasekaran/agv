#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Bool
from std_srvs.srv import Trigger
from custom_interfaces.msg import ControllerState
from custom_interfaces.srv import SelectTrack

class JunctionManager(Node):
    def __init__(self):
        super().__init__('junction_manager')
        
        # State tracking
        self.current_state = ControllerState.IDLE
        self.nav_angular_z = 0.0
        
        self.left_track_pos = 0.0
        self.right_track_pos = 0.0
        self.last_known_straight_pos = 0.0
        
        self.left_marker = False
        self.right_marker = False
        
        self.has_made_decision = False
        self.current_track_selection = -1
        
        # Divergence limit in meters (default 0.15m)
        self.declare_parameter('divergence_limit', 0.150)
        self.divergence_limit = self.get_parameter('divergence_limit').get_parameter_value().double_value
        
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
            
        self.sub_left_marker = self.create_subscription(
            Bool, '/sensor/left_marker', self.left_marker_callback, 10)
            
        self.sub_right_marker = self.create_subscription(
            Bool, '/sensor/right_marker', self.right_marker_callback, 10)
            
        # Service Clients to MGS1600
        self.cli_fork_left = self.create_client(Trigger, '/sensor/follow_left')
        self.cli_fork_right = self.create_client(Trigger, '/sensor/follow_right')
        self.cli_fork_clear = self.create_client(Trigger, '/sensor/clear_follow')
        
        # Service Client to PidController
        self.cli_select_track = self.create_client(SelectTrack, '/path_follower_node/select_track')
        
        # Fast timer to process divergence limits and markers
        self.timer = self.create_timer(0.05, self.process_track_logic)
        
        self.get_logger().info('Junction Manager Middleware Initialized.')
        
    def state_callback(self, msg: ControllerState):
        prev_state = self.current_state
        self.current_state = msg.state
        
        # Detect transition into JUNCTION_DETECTED
        if self.current_state == ControllerState.JUNCTION_DETECTED and prev_state != ControllerState.JUNCTION_DETECTED:
            self.handle_junction()
            self.has_made_decision = True
            
        # Reset decision flag and clear sensor tracking when merging back
        if self.current_state == ControllerState.RESUME_TRACKING and prev_state != ControllerState.RESUME_TRACKING:
            self.has_made_decision = False
            self.call_service(self.cli_fork_clear, 'clear_follow')
            self.set_controller_track(0)
            
    def cmd_vel_callback(self, msg: Twist):
        self.nav_angular_z = msg.angular.z
        
    def track_pos_callback(self, msg: Float32):
        if self.current_state == ControllerState.FOLLOW_LINE:
            # Continuously update the last known straight position before a fork
            self.last_known_straight_pos = msg.data
            
    def left_track_callback(self, msg: Float32):
        self.left_track_pos = msg.data
        
    def right_track_callback(self, msg: Float32):
        self.right_track_pos = msg.data
        
    def left_marker_callback(self, msg: Bool):
        self.left_marker = msg.data

    def right_marker_callback(self, msg: Bool):
        self.right_marker = msg.data

    def process_track_logic(self):
        # We only apply marker-based preemptive tracking if we are not actively in a detected junction state yet
        if self.current_state == ControllerState.JUNCTION_DETECTED:
            return

        divergence = abs(self.left_track_pos - self.right_track_pos)
        
        if divergence < self.divergence_limit:
            if self.right_marker:
                self.update_tracking(2, self.cli_fork_right, 'follow_right')
            elif self.left_marker:
                self.update_tracking(1, self.cli_fork_left, 'follow_left')
            else:
                self.update_tracking(0, self.cli_fork_clear, 'clear_follow')
                
    def update_tracking(self, track_id, client, srv_name):
        if self.current_track_selection != track_id:
            self.current_track_selection = track_id
            self.set_controller_track(track_id)
            self.call_service(client, srv_name)

    def set_controller_track(self, track_id):
        if not self.cli_select_track.wait_for_service(timeout_sec=0.1):
            return
        req = SelectTrack.Request()
        req.track_id = track_id
        self.cli_select_track.call_async(req)

    def handle_junction(self):
        self.get_logger().info('Junction Detected! Making tracking decision...')
        
        # 1. Nav server wants to turn LEFT
        if self.nav_angular_z > 0.1:
            self.get_logger().info('Nav server commanded LEFT.')
            self.update_tracking(1, self.cli_fork_left, 'follow_left')
            
        # 2. Nav server wants to turn RIGHT
        elif self.nav_angular_z < -0.1:
            self.get_logger().info('Nav server commanded RIGHT.')
            self.update_tracking(2, self.cli_fork_right, 'follow_right')
            
        # 3. Nav server wants to go STRAIGHT (or ambiguous 0.0)
        else:
            self.get_logger().info(f'Nav server commanded STRAIGHT (angular.z = {self.nav_angular_z:.2f}).')
            self.get_logger().info(f'Last known straight pos: {self.last_known_straight_pos:.3f}')
            self.get_logger().info(f'Left Track: {self.left_track_pos:.3f} | Right Track: {self.right_track_pos:.3f}')
            
            # Compare which diverging track is closer to our last known straight line
            left_diff = abs(self.left_track_pos - self.last_known_straight_pos)
            right_diff = abs(self.right_track_pos - self.last_known_straight_pos)
            
            if left_diff < right_diff:
                self.get_logger().info(f'Left track ({left_diff:.3f}) is closer to straight than Right ({right_diff:.3f}). Selecting LEFT.')
                self.update_tracking(1, self.cli_fork_left, 'follow_left')
            else:
                self.get_logger().info(f'Right track ({right_diff:.3f}) is closer to straight than Left ({left_diff:.3f}). Selecting RIGHT.')
                self.update_tracking(2, self.cli_fork_right, 'follow_right')

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
