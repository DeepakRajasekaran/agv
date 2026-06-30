#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Twist
from custom_interfaces.msg import ControllerState
from custom_interfaces.srv import SelectTrack

from std_srvs.srv import Trigger
from std_msgs.msg import Bool

import os
import yaml
from ament_index_python.packages import get_package_share_directory

class NavSimulator(Node):
    def __init__(self):
        super().__init__('nav_simulator')
        
        # Load nominal_speed from the shared params.yaml
        default_speed = 0.2
        default_track_detect_stable_ms = 1000
        try:
            params_path = os.path.join(get_package_share_directory('path_follower'), 'config', 'params.yaml')
            with open(params_path, 'r') as f:
                config = yaml.safe_load(f)
                ros_params = config['/path_follower_node']['ros__parameters']
                default_speed = ros_params['robot']['nominal_speed']
                default_track_detect_stable_ms = ros_params['safety'].get(
                    'track_detect_stable_ms', default_track_detect_stable_ms)
                self.get_logger().info(f"Loaded nominal_speed from params.yaml: {default_speed}")
        except Exception as e:
            self.get_logger().warn(f"Could not load nominal_speed from params.yaml, defaulting to {default_speed}: {e}")
            
        self.declare_parameter('nominal_speed', float(default_speed))
        self.declare_parameter('track_detect_stable_ms', int(default_track_detect_stable_ms))
        self.declare_parameter('test_sequence', [0, 2, 0, 1])  # 0=AVG, 1=LEFT, 2=RIGHT
        self.declare_parameter('sequence_looping', True)
        
        self.nominal_speed = self.get_parameter('nominal_speed').value
        self.track_detect_stable_ms = int(self.get_parameter('track_detect_stable_ms').value)
        self.test_sequence = self.get_parameter('test_sequence').value
        self.sequence_looping = self.get_parameter('sequence_looping').value
        
        self.add_on_set_parameters_callback(self.parameters_callback)
        
        self.current_state = ControllerState.IDLE
        self.event_index = 0
        self.pub_cmd_vel = self.create_publisher(Twist, '/nav/cmd_vel', 10)
        self.pub_track_detect = self.create_publisher(Bool, '/sensor/track_detect', 10)
        
        self.declare_parameter('force_track_detect', False)
        
        self.sub_state = self.create_subscription(
            ControllerState, '/controller_state', self.state_callback, 10)
            
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('Nav Simulator started. First command defaults to STRAIGHT.')
        
        self.start_client = self.create_client(Trigger, '/path_follower_node/start')
        self.select_track_client = self.create_client(SelectTrack, '/path_follower_node/select_track')
        self.start_called = False
        self.start_request_pending = False
        self.error_start_time = None
        
        self.track_detect = False
        self.track_detect_true_since = None
        self.sub_track_detect = self.create_subscription(
            Bool, '/sensor/track_detect', self.track_detect_callback, 10)
        
    def parameters_callback(self, params):
        for param in params:
            if param.name == 'nominal_speed':
                self.nominal_speed = param.value
                self.get_logger().info(f"Updated nominal_speed to {self.nominal_speed}")
            elif param.name == 'track_detect_stable_ms':
                if int(param.value) < 0:
                    return SetParametersResult(
                        successful=False,
                        reason='track_detect_stable_ms must be >= 0')
                self.track_detect_stable_ms = int(param.value)
                self.get_logger().info(
                    f"Updated track_detect_stable_ms to {self.track_detect_stable_ms}")
            elif param.name == 'test_sequence':
                self.test_sequence = param.value
                self.get_logger().info(f"Updated test_sequence to {self.test_sequence}")
            elif param.name == 'sequence_looping':
                self.sequence_looping = param.value
                self.get_logger().info(f"Updated sequence_looping to {self.sequence_looping}")
        return SetParametersResult(successful=True)
        
    def update_track_detect(self, detected: bool):
        if detected:
            if not self.track_detect or self.track_detect_true_since is None:
                self.track_detect_true_since = self.get_clock().now()
        else:
            self.track_detect_true_since = None
        self.track_detect = detected

    def is_track_detect_stable(self) -> bool:
        if not self.track_detect or self.track_detect_true_since is None:
            return False

        elapsed_ms = (
            self.get_clock().now() - self.track_detect_true_since
        ).nanoseconds / 1e6
        return elapsed_ms >= self.track_detect_stable_ms

    def request_start(self, reason: str):
        if self.start_request_pending:
            return

        if self.start_client.wait_for_service(timeout_sec=0.1):
            self.get_logger().info(f'Calling /path_follower_node/start... ({reason})')
            req = Trigger.Request()
            future = self.start_client.call_async(req)
            self.start_request_pending = True
            future.add_done_callback(self.start_response_callback)

    def start_response_callback(self, future):
        self.start_request_pending = False
        try:
            response = future.result()
            if response.success:
                self.start_called = True
                self.get_logger().info(f'Start accepted: {response.message}')
            else:
                self.start_called = False
                self.get_logger().warn(f'Start rejected: {response.message}')
        except Exception as e:
            self.start_called = False
            self.get_logger().error(f'Start service call failed: {str(e)}')

    def track_detect_callback(self, msg: Bool):
        self.update_track_detect(msg.data)
        
    def state_callback(self, msg: ControllerState):
        prev_state = self.current_state
        self.current_state = msg.state
        
        # Detect transition into JUNCTION_DETECTED
        if prev_state != ControllerState.JUNCTION_DETECTED and self.current_state == ControllerState.JUNCTION_DETECTED:
            if not self.test_sequence:
                self.get_logger().warn('Junction detected but test_sequence is empty!')
                return
                
            if self.event_index >= len(self.test_sequence):
                if self.sequence_looping:
                    self.event_index = 0
                    self.get_logger().info('Sequence looping back to start.')
                else:
                    self.get_logger().info('Test sequence completed. No further actions.')
                    return
                    
            track_id = self.test_sequence[self.event_index]
            self.event_index += 1
            
            req = SelectTrack.Request()
            self.get_logger().info(f'Junction Event {self.event_index}/{len(self.test_sequence)} -> Calling service to track ID: {track_id}')
            req.track_id = track_id
                
            if self.select_track_client.wait_for_service(timeout_sec=0.5):
                self.select_track_client.call_async(req)
            else:
                self.get_logger().error('Service /path_follower_node/select_track not available!')
                
        # Logging standard track resumption
        if prev_state == ControllerState.JUNCTION_DETECTED and self.current_state == ControllerState.FOLLOW_LINE:
            self.get_logger().info('Resumed standard tracking (Controller auto-reset track to AVERAGE).')

    def timer_callback(self):
        # Force track detect if enabled
        if self.get_parameter('force_track_detect').value:
            msg = Bool()
            msg.data = True
            self.pub_track_detect.publish(msg)
            self.update_track_detect(True)

        track_detect_stable = self.is_track_detect_stable()

        # Auto-start if IDLE
        if self.current_state == ControllerState.IDLE and not self.start_called and track_detect_stable:
            self.request_start('stable track detected')
        
        # Auto-recovery if ERROR
        if self.current_state == ControllerState.ERROR:
            if self.error_start_time is None:
                self.error_start_time = self.get_clock().now()
            else:
                elapsed = (self.get_clock().now() - self.error_start_time).nanoseconds / 1e9
                if elapsed > 2.0 and track_detect_stable:
                    self.get_logger().warn('Auto-recovering from ERROR state (tape detected)...')
                    self.request_start('stable track recovery')
                    self.error_start_time = None
        else:
            self.error_start_time = None
        
        msg = Twist()
        
        # If tracking is active, provide velocity
        if self.current_state in [ControllerState.FOLLOW_LINE, ControllerState.JUNCTION_DETECTED, ControllerState.RESUME_TRACKING]:
            msg.linear.x = self.nominal_speed
            msg.angular.z = 0.0
            self.get_logger().info(f'Tick | State: {self.current_state} | cmd_vel: v={msg.linear.x:.2f}, w={msg.angular.z:.2f}')
            self.pub_cmd_vel.publish(msg)
        else:
            # Do not publish when in IDLE or ERROR, so we don't block manual teleop recovery
            pass

def main(args=None):
    rclpy.init(args=args)
    node = NavSimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
