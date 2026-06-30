#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, UInt16
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool, Trigger
import time
import sys

class FeatureTester(Node):
    def __init__(self):
        super().__init__('feature_tester')
        
        # Nominal speed to publish
        self.nominal_speed = 1.0
        
        # Publishers
        self.pub_cmd_vel = self.create_publisher(Twist, '/nav/cmd_vel', 10)
        self.pub_track_pos = self.create_publisher(Float32, '/sensor/track_position', 10)
        self.pub_track_detect = self.create_publisher(Bool, '/sensor/track_detect', 10)
        self.pub_left_pos = self.create_publisher(Float32, '/sensor/left_track_position', 10)
        self.pub_right_pos = self.create_publisher(Float32, '/sensor/right_track_position', 10)
        self.pub_tape_cross = self.create_publisher(Bool, '/sensor/tape_cross', 10)
        
        self.pub_left_marker = self.create_publisher(Bool, '/sensor/left_marker', 10)
        self.pub_right_marker = self.create_publisher(Bool, '/sensor/right_marker', 10)
        self.pub_protective = self.create_publisher(Bool, '/plc_interface/lidar_protective_breach_fb', 10)
        self.pub_warning = self.create_publisher(Bool, '/plc_interface/lidar_warning_breach_fb', 10)
        
        # Subscribers to verify outputs
        self.sub_follower_cmd = self.create_subscription(Twist, '/path_follower/cmd_vel', self.cmd_callback, 10)
        self.sub_lidar_cmd = self.create_subscription(UInt16, '/plc_interface/lidar_cmd', self.lidar_cmd_callback, 10)
        
        # Service mock
        self.srv_quickstop = self.create_service(SetBool, '/plc_interface/trigger_quickstop', self.quickstop_callback)
        
        # Clients
        self.cli_start = self.create_client(Trigger, '/path_follower_node/start')
        
        # Recorded state
        self.last_follower_cmd = None
        self.last_lidar_cmd = None
        self.quickstop_calls = []
        
    def cmd_callback(self, msg):
        self.last_follower_cmd = msg
        
    def lidar_cmd_callback(self, msg):
        self.last_lidar_cmd = msg.data
        
    def quickstop_callback(self, request, response):
        self.quickstop_calls.append(request.data)
        response.success = True
        response.message = f"Quickstop state set to {request.data}"
        return response

    def run_tests(self):
        # 1. Start the controller node (send start trigger)
        self.get_logger().info("Waiting for path_follower_node/start service...")
        while not self.cli_start.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting...")
        
        # Publish nominal states to stabilize track detection
        self.get_logger().info("Publishing nominal track detect and cmd_vel...")
        self.spin_for(3.0)
            
        self.get_logger().info("Triggering start...")
        req = Trigger.Request()
        future = self.cli_start.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        self.get_logger().info(f"Start Response: {future.result().message}")

        # Let the node process the active state
        self.spin_for(1.0)

        # Test Case 1: Lidar Warning Breach
        self.get_logger().info("--- TEST 1: Lidar Warning Breach (50% reduction) ---")
        self.spin_for(1.5, warning=True)
        
        assert self.last_follower_cmd is not None
        self.get_logger().info(f"Target linear velocity: {self.nominal_speed} m/s, Output: {self.last_follower_cmd.linear.x:.3f} m/s")
        assert abs(self.last_follower_cmd.linear.x - 0.5) < 0.1, f"Expected ~0.5 m/s, got {self.last_follower_cmd.linear.x}"
        self.get_logger().info("TEST 1 PASSED!")

        # Clear warning
        self.spin_for(1.0)
        self.get_logger().info(f"Output velocity after clearing warning: {self.last_follower_cmd.linear.x:.3f} m/s")
        assert abs(self.last_follower_cmd.linear.x - 1.0) < 0.1, "Should recover to 1.0 m/s"

        # Test Case 2: Lidar Protective Breach (Quickstop & Stop)
        self.get_logger().info("--- TEST 2: Lidar Protective Breach ---")
        self.quickstop_calls.clear()
        self.spin_for(0.5, protective=True)
        
        self.get_logger().info(f"Output velocity: {self.last_follower_cmd.linear.x:.3f} m/s")
        assert self.last_follower_cmd.linear.x == 0.0, "Velocity must be 0.0"
        assert True in self.quickstop_calls, "Quickstop trigger_quickstop(true) must be called"
        self.get_logger().info("TEST 2 (Protective stop) PASSED!")

        # Clear protective breach and verify recovery and ramping
        self.get_logger().info("--- TEST 2b: Recovery from Protective Breach ---")
        self.quickstop_calls.clear()
        
        # Check that it doesn't jump immediately to 1.0 (acceleration limit)
        self.spin_for(0.1)
        self.get_logger().info(f"Speed immediately after recovery: {self.last_follower_cmd.linear.x:.3f} m/s")
        assert self.last_follower_cmd.linear.x < 0.8, "Should not jump immediately to 1.0 (ramping check)"
        
        self.spin_for(2.5)
        self.get_logger().info(f"Speed after 2.5 seconds: {self.last_follower_cmd.linear.x:.3f} m/s")
        assert abs(self.last_follower_cmd.linear.x - 1.0) < 0.1, f"Should recover to 1.0 m/s, got {self.last_follower_cmd.linear.x}"
        assert False in self.quickstop_calls, "Quickstop trigger_quickstop(false) must be called"
        self.get_logger().info("TEST 2b PASSED!")

        # Test Case 3: Junction Detection
        self.get_logger().info("--- TEST 3: Double-Marker Junction ---")
        self.spin_for(1.5, left_marker=True, right_marker=True)
        self.get_logger().info(f"Junction Entry Speed: {self.last_follower_cmd.linear.x:.3f} m/s")
        assert abs(self.last_follower_cmd.linear.x - 0.3) < 0.1, f"Should clamp to junction speed (0.3), got {self.last_follower_cmd.linear.x}"

        # Transition state (markers clear)
        self.spin_for(0.5)
        self.get_logger().info(f"Junction Transition Speed: {self.last_follower_cmd.linear.x:.3f} m/s")
        assert abs(self.last_follower_cmd.linear.x - 0.3) < 0.1, "Should remain clamped to junction speed in transition"

        # Exit marker
        self.spin_for(0.5, left_marker=True, right_marker=True)
        # Clear exit marker
        self.spin_for(2.5)
        self.get_logger().info(f"Junction Exit Speed: {self.last_follower_cmd.linear.x:.3f} m/s")
        assert abs(self.last_follower_cmd.linear.x - 1.0) < 0.1, "Should recover to nominal speed after exiting junction"
        self.get_logger().info("TEST 3 PASSED!")

        # Test Case 4: Turn Detection
        self.get_logger().info("--- TEST 4: Single-Marker Turn ---")
        self.spin_for(1.5, left_marker=True, right_marker=False)
        self.get_logger().info(f"Turn Entry Speed: {self.last_follower_cmd.linear.x:.3f} m/s")
        assert abs(self.last_follower_cmd.linear.x - 0.4) < 0.1, f"Should clamp to turn speed (0.4), got {self.last_follower_cmd.linear.x}"

        # Transition state (markers clear)
        self.spin_for(0.5)
        self.get_logger().info(f"Turn Transition Speed: {self.last_follower_cmd.linear.x:.3f} m/s")
        assert abs(self.last_follower_cmd.linear.x - 0.4) < 0.1, "Should remain clamped to turn speed in transition"

        # Exit turn marker (any single marker)
        self.spin_for(0.5, left_marker=True, right_marker=False)
        self.spin_for(2.5)
        self.get_logger().info(f"Turn Exit Speed: {self.last_follower_cmd.linear.x:.3f} m/s")
        assert abs(self.last_follower_cmd.linear.x - 1.0) < 0.1, "Should recover to nominal speed after exiting turn"
        self.get_logger().info("TEST 4 PASSED!")

        # Test Case 5: Safety field switching thresholds
        self.get_logger().info("--- TEST 5: Lidar Command Switching ---")
        # 1.0 m/s -> should be command 3 (> 0.7)
        self.get_logger().info(f"Lidar command at 1.0 m/s: {self.last_lidar_cmd}")
        assert self.last_lidar_cmd == 3, f"Expected lidar command 3, got {self.last_lidar_cmd}"

        # Reduce speed to 0.5 m/s (between 0.3 and 0.7) -> command 2
        self.nominal_speed = 0.5
        self.spin_for(2.5)
        self.get_logger().info(f"Lidar command at 0.5 m/s: {self.last_lidar_cmd}")
        assert self.last_lidar_cmd == 2, f"Expected lidar command 2, got {self.last_lidar_cmd}"

        # Reduce speed to 0.2 m/s (< 0.3) -> command 1
        self.nominal_speed = 0.2
        self.spin_for(2.5)
        self.get_logger().info(f"Lidar command at 0.2 m/s: {self.last_lidar_cmd}")
        assert self.last_lidar_cmd == 1, f"Expected lidar command 1, got {self.last_lidar_cmd}"
        self.get_logger().info("TEST 5 PASSED!")

        self.get_logger().info("ALL TESTS COMPLETED SUCCESSFULLY!")

    def make_twist(self, linear, angular):
        t = Twist()
        t.linear.x = float(linear)
        t.angular.z = float(angular)
        return t

    def publish_state(self, left_marker=False, right_marker=False, protective=False, warning=False):
        self.pub_cmd_vel.publish(self.make_twist(self.nominal_speed, 0.0))
        f = Float32()
        f.data = 0.0
        self.pub_track_pos.publish(f)
        self.pub_left_pos.publish(f)
        self.pub_right_pos.publish(f)
        b = Bool()
        b.data = True
        self.pub_track_detect.publish(b)
        b.data = False
        self.pub_tape_cross.publish(b)
        
        b.data = left_marker
        self.pub_left_marker.publish(b)
        b.data = right_marker
        self.pub_right_marker.publish(b)
        b.data = protective
        self.pub_protective.publish(b)
        b.data = warning
        self.pub_warning.publish(b)

    def spin_for(self, duration_sec, left_marker=False, right_marker=False, protective=False, warning=False):
        start = time.time()
        last_pub = 0.0
        while time.time() - start < duration_sec:
            now = time.time()
            if now - last_pub >= 0.02: # 50Hz
                self.publish_state(left_marker, right_marker, protective, warning)
                last_pub = now
            rclpy.spin_once(self, timeout_sec=0.005)

def main():
    rclpy.init()
    tester = FeatureTester()
    try:
        tester.run_tests()
    except AssertionError as e:
        tester.get_logger().error(f"TEST FAILURE: {str(e)}")
        sys.exit(1)
    except Exception as e:
        tester.get_logger().error(f"UNEXPECTED ERROR: {str(e)}")
        sys.exit(1)
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
