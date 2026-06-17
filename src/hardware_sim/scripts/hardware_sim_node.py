#!/usr/bin/env python3
# Name:        hardware_sim_node.py
# Author:      Deepak Rajasekaran
# Date:        2026-06-12
# Version:     1.0
# Description: MuJoCo-based hardware simulator node ported to ROS 2.

import os
import math
import time
import numpy as np
import mujoco
import mujoco.viewer
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Float32, Bool, String
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Path
import sys

# Add the script's directory to sys.path to find relative imports
sys.path.append(os.path.dirname(os.path.realpath(__file__)))
from TapeMap import TapeMap

class HardwareSimNode(Node):
    def __init__(self):
        super().__init__('hardware_sim_node')
        
        # Initialize TapeMap
        self.tape_map = TapeMap()
        
        # Load MuJoCo Model
        pkg_share = get_package_share_directory('hardware_sim')
        model_path = os.path.join(pkg_share, 'models', 'scene.xml')
        self.get_logger().info(f"Loading MuJoCo model from: {model_path}")
        
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        
        # Set initial robot position
        # Start slightly offset to verify PID convergence: x=-2.5, y=2.02, yaw=0
        self.data.qpos[0] = -2.5  # X
        self.data.qpos[1] = 2.02 # Y (offset)
        self.data.qpos[2] = 0.08 # Z (touching ground)
        self.data.qpos[3] = 1.0  # qw
        self.data.qpos[4] = 0.0  # qx
        self.data.qpos[5] = 0.0  # qy
        self.data.qpos[6] = 0.0  # qz
        
        # Robot physical parameters
        self.wheel_base = 0.512  # L
        self.wheel_radius = 0.08 # R
        
        # ROS 2 Publishers
        self.pub_track_pos = self.create_publisher(Float32, '/sensor/track_position', 10)
        self.pub_track_detect = self.create_publisher(Bool, '/sensor/track_detect', 10)
        self.pub_left_marker = self.create_publisher(Bool, '/sensor/left_marker', 10)
        self.pub_right_marker = self.create_publisher(Bool, '/sensor/right_marker', 10)
        self.pub_left_track_pos = self.create_publisher(Float32, '/sensor/left_track_position', 10)
        self.pub_right_track_pos = self.create_publisher(Float32, '/sensor/right_track_position', 10)
        self.pub_tag_id = self.create_publisher(String, '/sensor/tag_id', 10)
        self.pub_tag_cmd = self.create_publisher(String, '/sensor/tag_command', 10)
        self.pub_pose = self.create_publisher(Pose2D, '/robot_pose', 10)
        
        # ROS 2 Subscribers
        self.sub_cmd_vel = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.sub_select_track = self.create_subscription(String, '/sensor/select_track', self.select_track_callback, 10)
        self.sub_fault = self.create_subscription(String, '/sim/inject_fault', self.fault_callback, 10)
        self.sub_plan = self.create_subscription(Path, '/plan', self.plan_callback, 10)
        
        # Fault injection state
        self.sensor_dropout_active = False
        self.frozen_track_pos = 0.0
        self.current_plan = None
        
        # Tag detection debounce state
        self.active_tag_id = None
        
        # Declare headless parameter (to disable passive GUI viewer)
        self.declare_parameter('headless', False)
        self.headless = self.get_parameter('headless').get_parameter_value().bool_value
        
        # Start passive viewer if not headless
        if not self.headless:
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
        else:
            self.viewer = None
        
        self.step_count = 0
        self.get_logger().info(f"Hardware Simulator Node Initialized. Headless mode: {self.headless}")

    def cmd_vel_callback(self, msg: Twist):
        # Differential drive inverse kinematics
        v = msg.linear.x
        omega = msg.angular.z
        
        # Compute target wheel angular velocities and pass directly to velocity-servo actuators.
        # The actuator type is 'velocity' in scene.xml, so ctrl = target omega (rad/s).
        # Negation is due to MuJoCo's coordinate frame: positive qvel = reverse for these joints.
        v_l = (v - omega * self.wheel_base / 2.0) / self.wheel_radius
        v_r = (v + omega * self.wheel_base / 2.0) / self.wheel_radius
        
        self.data.actuator('left_wheel_motor').ctrl[0] = -v_l
        self.data.actuator('right_wheel_motor').ctrl[0] = -v_r
        
        self.get_logger().debug(f"[CMD_VEL] v={v:.3f}, omega={omega:.3f} | ctrl_l={-v_l:.3f}, ctrl_r={-v_r:.3f}")

    def plan_callback(self, msg: Path):
        self.current_plan = msg
        self.get_logger().info(f"Simulator received new plan with {len(msg.poses)} waypoints.")

    def select_track_callback(self, msg: String):
        self.get_logger().info(f"Selecting track branch: {msg.data}")
        self.tape_map.select_branch(msg.data)

    def fault_callback(self, msg: String):
        command = msg.data.lower()
        if command == "sensor_dropout":
            self.sensor_dropout_active = True
            self.get_logger().warn("INJECTING FAULT: Sensor dropout!")
        elif command == "clear":
            self.sensor_dropout_active = False
            self.get_logger().info("FAULT CLEARED: Sensor normal.")

    def draw_tape_path(self):
        if not self.viewer.is_running():
            return
            
        # Reset user geoms count
        self.viewer.user_scn.ngeom = 0
        
        # Color definitions (RGBA)
        color_track = np.array([0.15, 0.15, 0.15, 1.0])  # Dark Charcoal for all tracks
        
        # Helper to draw a list of waypoints
        def draw_segment_list(waypoints):
            half_width = 0.035 # 70mm line thickness -> 35mm half-width
            half_height = 0.0001 # 0.2mm total thickness
            
            for i in range(len(waypoints) - 1):
                p1 = np.array(waypoints[i])
                p2 = np.array(waypoints[i+1])
                
                center = (p1 + p2) / 2.0
                pos = np.array([center[0], center[1], 0.0005]) # 0.5mm above floor
                
                v = p2 - p1
                length = np.linalg.norm(v)
                if length < 1e-6:
                    continue
                    
                # Calculate yaw rotation
                yaw = np.arctan2(v[1], v[0])
                c = np.cos(yaw)
                s = np.sin(yaw)
                # 3x3 rotation matrix around Z axis
                mat = np.array([
                    [c, -s, 0.0],
                    [s,  c, 0.0],
                    [0.0, 0.0, 1.0]
                ]).flatten()
                
                if self.viewer.user_scn.ngeom < self.viewer.user_scn.maxgeom:
                    idx = self.viewer.user_scn.ngeom
                    mujoco.mjv_initGeom(
                        self.viewer.user_scn.geoms[idx],
                        type=mujoco.mjtGeom.mjGEOM_BOX,
                        size=[length / 2.0, half_width, half_height],
                        pos=pos,
                        mat=mat,
                        rgba=color_track
                    )
                    self.viewer.user_scn.ngeom += 1

        # Draw paths (all in dark charcoal, no highlights)
        draw_segment_list(self.tape_map.path_outer)
        draw_segment_list(self.tape_map.path_mid_down)
        draw_segment_list(self.tape_map.path_mid_up)
        
        # Draw the planned path overlay if available (thin, flat cyan line)
        if self.current_plan and len(self.current_plan.poses) > 0:
            plan_pts = []
            for pose in self.current_plan.poses:
                plan_pts.append((pose.pose.position.x, pose.pose.position.y))
            # Draw this as a thin cyan line overlay (thickness 16mm -> half_width 8mm)
            half_width = 0.008
            half_height = 0.00015
            color_plan = np.array([0.0, 0.9, 0.9, 0.8]) # Cyan overlay
            for i in range(len(plan_pts) - 1):
                p1 = np.array(plan_pts[i])
                p2 = np.array(plan_pts[i+1])
                center = (p1 + p2) / 2.0
                pos = np.array([center[0], center[1], 0.001]) # 1.0mm above floor
                
                v = p2 - p1
                length = np.linalg.norm(v)
                if length < 1e-6:
                    continue
                yaw = np.arctan2(v[1], v[0])
                c = np.cos(yaw)
                s = np.sin(yaw)
                mat = np.array([
                    [c, -s, 0.0],
                    [s,  c, 0.0],
                    [0.0, 0.0, 1.0]
                ]).flatten()
                
                if self.viewer.user_scn.ngeom < self.viewer.user_scn.maxgeom:
                    idx = self.viewer.user_scn.ngeom
                    mujoco.mjv_initGeom(
                        self.viewer.user_scn.geoms[idx],
                        type=mujoco.mjtGeom.mjGEOM_BOX,
                        size=[length / 2.0, half_width, half_height],
                        pos=pos,
                        mat=mat,
                        rgba=color_plan
                    )
                    self.viewer.user_scn.ngeom += 1

        # Determine the destination tag from current plan to highlight
        dest_tag_id = None
        if self.current_plan and len(self.current_plan.poses) > 0:
            dest_pose = self.current_plan.poses[-1].pose.position
            # Find the closest tag to dest_pose
            min_d = 0.5
            for tag in self.tape_map.tags:
                d = math.hypot(tag["pos"][0] - dest_pose.x, tag["pos"][1] - dest_pose.y)
                if d < min_d:
                    min_d = d
                    dest_tag_id = tag["tag_id"]

        # Draw RFID tags as small cylinders (highlight destination tag in bright gold, others in dark green)
        for tag in self.tape_map.tags:
            t_pos = np.array([tag["pos"][0], tag["pos"][1], 0.002])
            is_dest = (tag["tag_id"] == dest_tag_id)
            rgba = [1.0, 0.8, 0.0, 1.0] if is_dest else [0.0, 0.4, 0.0, 0.8]
            if self.viewer.user_scn.ngeom < self.viewer.user_scn.maxgeom:
                idx = self.viewer.user_scn.ngeom
                mujoco.mjv_initGeom(
                    self.viewer.user_scn.geoms[idx],
                    type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                    size=[0.05, 0.001, 0.0], # radius 5cm, height 1mm
                    pos=t_pos,
                    mat=np.eye(3).flatten(),
                    rgba=rgba
                )
                self.viewer.user_scn.ngeom += 1

        # Draw Junction Markers as orange cylinders
        for marker in self.tape_map.markers:
            m_pos = np.array([marker["pos"][0], marker["pos"][1], 0.002])
            if self.viewer.user_scn.ngeom < self.viewer.user_scn.maxgeom:
                idx = self.viewer.user_scn.ngeom
                mujoco.mjv_initGeom(
                    self.viewer.user_scn.geoms[idx],
                    type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                    size=[0.02, 0.001, 0.0], # radius 2cm, height 1mm
                    pos=m_pos,
                    mat=np.eye(3).flatten(),
                    rgba=[1.0, 0.5, 0.0, 1.0]
                )
                self.viewer.user_scn.ngeom += 1

    def spin(self):
        while rclpy.ok():
            # Process incoming subscriptions/callbacks
            rclpy.spin_once(self, timeout_sec=0.0)
            
            # Step MuJoCo physics TWICE per 10ms control cycle (2 x 5ms = 10ms).
            # scene.xml uses timestep=0.005s, so two steps = one 10ms wall-clock tick.
            # The velocity-servo actuator updates control force at each physics step.
            mujoco.mj_step(self.model, self.data)
            mujoco.mj_step(self.model, self.data)
            self.step_count += 1
            
            # Sync viewer visualization at 20Hz (every 5 steps of 10ms)
            if not self.headless and self.viewer and self.viewer.is_running():
                if self.step_count % 5 == 0:
                    self.draw_tape_path()
                    self.viewer.sync()
                
            # Get robot position and orientation
            rx = self.data.qpos[0]
            ry = self.data.qpos[1]
            
            # Quaternion to Yaw
            qw = self.data.qpos[3]
            qx = self.data.qpos[4]
            qy = self.data.qpos[5]
            qz = self.data.qpos[6]
            
            yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
            
            # Publish robot pose for MapStudio UI
            pose_msg = Pose2D()
            pose_msg.x = float(rx)
            pose_msg.y = float(ry)
            pose_msg.theta = float(yaw)
            self.pub_pose.publish(pose_msg)
            
            # 1. Compute lateral error
            # Track detection threshold: 0.25m matches safety.lost_threshold in params.yaml.
            # The MGS1600 physical sensor has a ±90mm active range, but the simulation
            # extends this to allow the PID to attempt recovery up to the fault threshold.
            closest_pt, dist = self.tape_map.get_closest_point((rx, ry), self.tape_map.active_path)
            TRACK_DETECT_THRESHOLD = 0.25  # meters — must match safety.lost_threshold
            track_detected = dist < TRACK_DETECT_THRESHOLD
            
            if self.step_count % 200 == 0:
                self.get_logger().info(f"[ROBOT_POSE] step: {self.step_count}, pos: ({rx:.3f}, {ry:.3f}), yaw: {yaw:.3f}, dist_to_line: {dist:.3f}, branch: {self.tape_map.active_branch}")
            
            raw_error = 0.0
            if track_detected:
                raw_error = self.tape_map.compute_lateral_error(rx, ry, yaw)
            elif self.step_count % 20 == 0:
                self.get_logger().warn(f"[TRACK_LOST] pos: ({rx:.3f}, {ry:.3f}), dist: {dist:.3f}m, branch: {self.tape_map.active_branch}")
                
            # Apply fault injection
            if self.sensor_dropout_active:
                error_val = self.frozen_track_pos
            else:
                # Add tiny Gaussian noise to prevent false positive frozen sensor faults on straight lines
                if track_detected:
                    error_val = raw_error + np.random.normal(0, 1e-5)
                else:
                    error_val = raw_error
                self.frozen_track_pos = error_val
                
            # Publish track error and presence with explicit type casting
            self.pub_track_pos.publish(Float32(data=float(error_val)))
            self.pub_track_detect.publish(Bool(data=bool(track_detected)))
            
            # 2. Simulate Markers (left/right)
            left_m = False
            right_m = False
            for marker in self.tape_map.markers:
                m_pos = np.array(marker["pos"])
                dist_to_m = np.linalg.norm(np.array([rx, ry]) - m_pos)
                if dist_to_m < self.tape_map.marker_radius:
                    if marker["is_left"]:
                        left_m = True
                    else:
                        right_m = True
                        
            self.pub_left_marker.publish(Bool(data=bool(left_m)))
            self.pub_right_marker.publish(Bool(data=bool(right_m)))
            
            # 3. Simulate Fork (Left & Right errors)
            dist_to_j = np.linalg.norm(np.array([rx, ry]) - np.array(self.tape_map.junction_node))
            # Publish side branches if junction marker is active
            if left_m or right_m:
                l_err = 0.0
                r_err = 0.0
                self.pub_left_track_pos.publish(Float32(data=float(l_err)))
                self.pub_right_track_pos.publish(Float32(data=float(r_err)))
                
            # 4. Simulate RFID Tags
            tag_detected = False
            for tag in self.tape_map.tags:
                t_pos = np.array(tag["pos"])
                dist_to_t = np.linalg.norm(np.array([rx, ry]) - t_pos)
                if dist_to_t < self.tape_map.tag_radius:
                    tag_detected = True
                    if self.active_tag_id != tag["tag_id"]:
                        self.active_tag_id = tag["tag_id"]
                        
                        # Publish tag id
                        self.pub_tag_id.publish(String(data=str(tag["tag_id"])))
                        self.get_logger().info(f"READ TAG: ID={tag['tag_id']}")
                        
            # Reset tag debounce when moving away
            if not tag_detected:
                self.active_tag_id = None
                
            try:
                time.sleep(0.01) # Sleep for 10ms (100Hz frequency)
            except Exception:
                break

        if not self.headless and self.viewer:
            self.viewer.close()

def main(args=None):
    rclpy.init(args=args)
    node = HardwareSimNode()
    try:
        node.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
