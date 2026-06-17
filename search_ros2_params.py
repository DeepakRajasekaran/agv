#!/usr/bin/env python3
import subprocess
import time
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

class TuningMonitor(Node):
    def __init__(self):
        super().__init__('tuning_monitor')
        self.max_error = 0.0
        self.track_lost = False
        self.goal_reached = False
        self.error_count = 0
        
        self.sub_err = self.create_subscription(Float32, '/sensor/track_position', self.err_cb, 10)
        self.sub_tag = self.create_subscription(String, '/sensor/tag_id', self.tag_cb, 10)
        
    def err_cb(self, msg):
        err = abs(msg.data)
        if err > self.max_error:
            self.max_error = err
        if err >= 0.25:
            self.error_count += 1
            if self.error_count > 100:
                self.track_lost = True
        else:
            self.error_count = 0

    def tag_cb(self, msg):
        if msg.data == "TAG_BOT":
            self.goal_reached = True

def run_test(kp, kd):
    # Always clean stale processes first
    subprocess.run(["pkill", "-f", "ros2"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-f", "hardware_sim_node"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-f", "line_follower_controller_node"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-f", "nav_server_node"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1.0)

    # Start play.launch.py in background
    launch_log = open("/home/lucifer/anscer_workspace/LineFollower/launch_output.log", "w")
    launch_proc = subprocess.Popen(
        ["ros2", "launch", "line_follower_controller", "play.launch.py", "headless:=true"],
        stdout=launch_log,
        stderr=launch_log
    )
    
    time.sleep(4.0)
    
    rclpy.init()
    node = TuningMonitor()
    
    # Set parameters
    try:
        subprocess.run(["ros2", "param", "set", "/line_follower_controller_node", "pid.kp", str(kp)], check=True, stdout=subprocess.DEVNULL)
        subprocess.run(["ros2", "param", "set", "/line_follower_controller_node", "pid.kd", str(kd)], check=True, stdout=subprocess.DEVNULL)
    except Exception as e:
        node.destroy_node()
        rclpy.shutdown()
        launch_proc.terminate()
        launch_proc.wait()
        launch_log.close()
        return False, 999.0
        
    action_client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
    if not action_client.wait_for_server(timeout_sec=5.0):
        node.destroy_node()
        rclpy.shutdown()
        launch_proc.terminate()
        launch_proc.wait()
        launch_log.close()
        return False, 999.0
        
    goal_msg = NavigateToPose.Goal()
    goal_msg.pose.header.frame_id = 'map'
    goal_msg.pose.pose.position.x = 2.0
    goal_msg.pose.pose.position.y = -2.0
    
    send_goal_future = action_client.send_goal_async(goal_msg)
    
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        if send_goal_future.done():
            goal_handle = send_goal_future.result()
            if not goal_handle.accepted:
                node.destroy_node()
                rclpy.shutdown()
                launch_proc.terminate()
                launch_proc.wait()
                launch_log.close()
                return False, 999.0
            break
            
    start_time = time.time()
    success = False
    
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)
        
        if node.track_lost:
            break
            
        if node.goal_reached:
            success = True
            break
            
        # Short timeout of 25 seconds is enough if it is moving stably
        if time.time() - start_time > 25.0:
            break
            
    max_err = node.max_error
    
    node.destroy_node()
    rclpy.shutdown()
    
    launch_proc.terminate()
    launch_proc.wait()
    launch_log.close()
    
    return success, max_err

if __name__ == "__main__":
    candidates = [
        (3.0, 1.0), (3.0, 1.5), (3.0, 2.0),
        (4.0, 1.0), (4.0, 1.5), (4.0, 2.0),
        (5.0, 1.0), (5.0, 1.5), (5.0, 2.0)
    ]
            
    results = []
    for kp, kd in candidates:
        ok, err = run_test(kp, kd)
        status = "SUCCESS" if ok else "FAILED"
        print(f"Kp={kp:.1f}, Kd={kd:.1f} -> {status} (Max Error = {err:.4f} m)", flush=True)
        results.append((ok, err, kp, kd))
        
    print("\nSorted by success and min error:")
    results.sort(key=lambda x: (not x[0], x[1]))
    for ok, err, kp, kd in results:
        status = "SUCCESS" if ok else "FAILED"
        print(f"Kp={kp:.1f}, Kd={kd:.1f} -> {status} (Max Error = {err:.4f} m)")
