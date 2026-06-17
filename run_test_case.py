#!/usr/bin/env python3
import subprocess
import time
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

class TuningMonitor(Node):
    def __init__(self, kp, kd):
        super().__init__('tuning_monitor')
        self.kp = kp
        self.kd = kd
        self.max_error = 0.0
        self.track_lost = False
        self.goal_reached = False
        self.aborted = False
        self.last_tag = None
        self.error_count = 0
        
        self.sub_err = self.create_subscription(Float32, '/sensor/track_position', self.err_cb, 10)
        self.sub_tag = self.create_subscription(String, '/sensor/tag_id', self.tag_cb, 10)
        
    def err_cb(self, msg):
        err = abs(msg.data)
        if err > self.max_error:
            self.max_error = err
        if err >= 0.25: # lost threshold
            self.error_count += 1
            if self.error_count > 100: # 1 second at 100Hz
                self.track_lost = True
        else:
            self.error_count = 0

    def tag_cb(self, msg):
        self.last_tag = msg.data
        if msg.data == "TAG_BOT":
            self.goal_reached = True

def run_test(kp, kd):
    print(f"\n==========================================")
    print(f"Testing ROS 2 tuning: Kp={kp}, Kd={kd}")
    print(f"==========================================")
    
    # 1. Start play.launch.py in background, logging to launch_output.log
    launch_log = open("/home/lucifer/anscer_workspace/LineFollower/launch_output.log", "w")
    launch_proc = subprocess.Popen(
        ["ros2", "launch", "line_follower_controller", "play.launch.py", "headless:=true"],
        stdout=launch_log,
        stderr=launch_log
    )
    
    # Wait for nodes to start up
    time.sleep(4.0)
    
    rclpy.init()
    node = TuningMonitor(kp, kd)
    
    # 2. Set parameters
    try:
        # Set KP
        subprocess.run(["ros2", "param", "set", "/line_follower_controller_node", "pid.kp", str(kp)], check=True, stdout=subprocess.DEVNULL)
        # Set KD
        subprocess.run(["ros2", "param", "set", "/line_follower_controller_node", "pid.kd", str(kd)], check=True, stdout=subprocess.DEVNULL)
        print("Parameters set successfully.")
    except Exception as e:
        print(f"Failed to set parameters: {e}")
        node.destroy_node()
        rclpy.shutdown()
        launch_proc.terminate()
        launch_proc.wait()
        return False, 999.0
        
    # 3. Send navigation goal to TAG_BOT using ActionClient
    action_client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
    if not action_client.wait_for_server(timeout_sec=5.0):
        print("Action server not available.")
        node.destroy_node()
        rclpy.shutdown()
        launch_proc.terminate()
        launch_proc.wait()
        return False, 999.0
        
    goal_msg = NavigateToPose.Goal()
    goal_msg.pose.header.frame_id = 'map'
    goal_msg.pose.pose.position.x = 2.0
    goal_msg.pose.pose.position.y = -2.0
    
    print("Sending goal to TAG_BOT...")
    send_goal_future = action_client.send_goal_async(goal_msg)
    
    # Spin until goal is accepted
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        if send_goal_future.done():
            goal_handle = send_goal_future.result()
            if not goal_handle.accepted:
                print("Goal rejected.")
                node.destroy_node()
                rclpy.shutdown()
                launch_proc.terminate()
                launch_proc.wait()
                return False, 999.0
            break
            
    # Monitor the run
    start_time = time.time()
    success = False
    
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)
        
        # Check termination conditions
        if node.track_lost:
            print("FAILED: Robot lost the track / went out of track!")
            break
            
        if node.goal_reached:
            print("SUCCESS: Goal reached!")
            success = True
            break
            
        # Timeout after 45 seconds
        if time.time() - start_time > 45.0:
            print("FAILED: Timeout reached.")
            break
            
    max_err = node.max_error
    print(f"Max error recorded: {max_err:.4f} m")
    
    # Cleanup
    node.destroy_node()
    rclpy.shutdown()
    
    launch_proc.terminate()
    launch_proc.wait()
    launch_log.close()
    
    return success, max_err

if __name__ == "__main__":
    # Always clean any stale processes first
    subprocess.run(["pkill", "-f", "ros2"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-f", "hardware_sim_node"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-f", "line_follower_controller_node"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-f", "nav_server_node"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1.0)

    if len(sys.argv) == 3:
        run_test(float(sys.argv[1]), float(sys.argv[2]))
    else:
        # Run a small grid search of candidate tunings
        candidates = [
            (10.0, 0.5), # original
            (10.0, 0.8),
            (10.0, 1.2),
            (8.0, 0.6),
            (8.0, 0.8),
            (8.0, 1.0),
            (6.0, 0.6),
            (6.0, 0.8),
            (5.0, 0.8),
            (5.0, 1.0)
        ]
        results = []
        for kp, kd in candidates:
            # First, clean any stale processes
            subprocess.run(["pkill", "-f", "ros2"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            subprocess.run(["pkill", "-f", "hardware_sim_node"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            subprocess.run(["pkill", "-f", "line_follower_controller_node"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            subprocess.run(["pkill", "-f", "nav_server_node"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            time.sleep(1.0)
            
            ok, err = run_test(kp, kd)
            results.append((ok, err, kp, kd))
            
        print("\n\n==========================================")
        print("Tuning Search Summary:")
        print("==========================================")
        for ok, err, kp, kd in results:
            status = "SUCCESS" if ok else "FAILED"
            print(f"Kp={kp:.1f}, Kd={kd:.1f} -> {status} (Max Error = {err:.4f} m)")
