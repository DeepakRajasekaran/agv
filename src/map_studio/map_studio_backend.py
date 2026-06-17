#!/usr/bin/env python3
"""
MapStudio Backend — AGV-OS Fleet Management HTTP API + ROS 2 Bridge

Provides:
  - /api/robot_pose     GET   — Live robot pose (x, y, theta)
  - /api/plan           GET   — Current navigation path
  - /api/map            GET   — Active map (vertices + edges) from PostgreSQL
  - /api/navigate       POST  — Send NavigateToPose goal
  - /api/diagnostics    GET   — State machine, CTE, tag, nav velocity, fault log
  - /api/estop          POST  — Emergency stop (zero velocity)
  - /api/maps           GET   — List all available maps
  - /api/mission        POST  — Submit a mission (action sequence)
  - /api/mission/status GET   — Current mission progress
  - /api/sim/inject_fault POST — Inject simulation faults (tape_loss, obstacle)
"""
import json
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, HTTPServer
import psycopg2
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Twist
from nav_msgs.msg import Path
from std_msgs.msg import Float32, String
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType
from rcl_interfaces.msg import ParameterValue

# ══════════════════════════════════════════════════════════════════════════════
# Global State (thread-safe caches)
# ══════════════════════════════════════════════════════════════════════════════

robot_pose = {"x": 0.0, "y": 0.0, "theta": 0.0}
pose_lock = threading.Lock()

planned_path = []
plan_lock = threading.Lock()

diagnostics = {
    "state": "IDLE",
    "cross_track_error": 0.0,
    "last_tag": "",
    "nav_vel": {"linear": 0.0, "angular": 0.0},
    "fault_log": [],
}
diag_lock = threading.Lock()

# Fault log: circular buffer of last 100 entries
fault_log_buffer = deque(maxlen=100)
fault_log_lock = threading.Lock()

# Mission state
mission_state = {
    "status": "IDLE",
    "actions": [],
    "currentIndex": 0,
}
mission_lock = threading.Lock()


def add_fault_log(message, level="info"):
    """Append a timestamped entry to the fault log."""
    ts = time.strftime("%H:%M:%S")
    with fault_log_lock:
        fault_log_buffer.append({"time": ts, "message": message, "level": level})


# ══════════════════════════════════════════════════════════════════════════════
# ROS 2 Node
# ══════════════════════════════════════════════════════════════════════════════

class MapStudioNode(Node):
    def __init__(self):
        super().__init__('map_studio_node')

        # ── Subscribers ────────────────────────────────────────────────────
        self.subscription = self.create_subscription(
            Pose2D, '/robot_pose', self.pose_callback, 10)

        self.plan_subscription = self.create_subscription(
            Path, '/plan', self.plan_callback, 10)

        self.state_subscription = self.create_subscription(
            String, '/controller_state', self.state_callback, 10)

        self.cte_subscription = self.create_subscription(
            Float32, '/sensor/track_position', self.cte_callback, 10)

        self.tag_subscription = self.create_subscription(
            String, '/sensor/tag_id', self.tag_callback, 10)

        self.nav_vel_subscription = self.create_subscription(
            Twist, '/nav_vel', self.nav_vel_callback, 10)

        # ── Publishers ─────────────────────────────────────────────────────
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.fault_pub = self.create_publisher(String, '/sim/inject_fault', 10)

        # ── Action Client ──────────────────────────────────────────────────
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.goal_handle = None

        # ── Parameter Client ───────────────────────────────────────────────
        self._param_client = self.create_client(SetParameters, '/line_follower_controller/set_parameters')

        # ── E-Stop flag ────────────────────────────────────────────────────
        self.estop_active = False

        self.get_logger().info(
            "MapStudio Node initialized. Subscriptions: /robot_pose, /plan, "
            "/controller_state, /sensor/track_position, /sensor/tag_id, /nav_vel"
        )
        add_fault_log("SYSTEM READY")

    # ── Callbacks ──────────────────────────────────────────────────────────

    def pose_callback(self, msg):
        global robot_pose, planned_path
        with pose_lock:
            robot_pose["x"] = msg.x
            robot_pose["y"] = msg.y
            robot_pose["theta"] = msg.theta

        # Clear plan if robot reaches the end
        with plan_lock:
            if planned_path:
                last_pt = planned_path[-1]
                dist = ((msg.x - last_pt["x"])**2 + (msg.y - last_pt["y"])**2)**0.5
                if dist < 0.2:
                    planned_path = []
                    self.get_logger().info("Robot reached end of planned path. Clearing plan.")

    def plan_callback(self, msg):
        global planned_path
        points = []
        for pose_stamped in msg.poses:
            points.append({
                "x": pose_stamped.pose.position.x,
                "y": pose_stamped.pose.position.y
            })
        with plan_lock:
            planned_path = points
        self.get_logger().info(f"Received path with {len(points)} waypoints")

    def state_callback(self, msg):
        with diag_lock:
            old_state = diagnostics["state"]
            diagnostics["state"] = msg.data
        if msg.data != old_state:
            add_fault_log(f"STATE_CHANGE -> {msg.data}")
            self.get_logger().info(f"Controller state: {msg.data}")

    def cte_callback(self, msg):
        with diag_lock:
            diagnostics["cross_track_error"] = msg.data

    def tag_callback(self, msg):
        if msg.data:
            with diag_lock:
                diagnostics["last_tag"] = msg.data
            add_fault_log(f"{msg.data} DETECTED")
            self.get_logger().info(f"Tag detected: {msg.data}")

    def nav_vel_callback(self, msg):
        with diag_lock:
            diagnostics["nav_vel"]["linear"] = msg.linear.x
            diagnostics["nav_vel"]["angular"] = msg.angular.z

    # ── Navigation Goal ────────────────────────────────────────────────────

    def send_goal(self, x, y):
        """Send a NavigateToPose goal, cancelling any active one first."""
        if self.goal_handle is not None:
            self.get_logger().info("Cancelling active goal...")
            try:
                self.goal_handle.cancel_goal_async()
            except Exception as e:
                self.get_logger().error(f"Error cancelling goal: {e}")
            self.goal_handle = None

        if not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("NavigateToPose action server not available!")
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info(f"Sending goal: ({x}, {y})")
        future = self._action_client.send_goal_async(goal_msg)
        future.add_done_callback(self._goal_response_callback)
        return True

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            add_fault_log("GOAL REJECTED", "warn")
            return
        self.get_logger().info('Goal accepted')
        add_fault_log("GOAL ACCEPTED")
        self.goal_handle = goal_handle

    # ── Tuning ─────────────────────────────────────────────────────────────

    def tune_controller(self, params_dict):
        """Send parameters to the PID controller dynamically."""
        if not self._param_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Parameter service /line_follower_controller/set_parameters not available!")
            return False

        req = SetParameters.Request()
        for key, val in params_dict.items():
            p = Parameter()
            p.name = key
            p.value = ParameterValue()
            if isinstance(val, float) or isinstance(val, int):
                p.value.type = ParameterType.PARAMETER_DOUBLE
                p.value.double_value = float(val)
            req.parameters.append(p)

        self._param_client.call_async(req)
        self.get_logger().info(f"Sent tuning parameters: {params_dict}")
        return True

    # ── E-Stop ─────────────────────────────────────────────────────────────

    def emergency_stop(self):
        """Publish zero velocity and set E-Stop flag."""
        self.estop_active = True
        stop_msg = Twist()  # All zeros
        self.cmd_vel_pub.publish(stop_msg)
        self.get_logger().warn("E-STOP ACTIVATED")
        add_fault_log("E-STOP ACTIVATED", "error")

        # Cancel active goal
        if self.goal_handle is not None:
            try:
                self.goal_handle.cancel_goal_async()
            except Exception:
                pass
            self.goal_handle = None

    # ── Fault Injection ────────────────────────────────────────────────────

    def inject_fault(self, fault_type):
        """Publish a fault injection command to the hardware sim."""
        msg = String()
        msg.data = fault_type
        self.fault_pub.publish(msg)
        self.get_logger().info(f"Fault injected: {fault_type}")
        add_fault_log(f"FAULT_INJECT: {fault_type.upper()}", "warn")


# ══════════════════════════════════════════════════════════════════════════════
# HTTP Server
# ══════════════════════════════════════════════════════════════════════════════

class MapStudioHTTPHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        pass  # Suppress default HTTP logging

    def _set_headers(self, content_type="application/json", status=200):
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def _json_response(self, data, status=200):
        self._set_headers(status=status)
        self.wfile.write(json.dumps(data).encode('utf-8'))

    def _error_response(self, message, status=500):
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(json.dumps({"error": message}).encode('utf-8'))

    def _read_json_body(self):
        content_length = int(self.headers.get('Content-Length', 0))
        raw = self.rfile.read(content_length)
        return json.loads(raw.decode('utf-8')) if raw else {}

    def do_OPTIONS(self):
        self._set_headers()

    def do_GET(self):
        if self.path == '/api/robot_pose':
            with pose_lock:
                self._json_response(robot_pose)

        elif self.path == '/api/plan':
            with plan_lock:
                self._json_response(planned_path)

        elif self.path == '/api/diagnostics':
            with diag_lock:
                diag_copy = dict(diagnostics)
            with fault_log_lock:
                diag_copy["fault_log"] = list(fault_log_buffer)
            self._json_response(diag_copy)

        elif self.path == '/api/mission/status':
            with mission_lock:
                self._json_response(mission_state)

        elif self.path == '/api/maps':
            try:
                conn = self._get_db_connection()
                cursor = conn.cursor()
                cursor.execute("SELECT id, name, is_active FROM maps ORDER BY id;")
                maps = [{"id": r[0], "name": r[1], "is_active": r[2]} for r in cursor.fetchall()]
                cursor.close()
                conn.close()
                self._json_response(maps)
            except Exception:
                # If maps table doesn't exist yet, return default
                self._json_response([{"id": 1, "name": "ASSEMBLY LINE 1", "is_active": True}])

        elif self.path == '/api/map':
            try:
                conn = self._get_db_connection()
                cursor = conn.cursor()

                cursor.execute("SELECT id, pos_x, pos_y, type FROM vertices;")
                vertices = [{"id": r[0], "x": r[1], "y": r[2], "type": r[3]} for r in cursor.fetchall()]

                cursor.execute("SELECT id, start_vertex_id, end_vertex_id, is_bidirectional, curve_type, control_points FROM edges;")
                edges = [{
                    "id": r[0], "start_vertex_id": r[1], "end_vertex_id": r[2],
                    "is_bidirectional": r[3], "curve_type": r[4], "control_points": r[5]
                } for r in cursor.fetchall()]

                cursor.close()
                conn.close()
                self._json_response({"vertices": vertices, "edges": edges})
            except Exception as e:
                self._error_response(str(e))

        elif self.path == '/api/sim/status':
            try:
                import subprocess
                # Check if play.launch.py is running
                result = subprocess.run(["pgrep", "-f", "play.launch.py"], capture_output=True)
                is_running = (result.returncode == 0)
                self._json_response({"status": "running" if is_running else "stopped"})
            except Exception as e:
                self._error_response(str(e))

        else:
            self._error_response("Not Found", 404)

    def do_POST(self):
        if self.path == '/api/navigate':
            try:
                params = self._read_json_body()
                x = float(params.get('x', 0.0))
                y = float(params.get('y', 0.0))
                global ros_node
                success = ros_node.send_goal(x, y) if ros_node else False
                self._json_response({"success": success})
            except Exception as e:
                self._error_response(str(e))

        elif self.path == '/api/estop':
            try:
                if ros_node:
                    ros_node.emergency_stop()
                self._json_response({"success": True, "message": "E-STOP activated"})
            except Exception as e:
                self._error_response(str(e))

        elif self.path == '/api/controller/tune':
            try:
                params = self._read_json_body()
                if ros_node:
                    ros_node.tune_controller(params)
                self._json_response({"status": "success", "message": "Parameters sent"})
            except Exception as e:
                self._error_response(str(e))

        elif self.path == '/api/sim/inject_fault':
            try:
                params = self._read_json_body()
                fault_type = params.get('type', 'tape_loss')
                if ros_node:
                    ros_node.inject_fault(fault_type)
                self._json_response({"success": True, "fault_type": fault_type})
            except Exception as e:
                self._error_response(str(e))

        elif self.path == '/api/sim/start':
            try:
                params = self._read_json_body() if int(self.headers.get('Content-Length', 0)) > 0 else {}
                headless = params.get('headless', False)
                headless_str = "true" if headless else "false"
                import subprocess
                add_fault_log(f"SIMULATION STARTING (headless={headless_str})...")
                subprocess.Popen(["ros2", "launch", "line_follower_controller", "play.launch.py", f"headless:={headless_str}"])
                self._json_response({"success": True, "message": "Simulation started"})
            except Exception as e:
                self._error_response(str(e))

        elif self.path == '/api/sim/restart':
            try:
                params = self._read_json_body() if int(self.headers.get('Content-Length', 0)) > 0 else {}
                headless = params.get('headless', False)
                headless_str = "true" if headless else "false"
                import subprocess
                # Kill the existing play.launch.py process
                subprocess.run(["pkill", "-f", "play.launch.py"])
                add_fault_log(f"SIMULATION RESTARTING (headless={headless_str})...")
                # Start a new one detached
                subprocess.Popen(["ros2", "launch", "line_follower_controller", "play.launch.py", f"headless:={headless_str}"])
                self._json_response({"success": True, "message": "Simulation restarted"})
            except Exception as e:
                self._error_response(str(e))

        elif self.path == '/api/mission':
            try:
                params = self._read_json_body()
                actions = params.get('actions', [])
                with mission_lock:
                    mission_state["status"] = "RUNNING"
                    mission_state["actions"] = actions
                    mission_state["currentIndex"] = 0
                add_fault_log(f"MISSION RECEIVED ({len(actions)} actions)")
                # Start sequential execution in background
                threading.Thread(target=execute_mission, args=(actions,), daemon=True).start()
                self._json_response({"success": True, "actions_count": len(actions)})
            except Exception as e:
                self._error_response(str(e))

        else:
            self._error_response("Not Found", 404)

    def _get_db_connection(self):
        return psycopg2.connect(
            dbname="map_db",
            user="nav_user",
            password="nav_password",
            host="172.18.0.1",
            port=5432
        )


# ══════════════════════════════════════════════════════════════════════════════
# Mission Executor
# ══════════════════════════════════════════════════════════════════════════════

def execute_mission(actions):
    """Execute a sequence of mission actions one by one."""
    global ros_node
    for idx, action in enumerate(actions):
        with mission_lock:
            mission_state["currentIndex"] = idx
            mission_state["status"] = "RUNNING"

        action_type = action.get("type", "")
        add_fault_log(f"EXECUTING: {action_type} (step {idx+1}/{len(actions)})")

        if action_type == "MOVE_TO_TAG":
            x = float(action.get("target_x", 0.0))
            y = float(action.get("target_y", 0.0))
            if ros_node:
                ros_node.send_goal(x, y)
            # Wait for robot to reach goal (simple proximity check)
            while True:
                with pose_lock:
                    dx = robot_pose["x"] - x
                    dy = robot_pose["y"] - y
                dist = (dx**2 + dy**2)**0.5
                if dist < 0.3:
                    break
                time.sleep(0.5)

        elif action_type == "WAIT":
            wait_time = float(action.get("duration", 30.0))
            add_fault_log(f"WAITING {wait_time}s")
            time.sleep(wait_time)

        elif action_type == "DOCK":
            add_fault_log(f"DOCKING at {action.get('target', 'UNKNOWN')}")
            time.sleep(5.0)  # Simulate docking

        add_fault_log(f"COMPLETED: {action_type} (step {idx+1}/{len(actions)})")

    with mission_lock:
        mission_state["status"] = "COMPLETED"
        mission_state["currentIndex"] = len(actions)
    add_fault_log("MISSION COMPLETED")


# ══════════════════════════════════════════════════════════════════════════════
# Main
# ══════════════════════════════════════════════════════════════════════════════

ros_node = None


def run_http_server(port=8080):
    server_address = ('', port)
    httpd = HTTPServer(server_address, MapStudioHTTPHandler)
    print(f"MapStudio Backend HTTP API Server listening on port {port}...")
    httpd.serve_forever()


def main(args=None):
    global ros_node
    rclpy.init(args=args)
    ros_node = MapStudioNode()

    ros_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    ros_thread.start()

    try:
        run_http_server(8080)
    except KeyboardInterrupt:
        pass
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
