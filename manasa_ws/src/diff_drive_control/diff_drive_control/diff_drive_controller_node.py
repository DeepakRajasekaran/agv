#!/usr/bin/env python3
"""
diff_drive_controller_node
---------------------------
A standalone differential-drive controller node (topic-based, not a
hardware_interface plugin) that bridges high-level velocity commands and
low-level wheel RPM commands/feedback.

Data flow:

    /cmd_vel  (geometry_msgs/Twist)            [IN]  from teleop or nav stack
        |
        v   inverse kinematics (uses wheel_base, wheel_dia, gear_ratio)
        |
    /cmd_rpm  (std_msgs/Float32MultiArray)      [OUT] -> motor driver
              data = [left_motor_rpm, right_motor_rpm]

    /drive_rpm (std_msgs/Float32MultiArray)     [IN]  <- motor driver / encoders
              data = [left_motor_rpm, right_motor_rpm]
        |
        v   forward kinematics + dead-reckoning integration
        |
    /odom     (nav_msgs/Odometry)               [OUT] -> rest of the stack
    odom -> base_link TF                        [OUT]

Kinematic parameters (wheel_base, wheel_dia, gear_ratio) are declared as
ROS 2 parameters and can be set at launch time via YAML or live via
`ros2 param set`.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from rcl_interfaces.msg import SetParametersResult
import tf2_ros


class DiffDriveControllerNode(Node):

    def __init__(self):
        super().__init__('diff_drive_controller_node')

        # ---------------- Configurable kinematic parameters ----------------
        self.declare_parameter('wheel_base', 0.40)        # m, distance between wheel centers
        self.declare_parameter('wheel_dia', 0.15)         # m, wheel diameter
        self.declare_parameter('gear_ratio', 30.0)        # motor shaft rev per 1 wheel rev
        self.declare_parameter('max_motor_rpm', 300.0)    # clamp; <=0 disables clamping
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('cmd_vel_timeout', 0.5)    # s, safety stop if cmd_vel goes stale

        self._load_params()
        self.add_on_set_parameters_callback(self._on_param_change)

        # ---------------- State ----------------
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_odom_time = self.get_clock().now()
        self.last_cmd_vel_time = self.get_clock().now()

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                          history=HistoryPolicy.KEEP_LAST)

        # ---------------- Subscribers ----------------
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, qos)

        self.drive_rpm_sub = self.create_subscription(
            Float32MultiArray, '/drive_rpm', self.drive_rpm_callback, qos)

        # ---------------- Publishers ----------------
        self.cmd_rpm_pub = self.create_publisher(Float32MultiArray, '/cmd_rpm', qos)
        self.odom_pub = self.create_publisher(Odometry, '/odom', qos)

        # ---------------- TF ----------------
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # ---------------- Safety watchdog ----------------
        self.watchdog_timer = self.create_timer(0.1, self.watchdog_callback)

        self.get_logger().info(
            f'diff_drive_controller_node started | wheel_base={self.wheel_base} m, '
            f'wheel_dia={self.wheel_dia} m, gear_ratio={self.gear_ratio}'
        )

    # -------------------------------------------------------------------
    def _load_params(self):
        self.wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value
        self.wheel_dia = self.get_parameter('wheel_dia').get_parameter_value().double_value
        self.gear_ratio = self.get_parameter('gear_ratio').get_parameter_value().double_value
        self.max_motor_rpm = self.get_parameter('max_motor_rpm').get_parameter_value().double_value
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.publish_tf = self.get_parameter('publish_tf').get_parameter_value().bool_value
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').get_parameter_value().double_value

    def _on_param_change(self, params):
        for p in params:
            if p.name in ('wheel_base', 'wheel_dia', 'gear_ratio') and p.value <= 0.0:
                return SetParametersResult(successful=False, reason=f'{p.name} must be > 0')
        self._load_params()
        self.get_logger().info('Kinematic parameters updated live.')
        return SetParametersResult(successful=True)

    # -------------------------------------------------------------------
    # /cmd_vel -> /cmd_rpm  (inverse kinematics)
    # -------------------------------------------------------------------
    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd_vel_time = self.get_clock().now()

        v = msg.linear.x    # m/s, forward velocity
        w = msg.angular.z   # rad/s, yaw rate

        v_left = v - (w * self.wheel_base / 2.0)
        v_right = v + (w * self.wheel_base / 2.0)

        left_rpm = self._clamp_rpm(self._wheel_linear_vel_to_motor_rpm(v_left))
        right_rpm = self._clamp_rpm(self._wheel_linear_vel_to_motor_rpm(v_right))

        out = Float32MultiArray()
        out.data = [float(left_rpm), float(right_rpm)]
        self.cmd_rpm_pub.publish(out)

    def _wheel_linear_vel_to_motor_rpm(self, v_wheel):
        wheel_radius = self.wheel_dia / 2.0
        if wheel_radius <= 0.0:
            return 0.0
        wheel_angular_vel = v_wheel / wheel_radius              # rad/s at the wheel
        wheel_rpm = wheel_angular_vel * 60.0 / (2.0 * math.pi)  # rev/min at the wheel
        return wheel_rpm * self.gear_ratio                      # rev/min at the motor shaft

    def _clamp_rpm(self, rpm):
        if self.max_motor_rpm and self.max_motor_rpm > 0.0:
            return max(-self.max_motor_rpm, min(self.max_motor_rpm, rpm))
        return rpm

    def watchdog_callback(self):
        elapsed = (self.get_clock().now() - self.last_cmd_vel_time).nanoseconds / 1e9
        if elapsed > self.cmd_vel_timeout:
            zero = Float32MultiArray()
            zero.data = [0.0, 0.0]
            self.cmd_rpm_pub.publish(zero)

    # -------------------------------------------------------------------
    # /drive_rpm -> /odom  (forward kinematics + dead-reckoning)
    # -------------------------------------------------------------------
    def drive_rpm_callback(self, msg: Float32MultiArray):
        if len(msg.data) < 2:
            self.get_logger().warn('drive_rpm message must contain [left_rpm, right_rpm]')
            return

        left_motor_rpm, right_motor_rpm = msg.data[0], msg.data[1]

        now = self.get_clock().now()
        dt = (now - self.last_odom_time).nanoseconds / 1e9
        self.last_odom_time = now
        if dt <= 0.0:
            return

        v_left = self._motor_rpm_to_wheel_linear_vel(left_motor_rpm)
        v_right = self._motor_rpm_to_wheel_linear_vel(right_motor_rpm)

        v = (v_right + v_left) / 2.0
        w = (v_right - v_left) / self.wheel_base if self.wheel_base else 0.0

        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta += w * dt
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))  # normalize

        self._publish_odom(v, w, now)

    def _motor_rpm_to_wheel_linear_vel(self, motor_rpm):
        wheel_rpm = motor_rpm / self.gear_ratio if self.gear_ratio else 0.0
        wheel_angular_vel = wheel_rpm * 2.0 * math.pi / 60.0
        return wheel_angular_vel * (self.wheel_dia / 2.0)

    def _publish_odom(self, v, w, stamp):
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)

        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w

        self.odom_pub.publish(odom)

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = stamp.to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
