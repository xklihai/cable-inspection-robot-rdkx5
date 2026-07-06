# -*- coding: utf-8 -*-
"""差分轮电机 PID 控制节点。"""

from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

from .kinematics import DifferentialDrive
from .pid import PID
from .serial_protocol import build_speed_frame

try:
    import serial
except ImportError:  # pragma: no cover - 无串口库时仍可仿真
    serial = None


class MotorPidController(Node):
    def __init__(self) -> None:
        super().__init__("motor_pid_controller")
        self.declare_parameter("wheel_base_m", 0.235)
        self.declare_parameter("serial.enabled", False)
        self.declare_parameter("serial.port", "/dev/ttyUSB0")
        self.declare_parameter("serial.baudrate", 115200)
        self.drive = DifferentialDrive(float(self.get_parameter("wheel_base_m").value))
        self.left_pid = PID(1.2, 0.10, 0.02, integral_limit=1.0, output_limit=0.45)
        self.right_pid = PID(1.2, 0.10, 0.02, integral_limit=1.0, output_limit=0.45)
        self.target_left = 0.0
        self.target_right = 0.0
        self.measured_left = 0.0
        self.measured_right = 0.0
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_time = time.perf_counter()
        self.serial_dev = self.open_serial()
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 20)
        self.odom_pub = self.create_publisher(Odometry, "/wheel/odom", 20)
        self.debug_pub = self.create_publisher(String, "/motor/debug", 10)
        self.timer = self.create_timer(0.02, self.control_tick)

    def open_serial(self):
        enabled = bool(self.get_parameter("serial.enabled").value)
        if not enabled or serial is None:
            return None
        port = str(self.get_parameter("serial.port").value)
        baud = int(self.get_parameter("serial.baudrate").value)
        try:
            return serial.Serial(port, baudrate=baud, timeout=0.01)
        except Exception as exc:  # pragma: no cover
            self.get_logger().warning(f"串口打开失败，将进入仿真输出模式: {exc}")
            return None

    def on_cmd(self, msg: Twist) -> None:
        self.target_left, self.target_right = self.drive.twist_to_wheels(msg.linear.x, msg.angular.z)

    def control_tick(self) -> None:
        now = time.perf_counter()
        dt = max(now - self.last_time, 1e-3)
        self.last_time = now
        left_out = self.left_pid.update(self.target_left, self.measured_left, dt)
        right_out = self.right_pid.update(self.target_right, self.measured_right, dt)

        # 无硬件反馈时用一阶模型模拟轮速，便于桌面复现闭环。
        self.measured_left += (left_out - self.measured_left) * min(1.0, dt * 8.0)
        self.measured_right += (right_out - self.measured_right) * min(1.0, dt * 8.0)

        frame = build_speed_frame(left_out, right_out)
        if self.serial_dev is not None:
            self.serial_dev.write(frame)
        self.integrate_odom(dt)
        self.debug_pub.publish(String(data=f"left={left_out:.3f}, right={right_out:.3f}, frame={frame.hex()}"))

    def integrate_odom(self, dt: float) -> None:
        v, wz = self.drive.wheels_to_twist(self.measured_left, self.measured_right)
        self.yaw += wz * dt
        self.x += v * math.cos(self.yaw) * dt
        self.y += v * math.sin(self.yaw) * dt
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.orientation.z = math.sin(self.yaw * 0.5)
        msg.pose.pose.orientation.w = math.cos(self.yaw * 0.5)
        msg.twist.twist.linear.x = v
        msg.twist.twist.angular.z = wz
        self.odom_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = MotorPidController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

