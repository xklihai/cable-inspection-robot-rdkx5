# -*- coding: utf-8 -*-
"""差分轮电机 PID 控制节点。

订阅 /cmd_vel 速度指令，通过运动学解算得到左右轮目标速度，再经双路 PID
控制器生成输出；最终通过串口发送给 WHEELTEC C30D 主控板，同时发布轮式里程计。

订阅话题：
    /cmd_vel (geometry_msgs/Twist): 机器人级速度指令。

发布话题：
    /wheel/odom (nav_msgs/Odometry): 基于轮速积分得到的里程计。
    /motor/debug (std_msgs/String): PID 输出与串口帧调试信息。
"""

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
    """差分轮电机 PID 控制器：将 Twist 指令转换为左右轮控制量并发布里程计。"""

    def __init__(self) -> None:
        super().__init__("motor_pid_controller")

        # 运动学与串口参数
        self.declare_parameter("wheel_base_m", 0.235)
        self.declare_parameter("serial.enabled", False)
        self.declare_parameter("serial.port", "/dev/ttyUSB0")
        self.declare_parameter("serial.baudrate", 115200)

        # 初始化差分轮运动学模型
        self.drive = DifferentialDrive(float(self.get_parameter("wheel_base_m").value))

        # 左右轮独立的 PID 控制器（参数可根据实际电机响应整定）
        self.left_pid = PID(1.2, 0.10, 0.02, integral_limit=1.0, output_limit=0.45)
        self.right_pid = PID(1.2, 0.10, 0.02, integral_limit=1.0, output_limit=0.45)

        # 目标轮速与当前轮速（仿真模式下为模拟反馈值）
        self.target_left = 0.0
        self.target_right = 0.0
        self.measured_left = 0.0
        self.measured_right = 0.0

        # 里程计积分状态
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_time = time.perf_counter()

        # 尝试打开串口；失败则进入纯仿真模式
        self.serial_dev = self.open_serial()

        # 订阅速度指令，发布里程计与调试信息
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 20)
        self.odom_pub = self.create_publisher(Odometry, "/wheel/odom", 20)
        self.debug_pub = self.create_publisher(String, "/motor/debug", 10)

        # 以 50 Hz 运行控制循环
        self.timer = self.create_timer(0.02, self.control_tick)

    def open_serial(self):
        """根据参数打开 C30D 串口。

        Returns:
            serial.Serial 实例或 None（禁用/失败时进入仿真模式）。
        """
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
        """速度指令回调：将 Twist 解算为左右轮目标速度。"""
        self.target_left, self.target_right = self.drive.twist_to_wheels(msg.linear.x, msg.angular.z)

    def control_tick(self) -> None:
        """控制主循环：运行 PID、更新仿真反馈、发送串口帧、发布里程计。"""
        now = time.perf_counter()
        dt = max(now - self.last_time, 1e-3)  # 防止时间间隔过小或为零
        self.last_time = now

        # 分别计算左右轮 PID 输出
        left_out = self.left_pid.update(self.target_left, self.measured_left, dt)
        right_out = self.right_pid.update(self.target_right, self.measured_right, dt)

        # 无硬件反馈时用一阶惯性模型模拟轮速，便于桌面复现闭环。
        # 时间常数约为 1/8 = 0.125 s，表示电机从 0 到达目标 63% 所需时间。
        self.measured_left += (left_out - self.measured_left) * min(1.0, dt * 8.0)
        self.measured_right += (right_out - self.measured_right) * min(1.0, dt * 8.0)

        # 构造串口帧并发送；串口未打开时仅做调试输出
        frame = build_speed_frame(left_out, right_out)
        if self.serial_dev is not None:
            self.serial_dev.write(frame)

        # 根据当前轮速积分并发布里程计
        self.integrate_odom(dt)

        # 发布调试信息，便于观察 PID 输出与下发的十六进制帧
        self.debug_pub.publish(String(data=f"left={left_out:.3f}, right={right_out:.3f}, frame={frame.hex()}"))

    def integrate_odom(self, dt: float) -> None:
        """根据左右轮测量速度积分得到机器人位姿，并发布 /wheel/odom。

        采用简单航位推算（dead reckoning）：
            yaw_{t+1} = yaw_t + ω * dt
            x_{t+1}   = x_t + v * cos(yaw) * dt
            y_{t+1}   = y_t + v * sin(yaw) * dt

        Args:
            dt: 时间间隔（秒）。
        """
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
        # 假设机器人只在 2D 平面运动，仅使用四元数的 z/w 分量表示 yaw
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

