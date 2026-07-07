# -*- coding: utf-8 -*-
"""差分轮电机 PID 控制节点。

本节点订阅 /cmd_vel 速度指令，通过差分轮运动学解算得到左右轮目标速度，
再经双路 PID 控制器计算输出；最终通过串口发送给 WHEELTEC C30D 主控板，
同时基于轮速积分发布 /wheel/odom 里程计。

订阅话题：
    /cmd_vel (geometry_msgs/Twist): 机器人级速度指令。

发布话题：
    /wheel/odom (nav_msgs/Odometry): 基于轮速积分得到的轮式里程计。
    /motor/debug (std_msgs/String): PID 输出与串口帧调试信息。
"""

# 启用类型注解的未来导入
from __future__ import annotations

# 导入 math 模块，用于三角函数和积分计算
import math

# 导入 time 模块，用于获取高精度计时
import time

# 导入 ROS2 Python 客户端库
import rclpy

# 导入 ROS2 Twist 消息类型，用于订阅速度指令
from geometry_msgs.msg import Twist

# 导入 ROS2 Odometry 消息类型，用于发布轮式里程计
from nav_msgs.msg import Odometry

# 导入 ROS2 节点基类
from rclpy.node import Node

# 导入 ROS2 String 消息类型，用于发布调试信息
from std_msgs.msg import String

# 从本包导入差分轮运动学类
from .kinematics import DifferentialDrive

# 从本包导入 PID 控制器类
from .pid import PID

# 从本包导入串口帧构造函数
from .serial_protocol import build_speed_frame

# 尝试导入 pyserial 库；若未安装则进入纯仿真模式
# 在桌面环境或没有串口硬件时，程序仍可正常运行
try:
    import serial
except ImportError:  # pragma: no cover - 无串口库时仍可仿真
    serial = None


# 定义电机 PID 控制节点类，继承自 ROS2 Node 基类
class MotorPidController(Node):
    """差分轮电机 PID 控制器：将 Twist 指令转换为左右轮控制量并发布里程计。"""

    def __init__(self) -> None:
        """节点构造函数：声明参数、初始化 PID、打开串口、创建发布者与订阅者。"""

        # 调用父类构造函数，设置节点名称为 "motor_pid_controller"
        super().__init__("motor_pid_controller")

        # 声明轮基线参数，单位米，默认 0.235 m
        # 该参数用于差分轮运动学解算
        self.declare_parameter("wheel_base_m", 0.235)

        # 声明串口使能参数，默认 False
        # 首次调试建议保持 False，确认速度指令正常后再启用真实串口输出
        self.declare_parameter("serial.enabled", False)

        # 声明串口设备路径参数，默认 /dev/ttyUSB0
        # Linux 下 USB 转串口常见设备名
        self.declare_parameter("serial.port", "/dev/ttyUSB0")

        # 声明串口波特率参数，默认 115200 bps
        # 需与 C30D 控制器串口配置保持一致
        self.declare_parameter("serial.baudrate", 115200)

        # 读取轮基线参数并实例化差分轮运动学模型
        self.drive = DifferentialDrive(float(self.get_parameter("wheel_base_m").value))

        # 初始化左轮 PID 控制器
        # 参数说明：Kp=1.2, Ki=0.10, Kd=0.02，积分限幅 1.0，输出限幅 0.45
        # 这些参数为默认值，实际应根据电机响应进行整定
        self.left_pid = PID(1.2, 0.10, 0.02, integral_limit=1.0, output_limit=0.45)

        # 初始化右轮 PID 控制器，参数与左轮相同
        self.right_pid = PID(1.2, 0.10, 0.02, integral_limit=1.0, output_limit=0.45)

        # 初始化左轮目标速度，单位 m/s
        self.target_left = 0.0

        # 初始化右轮目标速度，单位 m/s
        self.target_right = 0.0

        # 初始化左轮测量速度，单位 m/s
        # 在仿真模式下表示模拟反馈值；在真实硬件模式下应由编码器反馈得到
        self.measured_left = 0.0

        # 初始化右轮测量速度，单位 m/s
        self.measured_right = 0.0

        # 初始化里程计 x 坐标，单位米
        self.x = 0.0

        # 初始化里程计 y 坐标，单位米
        self.y = 0.0

        # 初始化机器人航向角 yaw，单位弧度
        self.yaw = 0.0

        # 记录上一次控制循环的时间戳，用于计算时间间隔 dt
        self.last_time = time.perf_counter()

        # 根据参数尝试打开串口设备
        # 若串口未启用或打开失败，self.serial_dev 为 None，进入仿真模式
        self.serial_dev = self.open_serial()

        # 订阅 /cmd_vel 话题，接收来自规划节点的速度指令
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 20)

        # 创建 /wheel/odom 发布者，用于发布轮式里程计
        self.odom_pub = self.create_publisher(Odometry, "/wheel/odom", 20)

        # 创建 /motor/debug 发布者，用于发布 PID 输出与串口帧调试信息
        self.debug_pub = self.create_publisher(String, "/motor/debug", 10)

        # 创建定时器，每 0.02 秒（50 Hz）调用一次 control_tick()
        # 较高的控制频率有助于提升电机响应与里程计精度
        self.timer = self.create_timer(0.02, self.control_tick)

    def open_serial(self):
        """根据参数打开 C30D 串口设备。

        Returns:
            serial.Serial 实例：串口成功打开时返回。
            None：串口未启用、pyserial 未安装或打开失败时返回，
                  此时节点进入纯仿真输出模式。
        """

        # 读取 serial.enabled 参数，判断是否需要打开真实串口
        enabled = bool(self.get_parameter("serial.enabled").value)

        # 如果串口未启用或 pyserial 未安装，则返回 None
        if not enabled or serial is None:
            return None

        # 读取串口设备路径参数
        port = str(self.get_parameter("serial.port").value)

        # 读取串口波特率参数并转换为整数
        baud = int(self.get_parameter("serial.baudrate").value)

        try:
            # 尝试以指定波特率和 0.01 秒超时打开串口
            return serial.Serial(port, baudrate=baud, timeout=0.01)
        except Exception as exc:  # pragma: no cover
            # 打开失败时记录警告日志，进入仿真模式，不中断程序运行
            self.get_logger().warning(f"串口打开失败，将进入仿真输出模式: {exc}")
            return None

    def on_cmd(self, msg: Twist) -> None:
        """速度指令回调函数：将 Twist 解算为左右轮目标速度。

        Args:
            msg: 包含 linear.x 和 angular.z 的 Twist 速度指令。
        """

        # 调用差分轮逆运动学，将机器人速度转换为左右轮目标速度
        self.target_left, self.target_right = self.drive.twist_to_wheels(msg.linear.x, msg.angular.z)

    def control_tick(self) -> None:
        """控制主循环：运行 PID、更新仿真反馈、发送串口帧、发布里程计。

        该函数以 50 Hz 频率被定时器调用，是电机控制的核心。
        """

        # 获取当前高精度时间戳
        now = time.perf_counter()

        # 计算距离上一次控制循环的时间间隔 dt
        # 使用 max(now - self.last_time, 1e-3) 防止 dt 过小或为负
        dt = max(now - self.last_time, 1e-3)

        # 更新上一次时间戳为当前时间
        self.last_time = now

        # 计算左轮 PID 输出
        # 输入为目标速度与测量速度，输出为控制量
        left_out = self.left_pid.update(self.target_left, self.measured_left, dt)

        # 计算右轮 PID 输出
        right_out = self.right_pid.update(self.target_right, self.measured_right, dt)

        # 无硬件反馈时用一阶惯性模型模拟轮速，便于桌面复现闭环。
        # 模型形式：v_new = v_old + (u - v_old) * alpha
        # 其中 alpha = min(1.0, dt * 8.0)，时间常数约为 1/8 = 0.125 秒。
        # 物理意义：电机速度不能瞬时达到目标值，而是按指数规律趋近。
        self.measured_left += (left_out - self.measured_left) * min(1.0, dt * 8.0)
        self.measured_right += (right_out - self.measured_right) * min(1.0, dt * 8.0)

        # 根据左右轮 PID 输出构造串口控制帧
        frame = build_speed_frame(left_out, right_out)

        # 如果串口设备成功打开，则将控制帧写入串口
        if self.serial_dev is not None:
            self.serial_dev.write(frame)

        # 根据当前轮速积分并发布轮式里程计
        self.integrate_odom(dt)

        # 发布调试信息，包含 PID 输出值与下发的十六进制串口帧
        self.debug_pub.publish(String(data=f"left={left_out:.3f}, right={right_out:.3f}, frame={frame.hex()}"))

    def integrate_odom(self, dt: float) -> None:
        """根据左右轮测量速度积分得到机器人位姿，并发布 /wheel/odom。

        采用简单二维航位推算（dead reckoning）：
            yaw_{t+1} = yaw_t + ω * dt
            x_{t+1}   = x_t + v * cos(yaw) * dt
            y_{t+1}   = y_t + v * sin(yaw) * dt

        其中 v 为机器人线速度，ω 为机器人角速度。

        Args:
            dt: 时间间隔，单位秒。
        """

        # 调用差分轮正运动学，将左右轮测量速度转换为机器人线速度和角速度
        v, wz = self.drive.wheels_to_twist(self.measured_left, self.measured_right)

        # 更新航向角：yaw 对时间积分
        self.yaw += wz * dt

        # 更新 x 坐标：线速度在 x 轴方向的分量对时间积分
        self.x += v * math.cos(self.yaw) * dt

        # 更新 y 坐标：线速度在 y 轴方向的分量对时间积分
        self.y += v * math.sin(self.yaw) * dt

        # 创建 Odometry 消息对象
        msg = Odometry()

        # 设置消息时间戳为当前 ROS 时间
        msg.header.stamp = self.get_clock().now().to_msg()

        # 设置父坐标系为 "odom"
        msg.header.frame_id = "odom"

        # 设置子坐标系为 "base_link"
        msg.child_frame_id = "base_link"

        # 填充位置坐标
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y

        # 假设机器人只在二维平面运动，z 方向位置为 0
        msg.pose.pose.position.z = 0.0

        # 根据 yaw 角构造四元数（假设 roll = pitch = 0）
        # 四元数 z 分量 = sin(yaw / 2)
        msg.pose.pose.orientation.z = math.sin(self.yaw * 0.5)

        # 四元数 w 分量 = cos(yaw / 2)
        msg.pose.pose.orientation.w = math.cos(self.yaw * 0.5)

        # x 和 y 分量为 0（纯 z 轴旋转）
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0

        # 填充速度信息
        msg.twist.twist.linear.x = v
        msg.twist.twist.angular.z = wz

        # 发布轮式里程计消息
        self.odom_pub.publish(msg)


# 定义程序入口函数
def main() -> None:
    """节点主函数：初始化 ROS2、创建节点、进入 spin 循环、清理资源。"""

    # 初始化 ROS2 Python 客户端库
    rclpy.init()

    # 创建 MotorPidController 实例
    node = MotorPidController()

    # 进入事件循环
    rclpy.spin(node)

    # 销毁节点
    node.destroy_node()

    # 关闭 ROS2
    rclpy.shutdown()


# 当该脚本直接运行时，调用 main() 函数
if __name__ == "__main__":
    main()
