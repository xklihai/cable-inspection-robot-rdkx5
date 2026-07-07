# -*- coding: utf-8 -*-
"""巡检数据记录节点。

本节点负责将巡检过程中的关键数据持久化到本地文件系统，便于后续复现、
离线分析和生成巡检报告。

订阅话题：
    /perception/cable_offset (std_msgs/Float32MultiArray):
        线缆偏移与置信度。
    /odom/fused (nav_msgs/Odometry):
        融合里程计。
    /inspection/state (std_msgs/String):
        巡检状态。
    /perception/debug_image (sensor_msgs/Image):
        可视化调试图像。

输出文件：
    <log_dir>/inspection.csv:
        CSV 格式日志文件，包含时间戳、话题类型、位姿、偏移量、状态等字段。
    <log_dir>/images/debug_*.jpg:
        按 1/10 采样保存的调试图像。
"""

# 启用类型注解的未来导入
from __future__ import annotations

# 导入 csv 模块，用于写入 CSV 日志文件
import csv

# 导入 Path，用于跨平台路径操作
from pathlib import Path

# 导入 ROS2 Python 客户端库
import rclpy

# 导入 CvBridge，用于将 ROS Image 转换为 OpenCV 图像以便保存
from cv_bridge import CvBridge

# 导入 ROS2 Odometry 消息类型
from nav_msgs.msg import Odometry

# 导入 ROS2 节点基类
from rclpy.node import Node

# 导入 ROS2 Image 消息类型
from sensor_msgs.msg import Image

# 导入 ROS2 标准消息类型
from std_msgs.msg import Float32MultiArray, String


# 定义数据记录节点类，继承自 ROS2 Node 基类
class DataLoggerNode(Node):
    """巡检数据记录节点：CSV 日志 + 调试图像抽样保存。"""

    def __init__(self) -> None:
        """节点构造函数：声明参数、创建目录、打开 CSV 文件、订阅话题。"""

        # 调用父类构造函数，设置节点名称为 "data_logger_node"
        super().__init__("data_logger_node")

        # 声明日志目录参数，默认 "inspection_logs"
        # 该目录将存放 CSV 文件和图像子目录
        self.declare_parameter("log_dir", "inspection_logs")

        # 声明是否保存调试图像参数，默认 True
        self.declare_parameter("save_debug_images", True)

        # 读取日志目录参数并转换为 Path 对象
        self.log_dir = Path(str(self.get_parameter("log_dir").value))

        # 定义图像保存子目录路径
        self.image_dir = self.log_dir / "images"

        # 自动创建日志目录与图像子目录
        # parents=True 表示同时创建父目录；exist_ok=True 表示目录已存在时不报错
        self.image_dir.mkdir(parents=True, exist_ok=True)

        # 创建 CvBridge 实例，用于图像格式转换
        self.bridge = CvBridge()

        # 打开 CSV 日志文件，使用 utf-8 编码以支持中文
        # newline="" 是 csv 模块推荐写法，避免 Windows 下出现空行
        self.csv_file = (self.log_dir / "inspection.csv").open("w", newline="", encoding="utf-8")

        # 创建 csv writer 对象
        self.writer = csv.writer(self.csv_file)

        # 写入 CSV 表头
        # 字段设计：
        #   time_sec: 时间戳（秒）
        #   topic: 数据来源话题类型
        #   x, y: 里程计位置坐标
        #   yaw_or_offset: 里程计时保存 yaw（z/w），偏移时保存 offset
        #   state_or_confidence: 状态时保存状态字符串，偏移时保存置信度
        self.writer.writerow(["time_sec", "topic", "x", "y", "yaw_or_offset", "state_or_confidence"])

        # 订阅 /perception/cable_offset 话题，记录偏移与置信度
        self.create_subscription(Float32MultiArray, "/perception/cable_offset", self.on_offset, 10)

        # 订阅 /odom/fused 话题，记录融合里程计
        self.create_subscription(Odometry, "/odom/fused", self.on_odom, 20)

        # 订阅 /inspection/state 话题，记录巡检状态
        self.create_subscription(String, "/inspection/state", self.on_state, 10)

        # 订阅 /perception/debug_image 话题，保存调试图像
        # 队列大小为 5，因为图像数据量大，保存频率低
        self.create_subscription(Image, "/perception/debug_image", self.on_debug_image, 5)

        # 初始化图像计数器，用于抽样保存
        self.image_count = 0

    def stamp(self) -> float:
        """获取当前 ROS 时间并转换为秒。

        Returns:
            当前时间戳，单位秒，浮点数。
        """

        # 获取当前 ROS 时间的纳秒值
        now = self.get_clock().now().nanoseconds

        # 将纳秒转换为秒
        return now / 1e9

    def on_offset(self, msg: Float32MultiArray) -> None:
        """记录视觉偏移量与置信度到 CSV。

        Args:
            msg: 包含偏移量与置信度的 Float32MultiArray 消息。
        """

        # 从消息 data 数组中提取偏移量，若数组为空则默认为 0.0
        offset = msg.data[0] if msg.data else 0.0

        # 从消息 data 数组中提取置信度，若长度不足则默认为 0.0
        conf = msg.data[2] if len(msg.data) > 2 else 0.0

        # 写入 CSV 行：topic 字段为 "offset"，x/y 为空，后两列分别保存偏移和置信度
        self.writer.writerow([self.stamp(), "offset", "", "", f"{offset:.4f}", f"{conf:.4f}"])

    def on_odom(self, msg: Odometry) -> None:
        """记录融合里程计位姿到 CSV。

        Args:
            msg: 融合后的里程计消息。
        """

        # 获取姿态四元数
        q = msg.pose.pose.orientation

        # 写入 CSV 行：topic 字段为 "odom"，保存 x、y 位置与 z/w 四元数分量
        self.writer.writerow([
            self.stamp(),
            "odom",
            f"{msg.pose.pose.position.x:.4f}",
            f"{msg.pose.pose.position.y:.4f}",
            f"{q.z:.6f}/{q.w:.6f}",
            "",
        ])

    def on_state(self, msg: String) -> None:
        """记录巡检状态到 CSV。

        Args:
            msg: 包含巡检状态字符串的消息。
        """

        # 写入 CSV 行：topic 字段为 "state"，最后一列保存状态字符串
        self.writer.writerow([self.stamp(), "state", "", "", "", msg.data])

    def on_debug_image(self, msg: Image) -> None:
        """抽样保存调试图像。

        默认每 10 帧保存 1 帧，以降低磁盘 I/O 与存储占用，
        同时保留关键可视化证据。

        Args:
            msg: 可视化调试图像消息。
        """

        # 判断是否启用图像保存
        if not bool(self.get_parameter("save_debug_images").value):
            return

        # 抽样策略：仅当 image_count 为 10 的倍数时保存
        # 对于不满足条件的帧，仅增加计数器并返回
        if self.image_count % 10 != 0:
            self.image_count += 1
            return

        # 将 ROS Image 消息转换为 OpenCV BGR 图像
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # 构造保存路径，文件名使用 6 位零填充序号
        path = self.image_dir / f"debug_{self.image_count:06d}.jpg"

        # 导入 OpenCV 并保存图像为 JPEG 格式
        import cv2

        cv2.imwrite(str(path), frame)

        # 增加图像计数器
        self.image_count += 1

    def destroy_node(self) -> bool:
        """节点销毁回调：刷新并关闭 CSV 文件，防止数据丢失。

        Returns:
            父类 destroy_node 的返回值。
        """

        # 将缓冲区数据强制写入磁盘
        self.csv_file.flush()

        # 关闭 CSV 文件
        self.csv_file.close()

        # 调用父类销毁函数，完成节点清理
        return super().destroy_node()


# 定义程序入口函数
def main() -> None:
    """节点主函数：初始化 ROS2、创建节点、进入 spin 循环、清理资源。"""

    # 初始化 ROS2 Python 客户端库
    rclpy.init()

    # 创建 DataLoggerNode 实例
    node = DataLoggerNode()

    # 进入事件循环
    rclpy.spin(node)

    # 销毁节点，会触发 destroy_node 中的文件关闭逻辑
    node.destroy_node()

    # 关闭 ROS2
    rclpy.shutdown()


# 当该脚本直接运行时，调用 main() 函数
if __name__ == "__main__":
    main()
