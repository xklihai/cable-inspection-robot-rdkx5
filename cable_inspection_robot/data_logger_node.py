# -*- coding: utf-8 -*-
"""巡检数据记录节点。

将巡检过程中的关键数据持久化到本地，便于后续复现、分析与生成报告。

订阅话题：
    /perception/cable_offset (std_msgs/Float32MultiArray): 线缆偏移与置信度。
    /odom/fused (nav_msgs/Odometry): 融合里程计。
    /inspection/state (std_msgs/String): 巡检状态。
    /perception/debug_image (sensor_msgs/Image): 可视化调试图像。

输出文件：
    <log_dir>/inspection.csv: 时间戳、话题类型、位姿/偏移/状态等字段。
    <log_dir>/images/debug_*.jpg: 按 1/10 采样保存的调试图像。
"""

from __future__ import annotations

import csv
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, String


class DataLoggerNode(Node):
    """巡检数据记录节点：CSV 日志 + 调试图像抽样保存。"""

    def __init__(self) -> None:
        super().__init__("data_logger_node")

        # 日志目录与是否保存调试图像
        self.declare_parameter("log_dir", "inspection_logs")
        self.declare_parameter("save_debug_images", True)

        self.log_dir = Path(str(self.get_parameter("log_dir").value))
        self.image_dir = self.log_dir / "images"
        # 自动创建日志目录与图像子目录
        self.image_dir.mkdir(parents=True, exist_ok=True)

        self.bridge = CvBridge()

        # 创建 CSV 文件并写入表头
        self.csv_file = (self.log_dir / "inspection.csv").open("w", newline="", encoding="utf-8")
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(["time_sec", "topic", "x", "y", "yaw_or_offset", "state_or_confidence"])

        # 订阅需要记录的话题
        self.create_subscription(Float32MultiArray, "/perception/cable_offset", self.on_offset, 10)
        self.create_subscription(Odometry, "/odom/fused", self.on_odom, 20)
        self.create_subscription(String, "/inspection/state", self.on_state, 10)
        self.create_subscription(Image, "/perception/debug_image", self.on_debug_image, 5)

        self.image_count = 0  # 图像计数器，用于抽样保存

    def stamp(self) -> float:
        """获取当前 ROS 时间，转换为秒。"""
        now = self.get_clock().now().nanoseconds
        return now / 1e9

    def on_offset(self, msg: Float32MultiArray) -> None:
        """记录视觉偏移量与置信度。"""
        offset = msg.data[0] if msg.data else 0.0
        conf = msg.data[2] if len(msg.data) > 2 else 0.0
        self.writer.writerow([self.stamp(), "offset", "", "", f"{offset:.4f}", f"{conf:.4f}"])

    def on_odom(self, msg: Odometry) -> None:
        """记录融合里程计位姿（x, y, yaw 以 z/w 四元数分量形式保存）。"""
        q = msg.pose.pose.orientation
        self.writer.writerow([
            self.stamp(),
            "odom",
            f"{msg.pose.pose.position.x:.4f}",
            f"{msg.pose.pose.position.y:.4f}",
            f"{q.z:.6f}/{q.w:.6f}",
            "",
        ])

    def on_state(self, msg: String) -> None:
        """记录巡检状态字符串。"""
        self.writer.writerow([self.stamp(), "state", "", "", "", msg.data])

    def on_debug_image(self, msg: Image) -> None:
        """抽样保存调试图像，默认每 10 帧保存 1 帧。

        抽样策略可降低磁盘 I/O 与存储占用，同时保留关键可视化证据。
        """
        if not bool(self.get_parameter("save_debug_images").value):
            return
        if self.image_count % 10 != 0:
            self.image_count += 1
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        path = self.image_dir / f"debug_{self.image_count:06d}.jpg"
        cv2.imwrite(str(path), frame)
        self.image_count += 1

    def destroy_node(self) -> bool:
        """节点销毁前刷新并关闭 CSV 文件，防止数据丢失。"""
        self.csv_file.flush()
        self.csv_file.close()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = DataLoggerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

