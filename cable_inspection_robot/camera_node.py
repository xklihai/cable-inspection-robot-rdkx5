# -*- coding: utf-8 -*-
"""RGB 摄像头采集节点。

在 RDK X5 上可替换为 MIPI/USB 摄像头对应的 TROS 图像发布节点；本节点
保留 OpenCV 采集路径，便于桌面和样机阶段快速复现。

发布话题：
    /camera/image_raw (sensor_msgs/Image): BGR8 编码的原始图像。
"""

from __future__ import annotations

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class CameraNode(Node):
    """基于 OpenCV 的 RGB 图像采集节点。"""

    def __init__(self) -> None:
        super().__init__("camera_node")

        # 声明可调参数：设备索引、采集帧率、坐标系名称
        self.declare_parameter("device", 0)
        self.declare_parameter("fps", 30.0)
        self.declare_parameter("frame_id", "camera_rgb")

        # 创建图像发布者，下游节点（如 yolo_cable_detector）会订阅该话题
        self.pub = self.create_publisher(Image, "/camera/image_raw", 10)
        self.bridge = CvBridge()

        # 根据参数打开摄像头；如果失败则记录警告，节点仍保持运行
        device = self.get_parameter("device").value
        self.cap = cv2.VideoCapture(device)
        if not self.cap.isOpened():
            self.get_logger().warning("摄像头未打开，节点将保持运行等待外部图像源")

        # 按指定帧率创建定时器，定时读取并发布图像
        fps = float(self.get_parameter("fps").value)
        self.timer = self.create_timer(1.0 / max(fps, 1.0), self.tick)

    def tick(self) -> None:
        """定时回调：读取一帧图像并发布为 ROS Image 消息。"""
        ok, frame = self.cap.read()
        if not ok:
            # 读取失败时静默跳过，避免空帧导致下游节点异常
            return

        # 将 OpenCV 的 BGR 图像转换为 ROS Image 消息
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.get_parameter("frame_id").value)
        self.pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = CameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

