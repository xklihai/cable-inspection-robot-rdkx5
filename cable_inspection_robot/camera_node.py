# -*- coding: utf-8 -*-
"""RGB 摄像头采集节点。

本节点基于 OpenCV 的 VideoCapture 实现 RGB 图像采集，并将图像封装为
ROS2 sensor_msgs/Image 消息发布到 /camera/image_raw 话题。

在 RDK X5 上可替换为 MIPI/USB 摄像头对应的 TROS 图像发布节点；本节点
保留 OpenCV 采集路径，便于桌面开发、样机调试和无硬件场景下快速复现。

发布话题：
    /camera/image_raw (sensor_msgs/Image):
        BGR8 编码的原始图像，header 中包含时间戳与坐标系 ID。
"""

# 启用类型注解的未来导入，支持现代 Python 类型写法
from __future__ import annotations

# 导入 OpenCV 库，用于打开摄像头设备并读取图像帧
import cv2

# 导入 ROS2 Python 客户端库，用于初始化、创建节点与运行事件循环
import rclpy

# 导入 CvBridge，用于在 OpenCV 图像（numpy 数组）与 ROS Image 消息之间转换
from cv_bridge import CvBridge

# 导入 ROS2 节点基类
from rclpy.node import Node

# 导入 ROS2 标准图像消息类型
from sensor_msgs.msg import Image


# 定义摄像头采集节点类，继承自 ROS2 Node 基类
class CameraNode(Node):
    """基于 OpenCV 的 RGB 图像采集节点。

    功能：
        - 根据参数打开指定摄像头设备。
        - 以指定帧率定时读取图像帧。
        - 将 OpenCV 图像转换为 ROS Image 消息并发布。

    参数：
        device: 摄像头设备索引或路径，默认 0（系统默认摄像头）。
        fps: 目标采集帧率，默认 30.0 Hz。
        frame_id: 图像消息附带的坐标系名称，默认 "camera_rgb"。
    """

    def __init__(self) -> None:
        """节点构造函数：声明参数、创建发布者、打开摄像头并启动定时器。"""

        # 调用父类构造函数，设置节点名称为 "camera_node"
        super().__init__("camera_node")

        # 声明摄像头设备索引参数，默认值为 0
        # 在 Linux 上通常为 /dev/video0；也可传入字符串路径如 "/dev/video2"
        self.declare_parameter("device", 0)

        # 声明目标帧率参数，默认 30.0 Hz
        # 该参数决定图像发布定时器的周期
        self.declare_parameter("fps", 30.0)

        # 声明图像坐标系 ID 参数，默认 "camera_rgb"
        # 该 ID 会写入 Image 消息的 header.frame_id 字段
        self.declare_parameter("frame_id", "camera_rgb")

        # 创建图像发布者，话题名称为 /camera/image_raw，队列大小为 10
        # 队列大小为 10 可在短暂阻塞时缓存若干帧，避免丢帧
        self.pub = self.create_publisher(Image, "/camera/image_raw", 10)

        # 创建 CvBridge 实例，用于后续图像格式转换
        self.bridge = CvBridge()

        # 读取 device 参数的值，并尝试打开对应的摄像头设备
        device = self.get_parameter("device").value
        self.cap = cv2.VideoCapture(device)

        # 检查摄像头是否成功打开
        if not self.cap.isOpened():
            # 如果打开失败，记录警告日志但保持节点运行
            # 这样下游节点可以等待外部图像源（如 TROS 摄像头节点）发布图像
            self.get_logger().warning("摄像头未打开，节点将保持运行等待外部图像源")

        # 读取 fps 参数并转换为浮点数
        fps = float(self.get_parameter("fps").value)

        # 创建定时器，周期为 1/fps 秒
        # 使用 max(fps, 1.0) 防止 fps 设置过小导致周期过大或除零
        self.timer = self.create_timer(1.0 / max(fps, 1.0), self.tick)

    def tick(self) -> None:
        """定时回调函数：读取一帧图像并发布为 ROS Image 消息。

        该函数由 create_timer 按固定周期调用，负责执行实际的图像采集与发布。
        """

        # 从摄像头读取一帧图像
        # ok 为布尔值，表示读取是否成功；frame 为读取到的 BGR 图像矩阵
        ok, frame = self.cap.read()

        # 如果读取失败，直接返回，不发布空图像
        if not ok:
            return

        # 使用 CvBridge 将 OpenCV 的 BGR 图像转换为 ROS Image 消息
        # encoding="bgr8" 表示使用 8 位 BGR 编码
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")

        # 设置消息的时间戳为当前 ROS 时间
        msg.header.stamp = self.get_clock().now().to_msg()

        # 设置消息的坐标系 ID，从参数中读取并转换为字符串
        msg.header.frame_id = str(self.get_parameter("frame_id").value)

        # 发布图像消息到 /camera/image_raw 话题
        self.pub.publish(msg)


# 定义程序入口函数
def main() -> None:
    """节点主函数：初始化 ROS2、创建节点、进入 spin 循环、清理资源。"""

    # 初始化 ROS2 Python 客户端库
    rclpy.init()

    # 创建 CameraNode 实例
    node = CameraNode()

    # 进入事件循环，阻塞直到节点被销毁
    # spin 会处理订阅、发布、定时器、服务等各种回调
    rclpy.spin(node)

    # 销毁节点，释放资源
    node.destroy_node()

    # 关闭 ROS2 Python 客户端库
    rclpy.shutdown()


# 当该脚本直接运行时，调用 main() 函数
if __name__ == "__main__":
    main()
