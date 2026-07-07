# -*- coding: utf-8 -*-
"""线缆与缺陷检测节点。

部署到 RDK X5 时，`run_inference` 可替换为 hobot_dnn/TROS 模型推理接口；
接口输出保持一致：线缆中心偏移、角度、置信度、异常类别。这样上层规划、
日志与报告中的控制闭环不受推理后端变化影响。

订阅话题：
    /camera/image_raw (sensor_msgs/Image): 原始 RGB 图像。

发布话题：
    /perception/cable_offset (std_msgs/Float32MultiArray):
        [offset_norm, angle_rad, confidence, class_id, x1, y1, x2, y2]
    /perception/defect_event (std_msgs/String): JSON 格式的缺陷事件。
    /perception/debug_image (sensor_msgs/Image): 带可视化标注的图像。
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, String


# 缺陷类别定义：class_id = 0 表示正常线缆，其余为异常类型
DEFECT_LABELS = {
    0: "normal_cable",  # 正常线缆
    1: "gap",           # 缺口/间隙
    2: "bend",          # 弯折
    3: "damage",        # 破损
}


@dataclass
class DetectionResult:
    """检测结果数据结构，用于统一视觉后处理与上层规划之间的接口。

    Attributes:
        offset_norm: 线缆中心相对于图像中心的归一化偏移，范围约 [-1, 1]。
                     正值表示线缆偏右，机器人应向右微调。
        angle_rad: 线缆在图像中的方向角，单位弧度。
        confidence: 检测置信度，范围 [0, 1]。
        class_id: 类别编号，0 正常，1 缺口，2 弯折，3 破损。
        bbox_xyxy: 检测框左上角与右下角像素坐标 (x1, y1, x2, y2)。
    """

    offset_norm: float
    angle_rad: float
    confidence: float
    class_id: int
    bbox_xyxy: tuple[int, int, int, int]


class YoloCableDetector(Node):
    """线缆检测节点：默认使用颜色+霍夫直线回退检测，RDK X5 可替换为 BPU 推理。"""

    def __init__(self) -> None:
        super().__init__("yolo_cable_detector")

        # 声明参数：置信度阈值与是否使用 TROS 后端（当前为接口预留）
        self.declare_parameter("confidence_threshold", 0.45)
        self.declare_parameter("use_tros_backend", True)

        self.bridge = CvBridge()

        # 订阅原始图像
        self.sub = self.create_subscription(Image, "/camera/image_raw", self.on_image, 10)

        # 发布检测偏移量、缺陷事件与调试图像
        self.offset_pub = self.create_publisher(Float32MultiArray, "/perception/cable_offset", 10)
        self.event_pub = self.create_publisher(String, "/perception/defect_event", 10)
        self.debug_pub = self.create_publisher(Image, "/perception/debug_image", 10)

    def on_image(self, msg: Image) -> None:
        """图像回调：执行推理并发布结果。

        Args:
            msg: ROS Image 消息。
        """
        # 将 ROS 图像消息转换为 OpenCV 的 BGR 格式
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # 执行推理；未检测到线缆时直接返回
        result = self.run_inference(frame)
        if result is None:
            return

        # 构造并发布 Float32MultiArray 类型的偏移量消息
        offset_msg = Float32MultiArray()
        offset_msg.data = [
            float(result.offset_norm),
            float(result.angle_rad),
            float(result.confidence),
            float(result.class_id),
            float(result.bbox_xyxy[0]),
            float(result.bbox_xyxy[1]),
            float(result.bbox_xyxy[2]),
            float(result.bbox_xyxy[3]),
        ]
        self.offset_pub.publish(offset_msg)

        # 如果类别不是正常线缆，则发布缺陷事件（JSON 字符串）
        if result.class_id != 0:
            event = {
                "type": DEFECT_LABELS.get(result.class_id, "unknown"),
                "confidence": round(result.confidence, 3),
                "offset_norm": round(result.offset_norm, 3),
            }
            self.event_pub.publish(String(data=json.dumps(event, ensure_ascii=False)))

        # 绘制可视化调试图像并发布
        debug = self.draw_debug(frame, result)
        debug_msg = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
        debug_msg.header = msg.header
        self.debug_pub.publish(debug_msg)

    def run_inference(self, frame: np.ndarray) -> DetectionResult | None:
        """推理入口。

        默认实现使用颜色和直线检测作为无模型回退，便于没有模型文件时复现
        控制闭环；量产/比赛环境可在此替换为 YOLOv8-RDK X5 BPU 推理结果。

        当前回退逻辑：
            1. 转换到 HSV 色彩空间，提取红色/橙色线缆区域（适应常见电缆外皮颜色）。
            2. 通过形态学开运算去噪。
            3. 使用概率霍夫直线检测提取线段，选择最长线段作为线缆。
            4. 计算线缆中心偏移、方向角与置信度。

        Args:
            frame: OpenCV BGR 图像。

        Returns:
            DetectionResult 或 None（未检测到有效线缆）。
        """

        h, w = frame.shape[:2]

        # 转换到 HSV 空间，便于按颜色提取线缆
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 红色在 HSV 中跨越 0°/180°，因此需要两个范围并取并集
        mask1 = cv2.inRange(hsv, (0, 60, 50), (12, 255, 255))
        mask2 = cv2.inRange(hsv, (168, 60, 50), (180, 255, 255))

        # 形态学开运算：去除小噪声点，同时保持主干线段
        mask = cv2.morphologyEx(mask1 | mask2, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))

        # 概率霍夫直线检测：提取二值mask中的直线段
        lines = cv2.HoughLinesP(
            mask, 1, np.pi / 180,
            threshold=35,          # 累加器阈值，越大越严格
            minLineLength=40,      # 最短线段长度（像素）
            maxLineGap=25          # 允许合并为一条线的最大间隙
        )
        if lines is None:
            return None

        # 在所有检测到的线段中选择最长的一条作为线缆候选
        best = max(lines[:, 0, :], key=lambda p: (p[2] - p[0]) ** 2 + (p[3] - p[1]) ** 2)
        x1, y1, x2, y2 = [int(v) for v in best]

        # 计算线缆中心相对于图像中心的归一化偏移
        center_x = (x1 + x2) * 0.5
        offset_norm = (center_x - w * 0.5) / max(w * 0.5, 1.0)

        # 计算线缆方向角
        angle_rad = math.atan2(y2 - y1, x2 - x1)

        # 置信度：基于颜色区域占比的启发式值，上限 0.99
        confidence = min(0.99, 0.5 + cv2.countNonZero(mask) / max(w * h, 1) * 8.0)

        # 构造对齐的边界框坐标
        x_min, x_max = sorted((x1, x2))
        y_min, y_max = sorted((y1, y2))

        # 回退检测仅用于巡线，类别固定为正常线缆（class_id=0）
        return DetectionResult(offset_norm, angle_rad, confidence, 0, (x_min, y_min, x_max, y_max))

    def draw_debug(self, frame: np.ndarray, result: DetectionResult) -> np.ndarray:
        """在原始图像上绘制检测可视化信息。

        Args:
            frame: 原始 BGR 图像。
            result: 检测结果。

        Returns:
            带标注的 BGR 图像。
        """
        out = frame.copy()
        x1, y1, x2, y2 = result.bbox_xyxy
        h, w = out.shape[:2]

        # 绘制图像中心参考线（黄色）
        cv2.line(out, (w // 2, 0), (w // 2, h), (0, 255, 255), 2)

        # 绘制检测框（绿色）
        cv2.rectangle(out, (x1, y1), (x2, y2), (0, 220, 0), 2)

        # 绘制线缆中心点（红色）
        cx = int((x1 + x2) * 0.5)
        cy = int((y1 + y2) * 0.5)
        cv2.circle(out, (cx, cy), 5, (0, 0, 255), -1)

        # 叠加文字：偏移、角度、置信度（黑色描边+白色填充，提升可读性）
        text = f"offset={result.offset_norm:+.2f} angle={result.angle_rad:+.2f} conf={result.confidence:.2f}"
        cv2.putText(out, text, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (20, 20, 20), 3)
        cv2.putText(out, text, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 1)
        return out


def main() -> None:
    rclpy.init()
    node = YoloCableDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

