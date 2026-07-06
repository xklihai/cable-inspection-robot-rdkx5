# -*- coding: utf-8 -*-
"""线缆与缺陷检测节点。

部署到 RDK X5 时，`run_inference` 可替换为 hobot_dnn/TROS 模型推理接口；
接口输出保持一致：线缆中心偏移、角度、置信度、异常类别。这样上层规划、
日志与报告中的控制闭环不受推理后端变化影响。
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


DEFECT_LABELS = {
    0: "normal_cable",
    1: "gap",
    2: "bend",
    3: "damage",
}


@dataclass
class DetectionResult:
    offset_norm: float
    angle_rad: float
    confidence: float
    class_id: int
    bbox_xyxy: tuple[int, int, int, int]


class YoloCableDetector(Node):
    def __init__(self) -> None:
        super().__init__("yolo_cable_detector")
        self.declare_parameter("confidence_threshold", 0.45)
        self.declare_parameter("use_tros_backend", True)
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, "/camera/image_raw", self.on_image, 10)
        self.offset_pub = self.create_publisher(Float32MultiArray, "/perception/cable_offset", 10)
        self.event_pub = self.create_publisher(String, "/perception/defect_event", 10)
        self.debug_pub = self.create_publisher(Image, "/perception/debug_image", 10)

    def on_image(self, msg: Image) -> None:
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        result = self.run_inference(frame)
        if result is None:
            return

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

        if result.class_id != 0:
            event = {
                "type": DEFECT_LABELS.get(result.class_id, "unknown"),
                "confidence": round(result.confidence, 3),
                "offset_norm": round(result.offset_norm, 3),
            }
            self.event_pub.publish(String(data=json.dumps(event, ensure_ascii=False)))

        debug = self.draw_debug(frame, result)
        debug_msg = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
        debug_msg.header = msg.header
        self.debug_pub.publish(debug_msg)

    def run_inference(self, frame: np.ndarray) -> DetectionResult | None:
        """推理入口。

        默认实现使用颜色和直线检测作为无模型回退，便于没有模型文件时复现
        控制闭环；量产/比赛环境可在此替换为 YOLOv8-RDK X5 BPU 推理结果。
        """

        h, w = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, (0, 60, 50), (12, 255, 255))
        mask2 = cv2.inRange(hsv, (168, 60, 50), (180, 255, 255))
        mask = cv2.morphologyEx(mask1 | mask2, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        lines = cv2.HoughLinesP(mask, 1, np.pi / 180, threshold=35, minLineLength=40, maxLineGap=25)
        if lines is None:
            return None

        best = max(lines[:, 0, :], key=lambda p: (p[2] - p[0]) ** 2 + (p[3] - p[1]) ** 2)
        x1, y1, x2, y2 = [int(v) for v in best]
        center_x = (x1 + x2) * 0.5
        offset_norm = (center_x - w * 0.5) / max(w * 0.5, 1.0)
        angle_rad = math.atan2(y2 - y1, x2 - x1)
        confidence = min(0.99, 0.5 + cv2.countNonZero(mask) / max(w * h, 1) * 8.0)
        x_min, x_max = sorted((x1, x2))
        y_min, y_max = sorted((y1, y2))
        return DetectionResult(offset_norm, angle_rad, confidence, 0, (x_min, y_min, x_max, y_max))

    def draw_debug(self, frame: np.ndarray, result: DetectionResult) -> np.ndarray:
        out = frame.copy()
        x1, y1, x2, y2 = result.bbox_xyxy
        h, w = out.shape[:2]
        cv2.line(out, (w // 2, 0), (w // 2, h), (0, 255, 255), 2)
        cv2.rectangle(out, (x1, y1), (x2, y2), (0, 220, 0), 2)
        cx = int((x1 + x2) * 0.5)
        cv2.circle(out, (cx, int((y1 + y2) * 0.5)), 5, (0, 0, 255), -1)
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

