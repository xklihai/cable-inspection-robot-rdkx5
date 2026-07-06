# -*- coding: utf-8 -*-
"""巡检数据记录节点。"""

from __future__ import annotations

import csv
from pathlib import Path

import rclpy
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, String


class DataLoggerNode(Node):
    def __init__(self) -> None:
        super().__init__("data_logger_node")
        self.declare_parameter("log_dir", "inspection_logs")
        self.declare_parameter("save_debug_images", True)
        self.log_dir = Path(str(self.get_parameter("log_dir").value))
        self.image_dir = self.log_dir / "images"
        self.image_dir.mkdir(parents=True, exist_ok=True)
        self.bridge = CvBridge()
        self.csv_file = (self.log_dir / "inspection.csv").open("w", newline="", encoding="utf-8")
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(["time_sec", "topic", "x", "y", "yaw_or_offset", "state_or_confidence"])
        self.create_subscription(Float32MultiArray, "/perception/cable_offset", self.on_offset, 10)
        self.create_subscription(Odometry, "/odom/fused", self.on_odom, 20)
        self.create_subscription(String, "/inspection/state", self.on_state, 10)
        self.create_subscription(Image, "/perception/debug_image", self.on_debug_image, 5)
        self.image_count = 0

    def stamp(self) -> float:
        now = self.get_clock().now().nanoseconds
        return now / 1e9

    def on_offset(self, msg: Float32MultiArray) -> None:
        offset = msg.data[0] if msg.data else 0.0
        conf = msg.data[2] if len(msg.data) > 2 else 0.0
        self.writer.writerow([self.stamp(), "offset", "", "", f"{offset:.4f}", f"{conf:.4f}"])

    def on_odom(self, msg: Odometry) -> None:
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
        self.writer.writerow([self.stamp(), "state", "", "", "", msg.data])

    def on_debug_image(self, msg: Image) -> None:
        if not bool(self.get_parameter("save_debug_images").value):
            return
        if self.image_count % 10 != 0:
            self.image_count += 1
            return
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        path = self.image_dir / f"debug_{self.image_count:06d}.jpg"
        import cv2

        cv2.imwrite(str(path), frame)
        self.image_count += 1

    def destroy_node(self) -> bool:
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

