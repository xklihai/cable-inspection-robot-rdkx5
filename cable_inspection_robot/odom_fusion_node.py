# -*- coding: utf-8 -*-
"""轮速里程计与 IMU 融合节点。

采用轻量互补融合：位置主要来自轮速里程计，航向使用 IMU yaw 修正漂移。
在需要更高精度时，可替换为 robot_localization EKF，外部接口保持不变。
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster


def yaw_from_quat(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quat_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class OdomFusionNode(Node):
    def __init__(self) -> None:
        super().__init__("odom_fusion_node")
        self.declare_parameter("imu_yaw_weight", 0.08)
        self.fused_pub = self.create_publisher(Odometry, "/odom/fused", 20)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.last_imu_yaw: float | None = None
        self.sub_odom = self.create_subscription(Odometry, "/wheel/odom", self.on_odom, 20)
        self.sub_imu = self.create_subscription(Imu, "/imu/data", self.on_imu, 50)

    def on_imu(self, msg: Imu) -> None:
        self.last_imu_yaw = yaw_from_quat(msg.orientation)

    def on_odom(self, msg: Odometry) -> None:
        fused = msg
        odom_yaw = yaw_from_quat(msg.pose.pose.orientation)
        if self.last_imu_yaw is not None:
            weight = float(self.get_parameter("imu_yaw_weight").value)
            yaw = self.wrap((1.0 - weight) * odom_yaw + weight * self.last_imu_yaw)
            fused.pose.pose.orientation = quat_from_yaw(yaw)
        self.fused_pub.publish(fused)
        self.publish_tf(fused)

    def publish_tf(self, odom: Odometry) -> None:
        t = TransformStamped()
        t.header = odom.header
        t.child_frame_id = odom.child_frame_id or "base_link"
        t.transform.translation.x = odom.pose.pose.position.x
        t.transform.translation.y = odom.pose.pose.position.y
        t.transform.translation.z = 0.0
        t.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

    @staticmethod
    def wrap(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))


def main() -> None:
    rclpy.init()
    node = OdomFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

