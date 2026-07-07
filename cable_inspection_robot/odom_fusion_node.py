# -*- coding: utf-8 -*-
"""轮速里程计与 IMU 融合节点。

采用轻量互补融合：位置主要来自轮速里程计，航向使用 IMU yaw 修正漂移。
在需要更高精度时，可替换为 robot_localization EKF，外部接口保持不变。

订阅话题：
    /wheel/odom (nav_msgs/Odometry): 轮速里程计。
    /imu/data (sensor_msgs/Imu): IMU 数据，主要提取航向角。

发布话题：
    /odom/fused (nav_msgs/Odometry): 融合后的里程计。

发布 TF：
    odom -> base_link: 机器人相对于里程计坐标系的位姿。
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
    """从四元数中提取绕 z 轴的航向角（yaw）。

    使用标准公式：
        sin(yaw) = 2 * (w*z + x*y)
        cos(yaw) = 1 - 2 * (y^2 + z^2)

    Args:
        q: 输入四元数。

    Returns:
        yaw 角，单位弧度，范围 [-π, π]。
    """
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quat_from_yaw(yaw: float) -> Quaternion:
    """根据 yaw 角构造仅含 z/w 分量的简化四元数（假设 roll=pitch=0）。

    Args:
        yaw: 绕 z 轴航向角，单位弧度。

    Returns:
        对应的单位四元数。
    """
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class OdomFusionNode(Node):
    """轻量级里程计融合节点：用 IMU yaw 对轮速里程计的航向做互补校正。"""

    def __init__(self) -> None:
        super().__init__("odom_fusion_node")

        # IMU yaw 融合权重：越大越信任 IMU，越小越信任轮速里程计
        self.declare_parameter("imu_yaw_weight", 0.08)

        # 发布融合后的里程计与 TF
        self.fused_pub = self.create_publisher(Odometry, "/odom/fused", 20)
        self.tf_broadcaster = TransformBroadcaster(self)

        # 缓存最新 IMU yaw
        self.last_imu_yaw: float | None = None

        # 订阅轮速里程计与 IMU
        self.sub_odom = self.create_subscription(Odometry, "/wheel/odom", self.on_odom, 20)
        self.sub_imu = self.create_subscription(Imu, "/imu/data", self.on_imu, 50)

    def on_imu(self, msg: Imu) -> None:
        """IMU 回调：缓存最新航向角。"""
        self.last_imu_yaw = yaw_from_quat(msg.orientation)

    def on_odom(self, msg: Odometry) -> None:
        """轮速里程计回调：融合 IMU yaw 后发布 /odom/fused 与 TF。

        融合策略：
            yaw_fused = (1 - w) * yaw_odom + w * yaw_imu
        其中 w 由参数 imu_yaw_weight 控制，默认 0.08。
        """
        fused = msg
        odom_yaw = yaw_from_quat(msg.pose.pose.orientation)

        if self.last_imu_yaw is not None:
            # 读取融合权重，并做加权平均
            weight = float(self.get_parameter("imu_yaw_weight").value)
            yaw = self.wrap((1.0 - weight) * odom_yaw + weight * self.last_imu_yaw)
            fused.pose.pose.orientation = quat_from_yaw(yaw)

        # 发布融合后的里程计与坐标变换
        self.fused_pub.publish(fused)
        self.publish_tf(fused)

    def publish_tf(self, odom: Odometry) -> None:
        """发布 odom -> base_link 的 TF 变换。

        Args:
            odom: 融合后的里程计消息。
        """
        t = TransformStamped()
        t.header = odom.header
        t.child_frame_id = odom.child_frame_id or "base_link"
        t.transform.translation.x = odom.pose.pose.position.x
        t.transform.translation.y = odom.pose.pose.position.y
        t.transform.translation.z = 0.0  # 2D 运动假设
        t.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

    @staticmethod
    def wrap(angle: float) -> float:
        """将角度归一化到 [-π, π] 区间。

        Args:
            angle: 任意弧度角。

        Returns:
            归一化后的角度。
        """
        return math.atan2(math.sin(angle), math.cos(angle))


def main() -> None:
    rclpy.init()
    node = OdomFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

