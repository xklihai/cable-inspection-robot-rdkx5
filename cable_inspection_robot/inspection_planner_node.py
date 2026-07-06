# -*- coding: utf-8 -*-
"""巡检速度规划节点。

控制律：
视觉偏移量 + LiDAR 障碍距离 -> 差分轮速度指令。
视觉偏移为正表示线缆在图像右侧，机器人应向右微调；避障状态优先级高于
沿线巡检，近距离障碍会触发减速或绕行角速度。
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray, String


class InspectionPlannerNode(Node):
    def __init__(self) -> None:
        super().__init__("inspection_planner_node")
        self.declare_parameter("base_speed", 0.22)
        self.declare_parameter("max_speed", 0.35)
        self.declare_parameter("max_angular", 0.9)
        self.declare_parameter("offset_gain", 0.85)
        self.declare_parameter("obstacle_slow_distance", 0.75)
        self.declare_parameter("obstacle_stop_distance", 0.35)
        self.offset_norm = 0.0
        self.offset_conf = 0.0
        self.min_obstacle = math.inf
        self.last_odom: Odometry | None = None
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 20)
        self.state_pub = self.create_publisher(String, "/inspection/state", 10)
        self.create_subscription(Float32MultiArray, "/perception/cable_offset", self.on_offset, 10)
        self.create_subscription(LaserScan, "/scan", self.on_scan, 10)
        self.create_subscription(Odometry, "/odom/fused", self.on_odom, 20)
        self.timer = self.create_timer(0.05, self.plan)

    def on_offset(self, msg: Float32MultiArray) -> None:
        if len(msg.data) >= 3:
            self.offset_norm = max(-1.0, min(1.0, float(msg.data[0])))
            self.offset_conf = float(msg.data[2])

    def on_scan(self, msg: LaserScan) -> None:
        valid = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        self.min_obstacle = min(valid) if valid else math.inf

    def on_odom(self, msg: Odometry) -> None:
        self.last_odom = msg

    def plan(self) -> None:
        base_speed = float(self.get_parameter("base_speed").value)
        max_speed = float(self.get_parameter("max_speed").value)
        max_angular = float(self.get_parameter("max_angular").value)
        gain = float(self.get_parameter("offset_gain").value)
        slow_d = float(self.get_parameter("obstacle_slow_distance").value)
        stop_d = float(self.get_parameter("obstacle_stop_distance").value)

        cmd = Twist()
        state = "tracking"
        angular = -gain * self.offset_norm

        if self.min_obstacle < stop_d:
            cmd.linear.x = 0.0
            cmd.angular.z = max_angular * (1.0 if self.offset_norm <= 0 else -1.0)
            state = "avoid_stop_turn"
        elif self.min_obstacle < slow_d:
            scale = max(0.25, (self.min_obstacle - stop_d) / max(slow_d - stop_d, 1e-6))
            cmd.linear.x = min(max_speed, base_speed * scale)
            cmd.angular.z = max(-max_angular, min(max_angular, angular * 1.4))
            state = "avoid_slow"
        else:
            cmd.linear.x = min(max_speed, base_speed)
            cmd.angular.z = max(-max_angular, min(max_angular, angular))

        if self.offset_conf < 0.35:
            cmd.linear.x *= 0.55
            state = "low_confidence_search"

        self.cmd_pub.publish(cmd)
        self.state_pub.publish(String(data=state))


def main() -> None:
    rclpy.init()
    node = InspectionPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

