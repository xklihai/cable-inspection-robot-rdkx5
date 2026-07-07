# -*- coding: utf-8 -*-
"""巡检速度规划节点。

控制律：
视觉偏移量 + LiDAR 障碍距离 -> 差分轮速度指令。
视觉偏移为正表示线缆在图像右侧，机器人应向右微调；避障状态优先级高于
沿线巡检，近距离障碍会触发减速或绕行角速度。

订阅话题：
    /perception/cable_offset (std_msgs/Float32MultiArray): 视觉偏移与置信度。
    /scan (sensor_msgs/LaserScan): LiDAR 扫描数据，用于避障。
    /odom/fused (nav_msgs/Odometry): 融合里程计（当前用于状态记录）。

发布话题：
    /cmd_vel (geometry_msgs/Twist): 底盘速度指令。
    /inspection/state (std_msgs/String): 当前巡检状态。
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
    """根据视觉偏移与障碍物距离生成速度指令的规划节点。"""

    def __init__(self) -> None:
        super().__init__("inspection_planner_node")

        # 速度规划参数
        self.declare_parameter("base_speed", 0.22)             # 默认前进速度（m/s）
        self.declare_parameter("max_speed", 0.35)              # 最大前进速度（m/s）
        self.declare_parameter("max_angular", 0.9)             # 最大角速度（rad/s）
        self.declare_parameter("offset_gain", 0.85)            # 偏移量到角速度的比例增益
        self.declare_parameter("obstacle_slow_distance", 0.75) # 开始减速的障碍距离（m）
        self.declare_parameter("obstacle_stop_distance", 0.35) # 停止并转向的障碍距离（m）

        # 内部状态缓存
        self.offset_norm = 0.0       # 当前线缆归一化偏移
        self.offset_conf = 0.0       # 当前检测置信度
        self.min_obstacle = math.inf # 最近障碍物距离
        self.last_odom: Odometry | None = None

        # 发布速度指令与状态
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 20)
        self.state_pub = self.create_publisher(String, "/inspection/state", 10)

        # 订阅感知与定位数据
        self.create_subscription(Float32MultiArray, "/perception/cable_offset", self.on_offset, 10)
        self.create_subscription(LaserScan, "/scan", self.on_scan, 10)
        self.create_subscription(Odometry, "/odom/fused", self.on_odom, 20)

        # 以 20 Hz 频率执行规划
        self.timer = self.create_timer(0.05, self.plan)

    def on_offset(self, msg: Float32MultiArray) -> None:
        """视觉偏移回调：提取归一化偏移与置信度。"""
        if len(msg.data) >= 3:
            # 偏移量裁剪到 [-1, 1]，防止异常值
            self.offset_norm = max(-1.0, min(1.0, float(msg.data[0])))
            self.offset_conf = float(msg.data[2])

    def on_scan(self, msg: LaserScan) -> None:
        """LiDAR 回调：计算有效测距中的最小值作为最近障碍物距离。"""
        valid = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        self.min_obstacle = min(valid) if valid else math.inf

    def on_odom(self, msg: Odometry) -> None:
        """里程计回调：缓存最新融合里程计。"""
        self.last_odom = msg

    def plan(self) -> None:
        """规划主循环：根据障碍距离与视觉偏移生成 /cmd_vel。

        状态优先级：
            1. 紧急避障（停转）：障碍距离 < stop_distance。
            2. 减速避障：stop_distance <= 障碍距离 < slow_distance。
            3. 正常巡线：障碍距离 >= slow_distance。
            4. 低置信度：检测置信度低时降低速度，进入搜索状态。
        """

        # 读取参数（支持运行时动态重载）
        base_speed = float(self.get_parameter("base_speed").value)
        max_speed = float(self.get_parameter("max_speed").value)
        max_angular = float(self.get_parameter("max_angular").value)
        gain = float(self.get_parameter("offset_gain").value)
        slow_d = float(self.get_parameter("obstacle_slow_distance").value)
        stop_d = float(self.get_parameter("obstacle_stop_distance").value)

        cmd = Twist()
        state = "tracking"

        # 基于视觉偏移计算角速度：offset_norm > 0 表示线缆偏右，应向右转（负角速度）
        angular = -gain * self.offset_norm

        if self.min_obstacle < stop_d:
            # 障碍过近：停止前进，原地转向以脱离危险
            cmd.linear.x = 0.0
            cmd.angular.z = max_angular * (1.0 if self.offset_norm <= 0 else -1.0)
            state = "avoid_stop_turn"
        elif self.min_obstacle < slow_d:
            # 障碍进入减速区：线速度按距离线性缩放，角速度增益加大以更快回正
            scale = max(0.25, (self.min_obstacle - stop_d) / max(slow_d - stop_d, 1e-6))
            cmd.linear.x = min(max_speed, base_speed * scale)
            cmd.angular.z = max(-max_angular, min(max_angular, angular * 1.4))
            state = "avoid_slow"
        else:
            # 正常巡线
            cmd.linear.x = min(max_speed, base_speed)
            cmd.angular.z = max(-max_angular, min(max_angular, angular))

        # 低置信度时减速，给视觉检测留出更多时间
        if self.offset_conf < 0.35:
            cmd.linear.x *= 0.55
            state = "low_confidence_search"

        # 发布速度与状态
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

