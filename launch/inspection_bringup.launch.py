# -*- coding: utf-8 -*-
"""线缆巡检机器人一键启动文件。

启动完整巡检闭环所需的所有 ROS2 节点：
    1. camera_node: 图像采集。
    2. yolo_cable_detector: 线缆/缺陷检测。
    3. odom_fusion_node: 轮速里程计与 IMU 融合。
    4. inspection_planner_node: 速度规划。
    5. motor_pid_controller: 电机 PID 控制与里程计发布。
    6. data_logger_node: 巡检数据记录。

所有节点共享 config/inspection_params.yaml 中的参数配置。
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    """构造 LaunchDescription，加载参数文件并启动全部核心节点。"""

    # 获取本功能包的共享目录路径
    pkg = Path(get_package_share_directory("cable_inspection_robot"))

    # 巡检参数文件路径（YAML）
    params = str(pkg / "config" / "inspection_params.yaml")

    return LaunchDescription([
        # 摄像头采集节点：发布 /camera/image_raw
        Node(package="cable_inspection_robot", executable="camera_node", name="camera_node", parameters=[params]),

        # 线缆检测节点：订阅图像，发布偏移量与缺陷事件
        Node(package="cable_inspection_robot", executable="yolo_cable_detector", name="yolo_cable_detector", parameters=[params]),

        # 里程计融合节点：融合轮速与 IMU，发布 /odom/fused 与 TF
        Node(package="cable_inspection_robot", executable="odom_fusion_node", name="odom_fusion_node", parameters=[params]),

        # 巡检规划节点：根据偏移与障碍生成 /cmd_vel
        Node(package="cable_inspection_robot", executable="inspection_planner_node", name="inspection_planner_node", parameters=[params]),

        # 电机控制节点：将 /cmd_vel 转换为轮速并下发串口
        Node(package="cable_inspection_robot", executable="motor_pid_controller", name="motor_pid_controller", parameters=[params]),

        # 数据记录节点：保存 CSV 与调试图像
        Node(package="cable_inspection_robot", executable="data_logger_node", name="data_logger_node", parameters=[params]),
    ])

