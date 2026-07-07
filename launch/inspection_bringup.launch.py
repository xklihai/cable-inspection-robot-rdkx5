# -*- coding: utf-8 -*-
"""线缆巡检机器人一键启动文件。

本 launch 文件用于一次性启动完整巡检闭环所需的所有 ROS2 节点：
    1. camera_node: 图像采集节点，发布 /camera/image_raw。
    2. yolo_cable_detector: 线缆/缺陷检测节点，发布偏移量与缺陷事件。
    3. odom_fusion_node: 轮速里程计与 IMU 融合节点，发布 /odom/fused 与 TF。
    4. inspection_planner_node: 速度规划节点，根据偏移与障碍生成 /cmd_vel。
    5. motor_pid_controller: 电机 PID 控制节点，下发串口指令并发布 /wheel/odom。
    6. data_logger_node: 巡检数据记录节点，保存 CSV 与调试图像。

所有节点共享 config/inspection_params.yaml 中的参数配置，
便于集中管理与调参。
"""

# 导入 LaunchDescription 类，用于描述 launch 文件内容
from launch import LaunchDescription

# 导入 Node 动作类，用于启动 ROS2 节点
from launch_ros.actions import Node

# 导入 ament_index_python 的 get_package_share_directory 函数，
# 用于获取功能包的共享目录路径
from ament_index_python.packages import get_package_share_directory

# 导入 Path 类，用于路径拼接
from pathlib import Path


def generate_launch_description():
    """构造并返回 LaunchDescription。

    该函数是 ROS2 launch 文件的入口，ROS2 会在启动时调用它。

    Returns:
        LaunchDescription 实例，包含所有待启动节点。
    """

    # 通过功能包名称获取其在 install/share 目录下的路径
    pkg = Path(get_package_share_directory("cable_inspection_robot"))

    # 拼接巡检参数文件的完整路径
    # 该 YAML 文件包含所有节点的参数配置
    params = str(pkg / "config" / "inspection_params.yaml")

    # 返回 LaunchDescription，包含 6 个核心节点
    return LaunchDescription([
        # 节点 1：摄像头采集节点
        # 负责从摄像头读取图像并发布到 /camera/image_raw
        Node(
            package="cable_inspection_robot",  # 功能包名称
            executable="camera_node",          # 可执行文件/入口点名称
            name="camera_node",                # 节点命名空间内的名称
            parameters=[params]                # 加载的参数文件路径
        ),

        # 节点 2：线缆检测节点
        # 订阅 /camera/image_raw，发布 /perception/cable_offset 等
        Node(
            package="cable_inspection_robot",
            executable="yolo_cable_detector",
            name="yolo_cable_detector",
            parameters=[params]
        ),

        # 节点 3：里程计融合节点
        # 订阅 /wheel/odom 与 /imu/data，发布 /odom/fused 与 odom->base_link TF
        Node(
            package="cable_inspection_robot",
            executable="odom_fusion_node",
            name="odom_fusion_node",
            parameters=[params]
        ),

        # 节点 4：巡检规划节点
        # 订阅 /perception/cable_offset、/scan、/odom/fused，发布 /cmd_vel
        Node(
            package="cable_inspection_robot",
            executable="inspection_planner_node",
            name="inspection_planner_node",
            parameters=[params]
        ),

        # 节点 5：电机控制节点
        # 订阅 /cmd_vel，转换为轮速并下发串口，发布 /wheel/odom
        Node(
            package="cable_inspection_robot",
            executable="motor_pid_controller",
            name="motor_pid_controller",
            parameters=[params]
        ),

        # 节点 6：数据记录节点
        # 订阅偏移、里程计、状态、调试图像，保存到本地日志目录
        Node(
            package="cable_inspection_robot",
            executable="data_logger_node",
            name="data_logger_node",
            parameters=[params]
        ),
    ])
