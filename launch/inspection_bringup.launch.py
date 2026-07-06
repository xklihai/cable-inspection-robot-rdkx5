# -*- coding: utf-8 -*-
"""线缆巡检机器人一键启动文件。"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    pkg = Path(get_package_share_directory("cable_inspection_robot"))
    params = str(pkg / "config" / "inspection_params.yaml")
    return LaunchDescription([
        Node(package="cable_inspection_robot", executable="camera_node", name="camera_node", parameters=[params]),
        Node(package="cable_inspection_robot", executable="yolo_cable_detector", name="yolo_cable_detector", parameters=[params]),
        Node(package="cable_inspection_robot", executable="odom_fusion_node", name="odom_fusion_node", parameters=[params]),
        Node(package="cable_inspection_robot", executable="inspection_planner_node", name="inspection_planner_node", parameters=[params]),
        Node(package="cable_inspection_robot", executable="motor_pid_controller", name="motor_pid_controller", parameters=[params]),
        Node(package="cable_inspection_robot", executable="data_logger_node", name="data_logger_node", parameters=[params]),
    ])

