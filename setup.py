# -*- coding: utf-8 -*-
from glob import glob
from setuptools import setup

package_name = "cable_inspection_robot"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Cable Inspection Robot Contributors",
    maintainer_email="cable-inspection-robot-contributors@example.com",
    description="RDK X5 cable inspection robot ROS2/TROS core nodes.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "camera_node = cable_inspection_robot.camera_node:main",
            "yolo_cable_detector = cable_inspection_robot.yolo_cable_detector:main",
            "odom_fusion_node = cable_inspection_robot.odom_fusion_node:main",
            "inspection_planner_node = cable_inspection_robot.inspection_planner_node:main",
            "motor_pid_controller = cable_inspection_robot.motor_pid_controller:main",
            "data_logger_node = cable_inspection_robot.data_logger_node:main",
        ],
    },
)
