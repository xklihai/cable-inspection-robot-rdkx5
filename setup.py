# -*- coding: utf-8 -*-
"""ROS2/TROS 功能包安装配置脚本。

本文件定义了 cable_inspection_robot 包的构建与安装信息，
包括 Python 包、启动文件、配置文件以及各 ROS2 节点的入口点（console scripts）。

colcon 在构建时会读取本文件，将源码安装到 ROS2 工作空间的 install 目录下，
并注册各个可执行节点。
"""

# 从 glob 模块导入 glob 函数，用于匹配 launch 和 config 目录下的文件
from glob import glob

# 从 setuptools 导入 setup 函数，用于定义 Python 包的安装配置
from setuptools import setup


# 定义功能包名称
# 该名称需要与 package.xml 中的 <name> 标签保持一致
package_name = "cable_inspection_robot"


# 调用 setup 函数配置功能包
setup(
    # 包名称
    name=package_name,

    # 版本号，遵循语义化版本规范
    version="0.1.0",

    # 需要安装的 Python 包列表
    packages=[package_name],

    # 需要安装的数据文件列表
    # ROS2 Python 包通常需要将 package.xml、launch、config 等文件
    # 安装到 share/<package_name> 目录下，以便运行时被找到
    data_files=[
        # ament 资源索引文件
        # 该文件用于 ROS2 在运行时通过 ament_index 查找功能包
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),

        # 安装 package.xml 与 README 文件到 share/<package_name> 目录
        # README 文件会被 NodeHub 等工具抓取展示
        ("share/" + package_name, ["package.xml", "README_cn.md", "README.MD"]),

        # 安装 launch 目录下的所有 .launch.py 文件
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),

        # 安装 config 目录下的所有配置文件
        ("share/" + package_name + "/config", glob("config/*")),
    ],

    # 运行时的 Python 依赖
    install_requires=["setuptools"],

    # 是否支持 zip_safe
    # 对于 ROS2 包通常设为 True，但包含数据文件时也可安全使用
    zip_safe=True,

    # 维护者信息
    maintainer="Cable Inspection Robot Contributors",
    maintainer_email="cable-inspection-robot-contributors@example.com",

    # 包的简短描述
    description="RDK X5 cable inspection robot ROS2/TROS core nodes.",

    # 开源许可证
    license="Apache-2.0",

    # 控制台脚本入口点
    # 这些条目会在安装时生成可执行脚本，ROS2 launch 通过 executable 名称调用它们
    entry_points={
        "console_scripts": [
            # camera_node 入口：调用 cable_inspection_robot.camera_node 模块的 main 函数
            "camera_node = cable_inspection_robot.camera_node:main",

            # yolo_cable_detector 入口：调用线缆检测模块的 main 函数
            "yolo_cable_detector = cable_inspection_robot.yolo_cable_detector:main",

            # odom_fusion_node 入口：调用里程计融合模块的 main 函数
            "odom_fusion_node = cable_inspection_robot.odom_fusion_node:main",

            # inspection_planner_node 入口：调用巡检规划模块的 main 函数
            "inspection_planner_node = cable_inspection_robot.inspection_planner_node:main",

            # motor_pid_controller 入口：调用电机控制模块的 main 函数
            "motor_pid_controller = cable_inspection_robot.motor_pid_controller:main",

            # data_logger_node 入口：调用数据记录模块的 main 函数
            "data_logger_node = cable_inspection_robot.data_logger_node:main",
        ],
    },
)
