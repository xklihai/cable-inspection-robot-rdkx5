# -*- coding: utf-8 -*-
"""线缆巡检机器人 ROS2/TROS 核心节点包。

本包包含基于 RDK X5 的线缆巡检机器人所需的核心 ROS2 节点，
覆盖感知、定位、规划、控制与数据记录等完整闭环：

    - camera_node: RGB 图像采集
    - yolo_cable_detector: 线缆与缺陷检测
    - odom_fusion_node: 轮速与 IMU 融合定位
    - inspection_planner_node: 速度规划与避障
    - motor_pid_controller: 电机 PID 控制与里程计发布
    - data_logger_node: 巡检数据记录

此外，本包还包含可复用的工具模块：

    - pid: 带限幅的离散 PID 控制器
    - kinematics: 差分轮底盘运动学
    - serial_protocol: WHEELTEC C30D 串口通信协议
"""
