# NodeHub 项目发布文案草稿

以下内容可用于地瓜机器人开发者社区 NodeHub 项目页。发布前请把仓库地址、演示视频地址、报告地址替换为实际链接。NodeHub 会抓取仓库根目录的 `README_cn.md` 作为中文页面内容，抓取 `README.MD` 作为英文页面内容。

## 项目标题

基于 RDK X5 的线缆巡检机器人

## 一句话简介

基于 RDK X5/ROS2/TROS 的差分轮线缆巡检机器人，支持视觉沿线、缺陷检测、LiDAR 避障、里程计融合、建图导航和巡检日志记录。

## 项目简介

本项目面向嵌入式与芯片设计大赛地瓜机器人赛题，构建了一套可复现的线缆巡检机器人方案。系统以 RDK X5 为主计算平台，运行 ROS2/TROS 感知、规划和日志节点；WHEELTEC C30D ROS 四驱主控板承担底盘电机闭环和差分轮执行；RGB 摄像头用于实时检测线缆偏移和缺口、弯折、破损等异常；可选 LiDAR、IMU 与轮速里程计用于避障、建图和定位。

软件闭环为：摄像头图像输入 -> 线缆/缺陷检测 -> 偏移量与障碍信息融合 -> `/cmd_vel` 速度规划 -> C30D 差分轮执行 -> 里程计与日志反馈。项目提供无硬件回退路径、PID 单元测试、串口协议封装、Cartographer/Nav2 参考配置和完整部署说明，便于社区用户在 RDK X5 或 Ubuntu ROS2 环境复现。

## 技术标签

RDK X5、ROS2、TROS、YOLOv8、LiDAR、Cartographer、Nav2、差分轮底盘、WHEELTEC C30D、线缆巡检、机器人视觉、BPU 推理

## 硬件平台

- RDK X5 主计算平台
- WHEELTEC C30D ROS 四驱智能小车主控板
- Raspberry Pi 4 Model B 规格板位/安装孔位
- RGB 摄像头
- 可选 LSLiDAR N10 或其他 2D LiDAR
- 可选 IMU 与轮速里程计

## 开源仓库

仓库地址：`<your-repository-url>`

## 演示视频

视频地址：`<your-video-url>`

## 设计报告

报告地址：`<your-report-url>`

## 复现步骤摘要

```bash
mkdir -p ~/rdkx5_ws/src
cd ~/rdkx5_ws/src
git clone <your-repository-url> cable-inspection-robot-rdkx5
cd ~/rdkx5_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select cable_inspection_robot
source install/setup.bash
ros2 launch cable_inspection_robot inspection_bringup.launch.py
```

## 目录说明

- `cable_inspection_robot/`：ROS2 核心节点源码。
- `config/`：巡检、Cartographer、Nav2 参数。
- `launch/`：核心系统启动文件。
- `tests/`：UTF-8 编码检查、PID 与串口协议测试。
- `docs/`：部署说明、NodeHub 发布文案和项目图片。

## 许可证

Apache License 2.0

## 推荐封面图

`docs/images/nodehub_banner.png`

## 推荐项目截图

- `docs/images/system_architecture.png`
- `docs/images/control_loop.png`
- `docs/images/hardware_stack.jpg`
- `docs/images/nodehub_banner.png`
