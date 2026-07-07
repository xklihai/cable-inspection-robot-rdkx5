# RDK X5 部署说明

本文档说明如何在 RDK X5/ROS2/TROS 环境部署线缆巡检机器人核心代码。若只做算法接口验证，也可以在 Ubuntu ROS2 环境运行无硬件测试。

## 1. 硬件清单

| 模块 | 作用 | 说明 |
|---|---|---|
| RDK X5 | 主计算平台 | 运行 ROS2/TROS、视觉推理、规划控制和日志记录 |
| WHEELTEC C30D ROS 四驱主控板 | 底盘控制 | 接收左右轮速度目标，完成电机闭环和底层实时控制 |
| Raspberry Pi 4 Model B 规格板位 | 结构与生态兼容 | RDK X5 可复用树莓派 4B 规格安装孔位和 40-pin 生态 |
| RGB 摄像头 | 线缆检测 | USB/MIPI 摄像头均可，需发布或转换为 `/camera/image_raw` |
| LiDAR，可选 | 避障与建图 | 示例参数按 `/scan` 输入设计，可接入 LSLiDAR N10 等 2D 激光雷达 |
| IMU，可选 | 航向辅助 | 发布 `/imu/data` 后用于轻量融合或替换为 robot_localization EKF |

## 2. 软件环境

- Ubuntu 22.04 或 RDK X5 官方系统镜像。
- ROS2 Humble 或与 TROS 适配的 ROS2 发行版。
- Python 3.10+。
- 常用 ROS 包：`rclpy`、`sensor_msgs`、`geometry_msgs`、`nav_msgs`、`std_msgs`、`cv_bridge`、`tf2_ros`。
- 可选组件：TROS/hobot_dnn、Cartographer、Nav2、RViz2、LiDAR 驱动。

## 3. 构建

```bash
mkdir -p ~/rdkx5_ws/src
cd ~/rdkx5_ws/src
git clone https://github.com/xklihai/cable-inspection-robot-rdkx5.git cable-inspection-robot-rdkx5
cd ~/rdkx5_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select cable_inspection_robot
source install/setup.bash
```

## 4. 参数配置

核心参数位于 `config/inspection_params.yaml`。

### 4.1 摄像头

```yaml
camera_node:
  ros__parameters:
    device: 0
    fps: 30.0
    frame_id: camera_rgb
```

若使用 TROS 摄像头节点，可关闭本仓库的 `camera_node`，直接保证图像话题为 `/camera/image_raw`。

### 4.2 C30D 串口

```yaml
motor_pid_controller:
  ros__parameters:
    serial.enabled: true
    serial.port: /dev/ttyUSB0
    serial.baudrate: 115200
```

首次调试建议保持 `serial.enabled: false`，确认 `/cmd_vel`、`/motor/debug` 和 `/wheel/odom` 正常后再写入串口。

### 4.3 巡检速度

低速巡检更利于视觉识别和安全避障。实车标定时建议从较小速度开始：

```yaml
inspection_planner_node:
  ros__parameters:
    base_speed: 0.10
    max_speed: 0.20
    max_angular: 0.50
```

## 5. 启动

```bash
ros2 launch cable_inspection_robot inspection_bringup.launch.py
```

常用检查命令：

```bash
ros2 node list
ros2 topic echo /perception/cable_offset
ros2 topic echo /inspection/state
ros2 topic echo /cmd_vel
ros2 topic echo /wheel/odom
```

## 6. RDK X5 BPU 推理替换点

`cable_inspection_robot/yolo_cable_detector.py` 中的 `run_inference()` 是唯一需要替换的推理入口。替换时保持 `DetectionResult` 字段不变，上层规划和日志节点即可复用。

推荐输出语义：

- `class_id = 0`：正常线缆。
- `class_id = 1`：缺口。
- `class_id = 2`：弯折。
- `class_id = 3`：破损。

如果数据集使用 `break`、`thunderbolt` 等类别，可在模型后处理阶段映射到上述语义；不参与报警的类别应在事件发布前过滤。

## 7. 建图与导航

本仓库提供 `config/cartographer_2d.lua` 和 `config/nav2_params.yaml` 作为参考。实际使用时建议：

1. 启动 LiDAR 驱动，确认 `/scan` 正常。
2. 启动底盘里程计和 IMU，确认 `/odom/fused`、`odom -> base_link` TF 正常。
3. 启动 Cartographer 建图，使用 RViz2 查看 `/map`、`/scan`、`/tf`。
4. 在稳定地图上接入 Nav2，保留 `/cmd_vel` 仲裁策略，避免导航指令和巡线指令冲突。

## 8. 数据记录

`data_logger_node` 默认写入 `inspection_logs/`：

- `inspection.csv`：时间、话题、位置、偏移量、状态。
- `images/debug_*.jpg`：抽样保存的检测调试图。

提交复现实验时建议同时保存 ROS bag、关键帧截图和参数文件。
