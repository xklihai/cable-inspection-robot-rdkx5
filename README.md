# 基于 RDK X5 的线缆巡检机器人

面向“地瓜机器人 RDK X5 | 极致易用的机器人开发平台”赛道的 ROS2/TROS 开源项目。机器人采用差分轮底盘沿线巡检，通过 RGB 摄像头识别线缆中心线和缺陷候选区域，可选接入 LiDAR、IMU 与轮速里程计完成避障、建图和定位。项目代码覆盖“感知 - 定位 - 规划 - 控制 - 记录”的闭环，便于在 RDK X5 和桌面 ROS2 环境复现核心流程。

![Robot overview](docs/images/robot_overview.jpg)

## 1. 项目亮点

- RDK X5 作为主计算平台，运行 ROS2/TROS、视觉感知、规划控制和日志记录。
- 上层为 WHEELTEC C30D ROS 四驱智能小车主控板，承担电机闭环、底盘执行和低层实时任务。
- 下层为 Raspberry Pi 4 Model B 规格板位；RDK X5 与树莓派 4B 同规格安装孔位和 40-pin 生态，便于替换验证。
- 支持 RGB 摄像头线缆检测、YOLOv8 缺陷识别接口、LiDAR 避障、IMU/轮速里程计融合、Cartographer 建图、Nav2 路径规划和 Web/RViz2 可视化扩展。
- 提供无硬件回退逻辑：没有模型或串口设备时，仍可运行单元测试、串口帧封装测试和算法接口验证。

## 2. 系统架构

![System architecture](docs/images/system_architecture.png)

控制闭环如下：

![Control loop](docs/images/control_loop.png)

核心链路：

1. `/camera/image_raw` 输入 RGB 图像。
2. `yolo_cable_detector` 输出 `/perception/cable_offset` 和 `/perception/debug_image`。
3. `inspection_planner_node` 融合视觉偏移量、LiDAR 最近障碍距离和融合里程计，发布 `/cmd_vel`。
4. `motor_pid_controller` 将 `/cmd_vel` 转换为左右轮目标速度，通过 UART/USB 下发给 C30D。
5. `data_logger_node` 记录图像、偏移量、里程计、状态和关键事件，便于复现实验。

## 3. 代码结构

```text
cable-inspection-robot-rdkx5/
├── cable_inspection_robot/
│   ├── camera_node.py              # RGB 摄像头采集节点
│   ├── yolo_cable_detector.py      # 线缆/缺陷检测与调试图发布
│   ├── inspection_planner_node.py   # 视觉偏移 + LiDAR 避障速度规划
│   ├── motor_pid_controller.py      # 差分轮 PID 与 C30D 串口下发
│   ├── odom_fusion_node.py          # 轮速里程计 + IMU 航向融合
│   ├── data_logger_node.py          # 巡检日志与关键帧记录
│   ├── serial_protocol.py           # C30D 串口短帧协议封装
│   ├── kinematics.py                # 差分轮运动学
│   └── pid.py                       # 可单元测试的 PID 控制器
├── config/
│   ├── inspection_params.yaml       # 巡检节点参数
│   ├── nav2_params.yaml             # Nav2 参考参数
│   └── cartographer_2d.lua          # Cartographer 2D 建图参考配置
├── launch/
│   └── inspection_bringup.launch.py # 一键启动核心节点
├── tests/
│   ├── check_utf8.py                # UTF-8 编码检查
│   └── test_pid.py                  # PID 与串口帧回环测试
└── docs/
    ├── DEPLOYMENT.md                # RDK X5/ROS2 部署说明
    └── NODEHUB_SUBMISSION.md        # NodeHub 发布文案草稿
```

## 4. 节点与话题

| 模块 | 文件 | 订阅 | 发布 | 作用 |
|---|---|---|---|---|
| 摄像头采集 | `camera_node.py` | - | `/camera/image_raw` | 采集 RGB 图像并发布 ROS 图像消息 |
| 线缆检测 | `yolo_cable_detector.py` | `/camera/image_raw` | `/perception/cable_offset`, `/perception/debug_image`, `/perception/defect_event` | 输出线缆偏移量、角度、置信度和缺陷事件 |
| 巡检规划 | `inspection_planner_node.py` | `/perception/cable_offset`, `/scan`, `/odom/fused` | `/cmd_vel`, `/inspection/state` | 由视觉偏移量和 LiDAR 避障信息生成差分底盘速度指令 |
| 电机控制 | `motor_pid_controller.py` | `/cmd_vel` | `/wheel/odom`, `/motor/debug` | 差分轮运动学、双轮 PID 与 C30D 串口协议 |
| 里程计融合 | `odom_fusion_node.py` | `/wheel/odom`, `/imu/data` | `/odom/fused`, `odom -> base_link` TF | 使用 IMU 航向轻量修正轮速里程计漂移 |
| 数据记录 | `data_logger_node.py` | 偏移量、里程计、状态、调试图像 | CSV/图像文件 | 记录完整巡检过程，支持复现实验和赛后分析 |

## 5. 快速运行

### 5.1 创建 ROS2 工作区

```bash
mkdir -p ~/rdkx5_ws/src
cd ~/rdkx5_ws/src
git clone <your-repository-url> cable-inspection-robot-rdkx5
cd ~/rdkx5_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select cable_inspection_robot
source install/setup.bash
```

### 5.2 无硬件测试

默认配置中 `serial.enabled: false`，不会写串口，适合桌面或开发板无底盘调试。

```bash
python3 src/cable-inspection-robot-rdkx5/tests/check_utf8.py
python3 src/cable-inspection-robot-rdkx5/tests/test_pid.py
ros2 launch cable_inspection_robot inspection_bringup.launch.py
```

### 5.3 连接 C30D 底盘

确认 C30D 串口设备后，修改 `config/inspection_params.yaml`：

```yaml
motor_pid_controller:
  ros__parameters:
    serial.enabled: true
    serial.port: /dev/ttyUSB0
    serial.baudrate: 115200
```

启动后可查看速度指令和底盘调试信息：

```bash
ros2 topic echo /cmd_vel
ros2 topic echo /motor/debug
ros2 topic echo /wheel/odom
```

## 6. 关键参数

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `camera_node.fps` | `30.0` | RGB 图像采集帧率 |
| `yolo_cable_detector.confidence_threshold` | `0.45` | 线缆/缺陷检测置信度阈值 |
| `inspection_planner_node.base_speed` | `0.22 m/s` | 常规巡检线速度 |
| `inspection_planner_node.max_angular` | `0.90 rad/s` | 最大角速度限制 |
| `inspection_planner_node.obstacle_slow_distance` | `0.75 m` | 遇障减速距离 |
| `inspection_planner_node.obstacle_stop_distance` | `0.35 m` | 遇障停转距离 |
| `motor_pid_controller.wheel_base_m` | `0.235 m` | 差分轮轮距 |
| `motor_pid_controller.serial.baudrate` | `115200` | C30D 串口波特率 |

实际比赛样机可按底盘尺寸、镜头视场、巡检线宽和地面摩擦条件重新标定。

## 7. RDK X5/TROS 适配说明

当前 `yolo_cable_detector.py` 保留了 OpenCV/Hough 回退路径，目的是在未接入模型文件时仍能复现控制链路。部署到 RDK X5 后，建议将 `run_inference()` 替换为 TROS/hobot_dnn 推理接口，保持输出结构不变：

```python
DetectionResult(
    offset_norm=...,        # 线缆中心相对图像中心的归一化偏移，范围约为 [-1, 1]
    angle_rad=...,          # 线缆方向角
    confidence=...,         # 检测置信度
    class_id=...,           # 0 normal, 1 gap, 2 bend, 3 damage
    bbox_xyxy=(x1, y1, x2, y2),
)
```

这样上层规划、日志和 Web/RViz2 可视化不需要改动。

## 8. 开源与提交

本仓库用于社区开源和复现实验，建议配套提交：

- 源码仓库地址。
- `README.md` 和 `docs/DEPLOYMENT.md`。
- 作品演示视频。
- 作品设计报告 PDF。
- 实车照片或关键截图。
- NodeHub 项目简介，见 `docs/NODEHUB_SUBMISSION.md`。

## 9. 许可证

代码采用 MIT License 开源。模型权重、第三方数据集、硬件厂商资料和比赛文档如另有授权，应遵循其原始许可证。
