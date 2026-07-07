# -*- coding: utf-8 -*-
"""线缆与缺陷检测节点。

本节点负责从 /camera/image_raw 订阅 RGB 图像，执行线缆检测与缺陷识别，
并发布线缆偏移量、缺陷事件和可视化调试图像。

部署到 RDK X5 时，`run_inference` 可替换为 hobot_dnn/TROS 模型推理接口；
接口输出保持一致：线缆中心偏移、角度、置信度、异常类别。这样上层规划、
日志与报告中的控制闭环不受推理后端变化影响。

当前默认实现为无模型回退方案：
    使用 HSV 颜色空间提取线缆颜色区域，再通过概率霍夫直线检测（HoughLinesP）
    提取线缆中心线，计算归一化偏移与方向角。该方案便于在没有训练模型或
    BPU 环境时快速复现控制闭环。

订阅话题：
    /camera/image_raw (sensor_msgs/Image): 原始 RGB 图像。

发布话题：
    /perception/cable_offset (std_msgs/Float32MultiArray):
        检测输出数组，顺序为：
        [offset_norm, angle_rad, confidence, class_id, x1, y1, x2, y2]
    /perception/defect_event (std_msgs/String):
        JSON 格式的缺陷事件，包含 type、confidence、offset_norm 字段。
    /perception/debug_image (sensor_msgs/Image):
        带可视化标注的检测调试图像。
"""

# 启用类型注解的未来导入
from __future__ import annotations

# 导入 json 模块，用于将缺陷事件字典序列化为 JSON 字符串
import json

# 导入 math 模块，用于计算方向角 atan2
import math

# 从 dataclasses 导入 dataclass 装饰器，用于定义检测结果数据结构
from dataclasses import dataclass

# 导入 OpenCV 库，用于图像处理与可视化
import cv2

# 导入 NumPy 库，用于图像数组操作
import numpy as np

# 导入 ROS2 Python 客户端库
import rclpy

# 导入 CvBridge，用于 ROS Image 与 OpenCV 图像之间的转换
from cv_bridge import CvBridge

# 导入 ROS2 节点基类
from rclpy.node import Node

# 导入 ROS2 图像消息类型
from sensor_msgs.msg import Image

# 导入 ROS2 标准消息类型：Float32MultiArray 用于偏移量，String 用于事件
from std_msgs.msg import Float32MultiArray, String


# 定义缺陷类别标签映射字典
# class_id = 0 表示正常线缆，1-3 分别表示不同类型缺陷
DEFECT_LABELS = {
    0: "normal_cable",  # 正常线缆
    1: "gap",           # 缺口或间隙
    2: "bend",          # 弯折
    3: "damage",        # 破损
}


# 使用 @dataclass 定义检测结果数据结构
@dataclass
class DetectionResult:
    """检测结果数据结构，统一视觉后处理与上层规划之间的接口。

    该结构将作为 run_inference() 的返回值，供 on_image() 打包成 ROS 消息。

    Attributes:
        offset_norm: 线缆中心相对于图像中心的归一化水平偏移。
                     范围约 [-1, 1]，正值表示线缆偏向图像右侧，
                     负值表示偏向左侧，0 表示居中。
        angle_rad: 线缆在图像中的方向角，单位弧度。
                   通过线段两个端点计算得到，反映线缆倾斜程度。
        confidence: 检测置信度，范围 [0, 1]。
                    值越大表示检测结果越可靠。
        class_id: 类别编号。
                  0 = 正常线缆，1 = 缺口，2 = 弯折，3 = 破损。
        bbox_xyxy: 检测框左上角与右下角像素坐标，格式为 (x1, y1, x2, y2)。
                   在回退检测中由线段端点构造。
    """

    offset_norm: float      # 归一化水平偏移量
    angle_rad: float        # 线缆方向角（弧度）
    confidence: float       # 检测置信度
    class_id: int           # 缺陷类别编号
    bbox_xyxy: tuple[int, int, int, int]  # 检测框坐标


# 定义 YOLO 线缆检测节点类，继承自 ROS2 Node 基类
class YoloCableDetector(Node):
    """线缆检测节点：默认使用颜色+霍夫直线回退检测，RDK X5 可替换为 BPU 推理。"""

    def __init__(self) -> None:
        """节点构造函数：声明参数、创建 CvBridge、订阅图像、创建发布者。"""

        # 调用父类构造函数，设置节点名称为 "yolo_cable_detector"
        super().__init__("yolo_cable_detector")

        # 声明置信度阈值参数，默认 0.45
        # 当前回退实现尚未使用该阈值，但保留以兼容后续模型推理接口
        self.declare_parameter("confidence_threshold", 0.45)

        # 声明是否使用 TROS 后端参数，默认 True
        # 当前为接口预留参数，实际推理后端替换时可通过该参数切换
        self.declare_parameter("use_tros_backend", True)

        # 创建 CvBridge 实例，用于 ROS Image 与 OpenCV 图像互转
        self.bridge = CvBridge()

        # 订阅 /camera/image_raw 话题，接收原始 RGB 图像
        # 队列大小为 10，可缓冲一定图像帧以应对处理波动
        self.sub = self.create_subscription(Image, "/camera/image_raw", self.on_image, 10)

        # 创建 /perception/cable_offset 发布者，发布 Float32MultiArray 类型的偏移量消息
        self.offset_pub = self.create_publisher(Float32MultiArray, "/perception/cable_offset", 10)

        # 创建 /perception/defect_event 发布者，发布 JSON 字符串类型缺陷事件
        self.event_pub = self.create_publisher(String, "/perception/defect_event", 10)

        # 创建 /perception/debug_image 发布者，发布可视化调试图像
        self.debug_pub = self.create_publisher(Image, "/perception/debug_image", 10)

    def on_image(self, msg: Image) -> None:
        """图像回调函数：执行推理并发布结果。

        处理流程：
            1. 将 ROS Image 消息转换为 OpenCV BGR 图像。
            2. 调用 run_inference() 进行线缆检测。
            3. 若检测到线缆，发布偏移量消息。
            4. 若检测到缺陷，发布缺陷事件消息。
            5. 绘制并发布调试图像。

        Args:
            msg: 从 /camera/image_raw 接收到的 ROS Image 消息。
        """

        # 将 ROS Image 消息转换为 OpenCV 的 BGR 格式 numpy 数组
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # 调用推理入口函数，获取检测结果
        result = self.run_inference(frame)

        # 如果未检测到有效线缆，直接返回，不发布任何消息
        if result is None:
            return

        # 创建 Float32MultiArray 消息，用于封装检测输出
        offset_msg = Float32MultiArray()

        # 将 DetectionResult 的各字段按固定顺序填充到 data 数组中
        # 顺序：offset_norm, angle_rad, confidence, class_id, x1, y1, x2, y2
        offset_msg.data = [
            float(result.offset_norm),      # 索引 0：归一化偏移
            float(result.angle_rad),        # 索引 1：方向角
            float(result.confidence),       # 索引 2：置信度
            float(result.class_id),         # 索引 3：类别编号
            float(result.bbox_xyxy[0]),     # 索引 4：检测框 x1
            float(result.bbox_xyxy[1]),     # 索引 5：检测框 y1
            float(result.bbox_xyxy[2]),     # 索引 6：检测框 x2
            float(result.bbox_xyxy[3]),     # 索引 7：检测框 y2
        ]

        # 发布线缆偏移量消息
        self.offset_pub.publish(offset_msg)

        # 如果类别编号不是 0（正常线缆），则认为是缺陷事件
        if result.class_id != 0:
            # 构造缺陷事件字典
            event = {
                # 根据 class_id 查询缺陷类型名称，若不存在则标记为 unknown
                "type": DEFECT_LABELS.get(result.class_id, "unknown"),
                # 置信度保留 3 位小数
                "confidence": round(result.confidence, 3),
                # 偏移量保留 3 位小数
                "offset_norm": round(result.offset_norm, 3),
            }

            # 将字典序列化为 JSON 字符串，并发布到 /perception/defect_event
            # ensure_ascii=False 保证中文字符正常显示
            self.event_pub.publish(String(data=json.dumps(event, ensure_ascii=False)))

        # 调用 draw_debug 在原始图像上绘制检测框、中心点与文字说明
        debug = self.draw_debug(frame, result)

        # 将调试图像转换回 ROS Image 消息
        debug_msg = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")

        # 复用原始图像消息的时间戳与坐标系信息，保证时间同步
        debug_msg.header = msg.header

        # 发布调试图像
        self.debug_pub.publish(debug_msg)

    def run_inference(self, frame: np.ndarray) -> DetectionResult | None:
        """推理入口函数。

        默认实现使用颜色和直线检测作为无模型回退，便于没有模型文件时复现
        控制闭环；量产/比赛环境可在此替换为 YOLOv8-RDK X5 BPU 推理结果。

        当前回退逻辑步骤：
            1. 获取图像高宽。
            2. 转换到 HSV 色彩空间。
            3. 提取红色/橙色线缆区域（红色在 HSV 中跨越 0°/180°，需要两个范围）。
            4. 通过形态学开运算去除小噪声点。
            5. 使用概率霍夫直线检测提取线段。
            6. 选择最长线段作为线缆候选。
            7. 计算线缆中心偏移、方向角与置信度。

        Args:
            frame: OpenCV BGR 格式输入图像，形状为 (H, W, 3)。

        Returns:
            DetectionResult 实例，若未检测到有效线缆则返回 None。
        """

        # 获取图像的高度 h 和宽度 w，用于后续坐标归一化
        h, w = frame.shape[:2]

        # 将 BGR 图像转换到 HSV 色彩空间
        # HSV 比 BGR 更适合基于颜色的分割，因为颜色信息集中在 H 通道
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 提取红色/橙色区域的第一个 HSV 范围
        # H 范围 0-12 度，S 和 V 设置较低阈值以排除过暗或过浅区域
        mask1 = cv2.inRange(hsv, (0, 60, 50), (12, 255, 255))

        # 提取红色/橙色区域的第二个 HSV 范围
        # 红色在 HSV 色轮中跨越 0°，因此需要同时包含 168-180 度范围
        mask2 = cv2.inRange(hsv, (168, 60, 50), (180, 255, 255))

        # 将两个掩码合并，并对合并结果做形态学开运算
        # 开运算：先腐蚀后膨胀，可去除小的孤立噪声点，同时保持主干区域
        mask = cv2.morphologyEx(mask1 | mask2, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))

        # 使用概率霍夫直线检测（HoughLinesP）从二值掩码中提取线段
        # 参数说明：
        #   rho=1: 像素级距离分辨率
        #   theta=π/180: 角度分辨率 1 度
        #   threshold=35: 累加器阈值，仅保留支持点数足够的直线
        #   minLineLength=40: 最小线段长度 40 像素
        #   maxLineGap=25: 允许连接为一条线的最大间隙 25 像素
        lines = cv2.HoughLinesP(
            mask, 1, np.pi / 180, threshold=35, minLineLength=40, maxLineGap=25
        )

        # 如果未检测到任何线段，返回 None
        if lines is None:
            return None

        # 在所有检测到的线段中选择最长的一条作为线缆候选
        # 使用线段端点间欧氏距离的平方作为长度度量（避免开方运算）
        best = max(lines[:, 0, :], key=lambda p: (p[2] - p[0]) ** 2 + (p[3] - p[1]) ** 2)

        # 解包最长线段的两个端点坐标
        x1, y1, x2, y2 = [int(v) for v in best]

        # 计算线缆中心点的 x 坐标
        center_x = (x1 + x2) * 0.5

        # 计算线缆中心相对于图像中心的归一化偏移
        # 分母使用 max(w * 0.5, 1.0) 防止宽度为 0 时除零
        offset_norm = (center_x - w * 0.5) / max(w * 0.5, 1.0)

        # 计算线缆方向角，使用 atan2 保证全象限正确
        angle_rad = math.atan2(y2 - y1, x2 - x1)

        # 计算检测置信度：基于颜色区域占比的启发式值
        # 颜色区域占比越大，认为检测越可靠；基准 0.5，上限 0.99
        confidence = min(0.99, 0.5 + cv2.countNonZero(mask) / max(w * h, 1) * 8.0)

        # 构造对齐的边界框坐标
        # x_min/x_max 为 x1 和 x2 中的较小/较大值，y 方向同理
        x_min, x_max = sorted((x1, x2))
        y_min, y_max = sorted((y1, y2))

        # 返回检测结果，class_id 固定为 0（正常线缆）
        # 因为回退检测不具备缺陷分类能力
        return DetectionResult(offset_norm, angle_rad, confidence, 0, (x_min, y_min, x_max, y_max))

    def draw_debug(self, frame: np.ndarray, result: DetectionResult) -> np.ndarray:
        """在原始图像上绘制检测可视化信息。

        绘制内容：
            - 图像中心垂直参考线（黄色）。
            - 线缆检测框（绿色矩形）。
            - 线缆中心点（红色圆点）。
            - 偏移、角度、置信度文字说明。

        Args:
            frame: 原始 BGR 图像。
            result: 检测结果，包含 bbox 与各项数值。

        Returns:
            带标注的 BGR 图像。
        """

        # 复制原始图像，避免直接修改输入图像
        out = frame.copy()

        # 解包检测框坐标
        x1, y1, x2, y2 = result.bbox_xyxy

        # 获取输出图像的高度和宽度
        h, w = out.shape[:2]

        # 绘制图像中心垂直参考线，用于直观判断线缆是否居中
        cv2.line(out, (w // 2, 0), (w // 2, h), (0, 255, 255), 2)

        # 绘制绿色检测框
        cv2.rectangle(out, (x1, y1), (x2, y2), (0, 220, 0), 2)

        # 计算检测框中心点坐标
        cx = int((x1 + x2) * 0.5)
        cy = int((y1 + y2) * 0.5)

        # 绘制红色中心点
        cv2.circle(out, (cx, cy), 5, (0, 0, 255), -1)

        # 构造显示文字，包含偏移、角度、置信度
        text = f"offset={result.offset_norm:+.2f} angle={result.angle_rad:+.2f} conf={result.confidence:.2f}"

        # 先绘制黑色描边文字，增强可读性
        cv2.putText(out, text, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (20, 20, 20), 3)

        # 再绘制白色填充文字
        cv2.putText(out, text, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 1)

        # 返回标注后的图像
        return out


# 定义程序入口函数
def main() -> None:
    """节点主函数：初始化 ROS2、创建节点、进入 spin 循环、清理资源。"""

    # 初始化 ROS2 Python 客户端库
    rclpy.init()

    # 创建 YoloCableDetector 实例
    node = YoloCableDetector()

    # 进入事件循环
    rclpy.spin(node)

    # 销毁节点
    node.destroy_node()

    # 关闭 ROS2
    rclpy.shutdown()


# 当该脚本直接运行时，调用 main() 函数
if __name__ == "__main__":
    main()
