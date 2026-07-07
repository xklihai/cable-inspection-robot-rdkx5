# -*- coding: utf-8 -*-
"""轮速里程计与 IMU 融合节点。

本节点采用轻量级互补融合策略：位置信息主要来自轮速里程计，
航向角（yaw）使用 IMU 的 yaw 进行轻微修正，以抑制轮速里程计的航向漂移。

在需要更高精度时，可替换为 robot_localization 包中的 EKF 扩展卡尔曼滤波器，
外部接口（/odom/fused 话题与 odom -> base_link TF）可保持不变。

订阅话题：
    /wheel/odom (nav_msgs/Odometry): 轮速里程计，提供位置与速度信息。
    /imu/data (sensor_msgs/Imu): IMU 数据，主要提供航向角。

发布话题：
    /odom/fused (nav_msgs/Odometry): 融合后的里程计。

发布 TF：
    odom -> base_link: 机器人相对于里程计坐标系的位姿变换。
"""

# 启用类型注解的未来导入
from __future__ import annotations

# 导入 math 模块，用于三角函数和角度归一化
import math

# 导入 ROS2 Python 客户端库
import rclpy

# 导入 ROS2 Quaternion 与 TransformStamped 消息类型
from geometry_msgs.msg import Quaternion, TransformStamped

# 导入 ROS2 Odometry 消息类型
from nav_msgs.msg import Odometry

# 导入 ROS2 节点基类
from rclpy.node import Node

# 导入 ROS2 IMU 消息类型
from sensor_msgs.msg import Imu

# 导入 TF2 变换广播器，用于发布 odom -> base_link 的坐标变换
from tf2_ros import TransformBroadcaster


def yaw_from_quat(q: Quaternion) -> float:
    """从四元数中提取绕 z 轴的航向角（yaw）。

    使用四元数转欧拉角的标准公式（Tait-Bryan，ZYX 顺序）：
        sin(yaw) = 2 * (w*z + x*y)
        cos(yaw) = 1 - 2 * (y^2 + z^2)
        yaw = atan2(sin(yaw), cos(yaw))

    该公式假设 roll 和 pitch 近似为 0，适用于二维平面运动的机器人。

    Args:
        q: 输入四元数，包含 x、y、z、w 四个分量。

    Returns:
        yaw 角，单位弧度，范围 [-π, π]。
    """

    # 计算 sin(yaw) 分量
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)

    # 计算 cos(yaw) 分量
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)

    # 使用 atan2 计算 yaw，保证结果在 [-π, π] 范围内
    return math.atan2(siny_cosp, cosy_cosp)


def quat_from_yaw(yaw: float) -> Quaternion:
    """根据 yaw 角构造仅含 z/w 分量的简化四元数。

    假设 roll = 0、pitch = 0，仅存在绕 z 轴的旋转。
    四元数与欧拉角的换算关系：
        z = sin(yaw / 2)
        w = cos(yaw / 2)
        x = 0
        y = 0

    Args:
        yaw: 绕 z 轴航向角，单位弧度。

    Returns:
        对应的单位四元数。
    """

    # 创建 Quaternion 消息对象
    q = Quaternion()

    # 设置 x 分量为 0（无 roll 旋转）
    q.x = 0.0

    # 设置 y 分量为 0（无 pitch 旋转）
    q.y = 0.0

    # 根据 yaw 角计算 z 分量
    q.z = math.sin(yaw * 0.5)

    # 根据 yaw 角计算 w 分量
    q.w = math.cos(yaw * 0.5)

    # 返回构造好的四元数
    return q


# 定义里程计融合节点类，继承自 ROS2 Node 基类
class OdomFusionNode(Node):
    """轻量级里程计融合节点：用 IMU yaw 对轮速里程计的航向做互补校正。"""

    def __init__(self) -> None:
        """节点构造函数：声明参数、创建发布者与 TF 广播器、订阅输入话题。"""

        # 调用父类构造函数，设置节点名称为 "odom_fusion_node"
        super().__init__("odom_fusion_node")

        # 声明 IMU yaw 融合权重参数，默认 0.08
        # 权重越大，融合结果越信任 IMU；越小越信任轮速里程计。
        # 默认值 0.08 表示轻微修正轮速漂移，不会引入过多 IMU 噪声。
        self.declare_parameter("imu_yaw_weight", 0.08)

        # 创建 /odom/fused 发布者，用于发布融合后的里程计
        self.fused_pub = self.create_publisher(Odometry, "/odom/fused", 20)

        # 创建 TF 广播器，用于发布 odom -> base_link 的坐标变换
        self.tf_broadcaster = TransformBroadcaster(self)

        # 初始化最新 IMU yaw 缓存，None 表示尚未收到 IMU 数据
        self.last_imu_yaw: float | None = None

        # 订阅 /wheel/odom 话题，接收轮速里程计
        # 队列大小 20，保证里程计数据不丢失
        self.sub_odom = self.create_subscription(Odometry, "/wheel/odom", self.on_odom, 20)

        # 订阅 /imu/data 话题，接收 IMU 数据
        # IMU 通常以较高频率发布（如 50~200 Hz），队列大小 50
        self.sub_imu = self.create_subscription(Imu, "/imu/data", self.on_imu, 50)

    def on_imu(self, msg: Imu) -> None:
        """IMU 回调函数：从四元数中提取并缓存最新航向角。

        Args:
            msg: 包含方向四元数的 IMU 消息。
        """

        # 从 IMU 方向四元数中提取 yaw 角并保存
        self.last_imu_yaw = yaw_from_quat(msg.orientation)

    def on_odom(self, msg: Odometry) -> None:
        """轮速里程计回调函数：融合 IMU yaw 后发布 /odom/fused 与 TF。

        融合策略：
            如果已收到 IMU 数据，则对轮速里程计的 yaw 做加权平均：
                yaw_fused = (1 - w) * yaw_odom + w * yaw_imu
            其中 w 由参数 imu_yaw_weight 控制。

            如果尚未收到 IMU 数据，则直接使用原始轮速里程计。

        Args:
            msg: 轮速里程计消息。
        """

        # 直接使用原始里程计消息作为融合结果的基础
        # 注意：这里赋值的是引用，由于后续只修改 orientation，不影响原始消息语义
        fused = msg

        # 从原始里程计四元数中提取 yaw 角
        odom_yaw = yaw_from_quat(msg.pose.pose.orientation)

        # 判断是否已收到有效的 IMU yaw 数据
        if self.last_imu_yaw is not None:
            # 读取融合权重参数
            weight = float(self.get_parameter("imu_yaw_weight").value)

            # 对轮速里程计 yaw 与 IMU yaw 做加权平均
            # (1 - weight) 部分来自轮速里程计，weight 部分来自 IMU
            yaw = self.wrap((1.0 - weight) * odom_yaw + weight * self.last_imu_yaw)

            # 将融合后的 yaw 角重新编码为四元数，赋值给融合结果
            fused.pose.pose.orientation = quat_from_yaw(yaw)

        # 发布融合后的里程计消息
        self.fused_pub.publish(fused)

        # 发布对应的 TF 变换
        self.publish_tf(fused)

    def publish_tf(self, odom: Odometry) -> None:
        """发布 odom -> base_link 的 TF 变换。

        Args:
            odom: 融合后的里程计消息，包含位置与姿态信息。
        """

        # 创建 TransformStamped 消息对象
        t = TransformStamped()

        # 设置 TF 头信息，与里程计消息头保持一致
        t.header = odom.header

        # 设置子坐标系 ID
        # 如果里程计消息中 child_frame_id 为空，则默认使用 "base_link"
        t.child_frame_id = odom.child_frame_id or "base_link"

        # 设置平移分量：x 和 y 来自里程计位置，z 假设为 0（二维运动）
        t.transform.translation.x = odom.pose.pose.position.x
        t.transform.translation.y = odom.pose.pose.position.y
        t.transform.translation.z = 0.0

        # 设置旋转分量：直接复用里程计姿态四元数
        t.transform.rotation = odom.pose.pose.orientation

        # 通过 TF 广播器发送变换
        self.tf_broadcaster.sendTransform(t)

    @staticmethod
    def wrap(angle: float) -> float:
        """将任意弧度角归一化到 [-π, π] 区间。

        使用 atan2(sin(angle), cos(angle)) 实现，可正确处理角度环绕问题。

        Args:
            angle: 任意弧度角。

        Returns:
            归一化后的角度，范围 [-π, π]。
        """

        # 通过三角函数将角度映射到标准区间
        return math.atan2(math.sin(angle), math.cos(angle))


# 定义程序入口函数
def main() -> None:
    """节点主函数：初始化 ROS2、创建节点、进入 spin 循环、清理资源。"""

    # 初始化 ROS2 Python 客户端库
    rclpy.init()

    # 创建 OdomFusionNode 实例
    node = OdomFusionNode()

    # 进入事件循环
    rclpy.spin(node)

    # 销毁节点
    node.destroy_node()

    # 关闭 ROS2
    rclpy.shutdown()


# 当该脚本直接运行时，调用 main() 函数
if __name__ == "__main__":
    main()
