# -*- coding: utf-8 -*-
"""巡检速度规划节点。

本节点负责根据视觉偏移量、LiDAR 障碍物距离和当前里程计状态，
生成差分轮底盘的速度指令（/cmd_vel），并发布当前巡检状态。

控制律说明：
    视觉偏移量 + LiDAR 障碍距离 -> 差分轮速度指令。
    视觉偏移为正表示线缆在图像右侧，机器人应向右微调（产生负角速度，
    因为机器人坐标系下右转对应负角速度）。
    避障状态优先级高于沿线巡检，近距离障碍会触发减速或原地转向。

订阅话题：
    /perception/cable_offset (std_msgs/Float32MultiArray):
        视觉偏移与置信度，data[0] 为 offset_norm，data[2] 为 confidence。
    /scan (sensor_msgs/LaserScan):
        LiDAR 扫描数据，用于障碍物检测与避障。
    /odom/fused (nav_msgs/Odometry):
        融合里程计，当前用于状态记录，未来可扩展为速度反馈。

发布话题：
    /cmd_vel (geometry_msgs/Twist):
        底盘速度指令，包含线速度 linear.x 和角速度 angular.z。
    /inspection/state (std_msgs/String):
        当前巡检状态字符串，例如 "tracking"、"avoid_slow" 等。
"""

# 启用类型注解的未来导入
from __future__ import annotations

# 导入 math 模块，用于使用正无穷大 math.inf 表示无障碍物
import math

# 导入 ROS2 Python 客户端库
import rclpy

# 导入 ROS2 Twist 消息类型，用于发布速度指令
from geometry_msgs.msg import Twist

# 导入 ROS2 Odometry 消息类型，用于订阅融合里程计
from nav_msgs.msg import Odometry

# 导入 ROS2 节点基类
from rclpy.node import Node

# 导入 ROS2 LaserScan 消息类型，用于订阅 LiDAR 数据
from sensor_msgs.msg import LaserScan

# 导入 ROS2 标准消息类型
from std_msgs.msg import Float32MultiArray, String


# 定义巡检规划节点类，继承自 ROS2 Node 基类
class InspectionPlannerNode(Node):
    """根据视觉偏移与障碍物距离生成速度指令的规划节点。"""

    def __init__(self) -> None:
        """节点构造函数：声明参数、初始化状态、创建发布者与订阅者、启动定时器。"""

        # 调用父类构造函数，设置节点名称为 "inspection_planner_node"
        super().__init__("inspection_planner_node")

        # 声明默认前进速度参数，单位 m/s
        # 该值决定机器人正常巡线时的基础线速度
        self.declare_parameter("base_speed", 0.22)

        # 声明最大前进速度参数，单位 m/s
        # 实际线速度不会超过该上限
        self.declare_parameter("max_speed", 0.35)

        # 声明最大角速度参数，单位 rad/s
        # 实际角速度会被限制在 [-max_angular, max_angular] 范围内
        self.declare_parameter("max_angular", 0.9)

        # 声明偏移量到角速度的比例增益
        # 增益越大，机器人对视觉偏移的修正反应越灵敏
        self.declare_parameter("offset_gain", 0.85)

        # 声明开始减速的障碍物距离阈值，单位 m
        # 当最近障碍物距离小于该值时，机器人进入减速避障状态
        self.declare_parameter("obstacle_slow_distance", 0.75)

        # 声明停止并转向的障碍物距离阈值，单位 m
        # 当最近障碍物距离小于该值时，机器人停止前进并原地转向
        self.declare_parameter("obstacle_stop_distance", 0.35)

        # 初始化线缆归一化偏移状态，默认 0 表示线缆居中
        self.offset_norm = 0.0

        # 初始化检测置信度状态，默认 0 表示尚未收到有效检测
        self.offset_conf = 0.0

        # 初始化最近障碍物距离，使用正无穷大表示当前无障碍物
        self.min_obstacle = math.inf

        # 初始化最新里程计缓存，用于记录或后续扩展控制逻辑
        self.last_odom: Odometry | None = None

        # 创建 /cmd_vel 发布者，队列大小 20
        # 较高的队列大小可保证控制指令连续性
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 20)

        # 创建 /inspection/state 发布者，队列大小 10
        self.state_pub = self.create_publisher(String, "/inspection/state", 10)

        # 订阅 /perception/cable_offset 话题，接收视觉偏移与置信度
        self.create_subscription(Float32MultiArray, "/perception/cable_offset", self.on_offset, 10)

        # 订阅 /scan 话题，接收 LiDAR 扫描数据
        self.create_subscription(LaserScan, "/scan", self.on_scan, 10)

        # 订阅 /odom/fused 话题，接收融合里程计
        self.create_subscription(Odometry, "/odom/fused", self.on_odom, 20)

        # 创建定时器，每 0.05 秒（20 Hz）调用一次 plan() 函数
        # 较高的控制频率有助于提升巡线稳定性
        self.timer = self.create_timer(0.05, self.plan)

    def on_offset(self, msg: Float32MultiArray) -> None:
        """视觉偏移回调函数：提取归一化偏移与置信度。

        Args:
            msg: 包含偏移量、角度、置信度等信息的 Float32MultiArray 消息。
        """

        # 检查消息 data 数组长度是否至少为 3
        # data[0] 为 offset_norm，data[2] 为 confidence
        if len(msg.data) >= 3:
            # 将偏移量裁剪到 [-1.0, 1.0] 范围内，防止异常值导致过大角速度
            self.offset_norm = max(-1.0, min(1.0, float(msg.data[0])))

            # 保存检测置信度
            self.offset_conf = float(msg.data[2])

    def on_scan(self, msg: LaserScan) -> None:
        """LiDAR 回调函数：计算有效测距中的最小值作为最近障碍物距离。

        Args:
            msg: 包含激光雷达扫描数据的 LaserScan 消息。
        """

        # 过滤有效测距值：只保留在 range_min 和 range_max 之间的值
        # 超出范围的值通常表示无效测量（过近或过远）
        valid = [r for r in msg.ranges if msg.range_min < r < msg.range_max]

        # 如果有有效测距值，取最小值作为最近障碍物距离；否则认为无障碍物
        self.min_obstacle = min(valid) if valid else math.inf

    def on_odom(self, msg: Odometry) -> None:
        """里程计回调函数：缓存最新融合里程计。

        Args:
            msg: 融合后的里程计消息。
        """

        # 将最新里程计保存到 last_odom，供后续控制或日志使用
        self.last_odom = msg

    def plan(self) -> None:
        """规划主循环：根据障碍距离与视觉偏移生成 /cmd_vel。

        状态优先级（从高到低）：
            1. 紧急避障（停转）：障碍距离 < obstacle_stop_distance。
            2. 减速避障：obstacle_stop_distance <= 障碍距离 < obstacle_slow_distance。
            3. 正常巡线：障碍距离 >= obstacle_slow_distance。
            4. 低置信度：检测置信度低时降低速度，进入搜索状态。

        该函数以 20 Hz 频率被定时器调用。
        """

        # 读取 base_speed 参数，即默认前进速度
        base_speed = float(self.get_parameter("base_speed").value)

        # 读取 max_speed 参数，即最大允许前进速度
        max_speed = float(self.get_parameter("max_speed").value)

        # 读取 max_angular 参数，即最大允许角速度
        max_angular = float(self.get_parameter("max_angular").value)

        # 读取 offset_gain 参数，即偏移量到角速度的比例增益
        gain = float(self.get_parameter("offset_gain").value)

        # 读取开始减速的障碍物距离阈值
        slow_d = float(self.get_parameter("obstacle_slow_distance").value)

        # 读取停止并转向的障碍物距离阈值
        stop_d = float(self.get_parameter("obstacle_stop_distance").value)

        # 创建 Twist 消息对象，用于封装速度指令
        cmd = Twist()

        # 初始化巡检状态为正常跟踪
        state = "tracking"

        # 根据视觉偏移计算角速度
        # offset_norm > 0 表示线缆偏右，机器人应向右转，对应负角速度
        angular = -gain * self.offset_norm

        # 判断障碍物距离状态并生成对应速度指令
        if self.min_obstacle < stop_d:
            # 情况 1：障碍物过近，进入紧急停转状态
            # 停止前进，原地旋转以避开障碍
            cmd.linear.x = 0.0

            # 根据当前线缆位置选择转向方向：
            # 若线缆偏左（offset_norm <= 0），向左侧旋转（正角速度）
            # 若线缆偏右（offset_norm > 0），向右侧旋转（负角速度）
            cmd.angular.z = max_angular * (1.0 if self.offset_norm <= 0 else -1.0)

            # 更新状态为紧急停转避障
            state = "avoid_stop_turn"

        elif self.min_obstacle < slow_d:
            # 情况 2：障碍物进入减速区，按比例降低速度并提高转向响应
            # scale 在 stop_d 处为 0，在 slow_d 处为 1，中间线性插值
            # 使用 max(0.25, ...) 保证至少保留 25% 基础速度
            scale = max(0.25, (self.min_obstacle - stop_d) / max(slow_d - stop_d, 1e-6))

            # 基础速度乘以缩放因子，并不超过最大速度
            cmd.linear.x = min(max_speed, base_speed * scale)

            # 角速度增益提高 1.4 倍，使机器人更快回正到线缆方向
            cmd.angular.z = max(-max_angular, min(max_angular, angular * 1.4))

            # 更新状态为减速避障
            state = "avoid_slow"

        else:
            # 情况 3：无障碍物或障碍物较远，执行正常巡线
            # 线速度为基础速度，但不超过最大速度
            cmd.linear.x = min(max_speed, base_speed)

            # 角速度根据视觉偏移计算，并限制在最大角速度范围内
            cmd.angular.z = max(-max_angular, min(max_angular, angular))

        # 低置信度处理：如果检测置信度低于 0.35，认为视觉信息不可靠
        if self.offset_conf < 0.35:
            # 将线速度降低到原来的 55%，给视觉检测更多时间
            cmd.linear.x *= 0.55

            # 更新状态为低置信度搜索
            state = "low_confidence_search"

        # 发布速度指令到 /cmd_vel
        self.cmd_pub.publish(cmd)

        # 发布当前巡检状态到 /inspection/state
        self.state_pub.publish(String(data=state))


# 定义程序入口函数
def main() -> None:
    """节点主函数：初始化 ROS2、创建节点、进入 spin 循环、清理资源。"""

    # 初始化 ROS2 Python 客户端库
    rclpy.init()

    # 创建 InspectionPlannerNode 实例
    node = InspectionPlannerNode()

    # 进入事件循环
    rclpy.spin(node)

    # 销毁节点
    node.destroy_node()

    # 关闭 ROS2
    rclpy.shutdown()


# 当该脚本直接运行时，调用 main() 函数
if __name__ == "__main__":
    main()
