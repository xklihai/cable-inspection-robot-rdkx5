# -*- coding: utf-8 -*-
"""差分轮底盘运动学模块。

本模块提供差分驱动机器人（differential-drive robot）的运动学转换，
包括正运动学与逆运动学计算：

- 逆运动学：将机器人级速度（线速度 v、角速度 ω）转换为左/右轮线速度。
- 正运动学：将左/右轮线速度转换为机器人级速度（线速度 v、角速度 ω）。

所有速度单位均为米/秒（m/s），轮基线（wheel_base）单位为米（m）。
"""

# 启用类型注解的未来导入，支持 tuple[float, float] 等现代类型写法
from __future__ import annotations

# 从 dataclasses 导入 dataclass 装饰器，用于定义不可变数据类
from dataclasses import dataclass


# frozen=True 表示实例创建后不可变，确保运动学参数在运行期间不会被意外修改
@dataclass(frozen=True)
class DifferentialDrive:
    """差分轮底盘运动学模型。

    差分轮底盘由两个同轴的驱动轮组成，通过控制左右轮的速度差实现
    前进、后退、转弯等运动。

    Attributes:
        wheel_base_m: 左右两轮中心之间的距离，称为轮基线或轮距，单位米。
                      该值直接影响机器人旋转所需的轮速差：
                      轮基线越大，相同角速度下所需的轮速差越大。
    """

    wheel_base_m: float  # 左右轮间距（轮基线），单位米

    def twist_to_wheels(self, linear_x: float, angular_z: float) -> tuple[float, float]:
        """将机器人速度转换为左/右轮线速度（逆运动学）。

        差分轮逆运动学公式推导：
            设轮基线为 L，机器人线速度为 v，角速度为 ω。
            左轮线速度 v_l = v - ω * (L / 2)
            右轮线速度 v_r = v + ω * (L / 2)

        物理意义：
            - 当 ω = 0 时，左右轮速度相等，机器人直线行驶。
            - 当 v = 0 时，左右轮速度大小相等、方向相反，机器人原地旋转。
            - 当 v 与 ω 均不为 0 时，机器人做圆弧运动。

        Args:
            linear_x: 机器人前进方向线速度，单位 m/s。
                      正值表示前进，负值表示后退。
            angular_z: 机器人绕垂直轴（z 轴）的角速度，单位 rad/s。
                       正值表示逆时针旋转，负值表示顺时针旋转。

        Returns:
            一个元组 (left_wheel_speed, right_wheel_speed)，分别表示左轮和右轮的
            目标线速度，单位 m/s。
        """

        # 计算轮基线的一半，因为左右轮速度相对于机器人中心对称分布
        half = self.wheel_base_m * 0.5

        # 根据逆运动学公式计算左轮速度
        # 左轮速度 = 线速度 - 角速度 * 半轮基线
        left = linear_x - angular_z * half

        # 根据逆运动学公式计算右轮速度
        # 右轮速度 = 线速度 + 角速度 * 半轮基线
        right = linear_x + angular_z * half

        # 返回左右轮速度元组
        return left, right

    def wheels_to_twist(self, left: float, right: float) -> tuple[float, float]:
        """将左/右轮线速度转换为机器人线速度和角速度（正运动学）。

        差分轮正运动学公式推导：
            设左轮速度为 v_l，右轮速度为 v_r，轮基线为 L。
            机器人线速度 v = (v_l + v_r) / 2
            机器人角速度 ω = (v_r - v_l) / L

        物理意义：
            - 线速度等于左右轮速度的平均值。
            - 角速度与左右轮速度差成正比，与轮基线成反比。

        Args:
            left: 左轮实际线速度，单位 m/s。
            right: 右轮实际线速度，单位 m/s。

        Returns:
            一个元组 (linear_x, angular_z)，分别表示机器人的线速度（m/s）
            和角速度（rad/s）。
        """

        # 计算机器人线速度：左右轮速度的平均值
        linear_x = (left + right) * 0.5

        # 计算机器人角速度：轮速差除以轮基线
        # 使用 max(self.wheel_base_m, 1e-6) 防止轮基线过小或为零时产生无穷大值
        angular_z = (right - left) / max(self.wheel_base_m, 1e-6)

        # 返回机器人级速度元组
        return linear_x, angular_z
