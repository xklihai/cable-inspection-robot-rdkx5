# -*- coding: utf-8 -*-
"""差分轮底盘运动学。

提供差分驱动机器人（differential-drive）的速度转换：
- 机器人级速度（线速度 v、角速度 ω）↔ 轮级速度（左轮、右轮线速度）。
所有速度单位均为 m/s，wheel_base 单位为米。
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class DifferentialDrive:
    """差分轮底盘运动学模型。

    Attributes:
        wheel_base_m: 左右轮间距（轮基线），单位米。
                      该值直接影响旋转时的轮速差。
    """

    wheel_base_m: float

    def twist_to_wheels(self, linear_x: float, angular_z: float) -> tuple[float, float]:
        """将机器人速度转换为左/右轮线速度。

        差分轮运动学：
            v_left  = v - ω * (L / 2)
            v_right = v + ω * (L / 2)

        Args:
            linear_x: 机器人前进方向线速度（m/s）。
            angular_z: 机器人绕 z 轴角速度（rad/s）。

        Returns:
            (left_wheel_speed, right_wheel_speed)，单位 m/s。
        """

        half = self.wheel_base_m * 0.5
        left = linear_x - angular_z * half
        right = linear_x + angular_z * half
        return left, right

    def wheels_to_twist(self, left: float, right: float) -> tuple[float, float]:
        """将左/右轮线速度转换为机器人线速度和角速度。

        逆运动学：
            v = (v_left + v_right) / 2
            ω = (v_right - v_left) / L

        Args:
            left: 左轮线速度（m/s）。
            right: 右轮线速度（m/s）。

        Returns:
            (linear_x, angular_z)，分别为机器人线速度（m/s）和角速度（rad/s）。
        """

        linear_x = (left + right) * 0.5
        # 防止 wheel_base 过小导致角速度发散
        angular_z = (right - left) / max(self.wheel_base_m, 1e-6)
        return linear_x, angular_z

