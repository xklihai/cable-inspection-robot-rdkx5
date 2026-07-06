# -*- coding: utf-8 -*-
"""差分轮底盘运动学。"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class DifferentialDrive:
    wheel_base_m: float

    def twist_to_wheels(self, linear_x: float, angular_z: float) -> tuple[float, float]:
        """将机器人速度转换为左/右轮线速度。"""

        half = self.wheel_base_m * 0.5
        left = linear_x - angular_z * half
        right = linear_x + angular_z * half
        return left, right

    def wheels_to_twist(self, left: float, right: float) -> tuple[float, float]:
        """将左/右轮线速度转换为机器人线速度和角速度。"""

        linear_x = (left + right) * 0.5
        angular_z = (right - left) / max(self.wheel_base_m, 1e-6)
        return linear_x, angular_z

