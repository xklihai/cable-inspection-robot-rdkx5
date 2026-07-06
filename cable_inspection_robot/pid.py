# -*- coding: utf-8 -*-
"""PID 控制器。

该模块不依赖 ROS，便于在上位机和 RDK X5 上做单元测试。
"""

from dataclasses import dataclass


@dataclass
class PID:
    """带积分限幅和输出限幅的离散 PID。"""

    kp: float
    ki: float
    kd: float
    integral_limit: float
    output_limit: float

    def __post_init__(self) -> None:
        self._integral = 0.0
        self._last_error = 0.0
        self._has_last = False

    def reset(self) -> None:
        self._integral = 0.0
        self._last_error = 0.0
        self._has_last = False

    def update(self, target: float, measurement: float, dt: float) -> float:
        if dt <= 0:
            return 0.0
        error = target - measurement
        self._integral += error * dt
        self._integral = max(-self.integral_limit, min(self.integral_limit, self._integral))
        derivative = 0.0 if not self._has_last else (error - self._last_error) / dt
        self._last_error = error
        self._has_last = True
        output = self.kp * error + self.ki * self._integral + self.kd * derivative
        return max(-self.output_limit, min(self.output_limit, output))

