# -*- coding: utf-8 -*-
"""PID 控制器。

该模块不依赖 ROS，便于在上位机和 RDK X5 上做单元测试。
采用位置式离散 PID，包含积分限幅（anti-windup）与输出限幅，
适合轮速、航向等常见机器人控制场景。
"""

from dataclasses import dataclass


@dataclass
class PID:
    """带积分限幅和输出限幅的离散 PID。

    Attributes:
        kp: 比例增益，决定当前误差对输出的直接影响。
        ki: 积分增益，用于消除稳态误差。
        kd: 微分增益，抑制超调、提升响应速度。
        integral_limit: 积分累计上限，防止积分饱和。
        output_limit: 控制器输出上限，保护执行器。
    """

    kp: float
    ki: float
    kd: float
    integral_limit: float
    output_limit: float

    def __post_init__(self) -> None:
        """初始化内部状态。"""
        self._integral = 0.0      # 误差积分累计值
        self._last_error = 0.0    # 上一次误差，用于微分计算
        self._has_last = False    # 标记是否已有历史误差

    def reset(self) -> None:
        """重置积分与历史误差，通常在切换控制模式或重新启动时调用。"""
        self._integral = 0.0
        self._last_error = 0.0
        self._has_last = False

    def update(self, target: float, measurement: float, dt: float) -> float:
        """根据目标值与测量值计算 PID 输出。

        Args:
            target: 期望目标值。
            measurement: 传感器当前测量值。
            dt: 两次调用的时间间隔（秒），必须大于 0。

        Returns:
            限幅后的控制器输出。
        """
        if dt <= 0:
            # 异常时间间隔，避免除以零或积分爆炸
            return 0.0
        error = target - measurement

        # 积分项：对误差进行累加，并做限幅以防止积分饱和
        self._integral += error * dt
        self._integral = max(-self.integral_limit, min(self.integral_limit, self._integral))

        # 微分项：利用历史误差估计误差变化率；首次调用时微分项为 0
        derivative = 0.0 if not self._has_last else (error - self._last_error) / dt
        self._last_error = error
        self._has_last = True

        # 计算 PID 输出并做限幅
        output = self.kp * error + self.ki * self._integral + self.kd * derivative
        return max(-self.output_limit, min(self.output_limit, output))

