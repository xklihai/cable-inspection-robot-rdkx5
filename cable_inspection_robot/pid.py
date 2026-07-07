# -*- coding: utf-8 -*-
"""PID 控制器模块。

该模块实现了一个离散位置式 PID 控制器，具备积分限幅（anti-windup）
与输出限幅功能，不依赖 ROS，可在上位机或 RDK X5 上直接进行单元测试。

位置式 PID 公式：
    u(t) = Kp * e(t) + Ki * ∫e(t)dt + Kd * de(t)/dt

其中：
    - e(t) 为当前时刻误差（目标值 - 测量值）
    - ∫e(t)dt 为误差积分，用于消除稳态误差
    - de(t)/dt 为误差微分，用于抑制超调和振荡
"""

# 从 dataclasses 模块导入 dataclass 装饰器，用于简化仅含数据的类定义
from dataclasses import dataclass


# 使用 @dataclass 自动生成 __init__、__repr__ 等方法，减少样板代码
@dataclass
class PID:
    """带积分限幅和输出限幅的离散 PID 控制器。

    Attributes:
        kp: 比例增益（Proportional gain）。
            决定当前误差对控制器输出的直接影响程度。
            值越大，系统响应越快，但过大会导致振荡。
        ki: 积分增益（Integral gain）。
            决定误差积分对控制器输出的影响程度。
            用于消除系统的稳态误差，但过大会引起积分饱和。
        kd: 微分增益（Derivative gain）。
            决定误差变化率对控制器输出的影响程度。
            用于预测误差趋势、抑制超调，但对噪声敏感。
        integral_limit: 积分累计项的上限值。
            用于实现 anti-windup，防止积分项在饱和状态下无限增长。
        output_limit: 控制器最终输出的上限值。
            保护执行器，防止输出过大损坏电机或机构。
    """

    kp: float  # 比例增益字段
    ki: float  # 积分增益字段
    kd: float  # 微分增益字段
    integral_limit: float  # 积分限幅字段
    output_limit: float  # 输出限幅字段

    def __post_init__(self) -> None:
        """对象构造完成后初始化内部状态变量。

        该钩子函数在 dataclass 自动生成的 __init__ 之后执行，
        用于设置运行过程中需要维护的累计量。
        """
        self._integral = 0.0  # 误差积分累计值，初始化为 0
        self._last_error = 0.0  # 上一次采样时刻的误差值，初始化为 0
        self._has_last = False  # 标记是否已经保存过上一时刻误差，用于微分项初始化

    def reset(self) -> None:
        """重置 PID 控制器的内部状态。

        适用场景：
            - 切换控制模式（如从手动切换到自动）
            - 目标值发生阶跃变化时防止积分历史干扰
            - 控制器重新启动或报错恢复后

        调用后积分项清零，微分项历史丢失，控制器重新进入初始状态。
        """
        self._integral = 0.0  # 清空误差积分
        self._last_error = 0.0  # 清空上一次误差
        self._has_last = False  # 标记为无历史误差

    def update(self, target: float, measurement: float, dt: float) -> float:
        """根据目标值与测量值计算 PID 控制器输出。

        Args:
            target: 期望目标值，即系统希望达到的设定值。
            measurement: 传感器当前测量值，即系统实际反馈值。
            dt: 两次调用之间的时间间隔，单位秒，必须大于 0。
                该值影响积分项与微分项的计算精度。

        Returns:
            经过输出限幅后的控制器输出值。
        """
        # 检查时间间隔是否合法：dt 必须为正数，否则无法计算积分与微分
        if dt <= 0:
            # 非法时间间隔时返回 0，避免除以零或积分项异常增长
            return 0.0

        # 计算当前误差：目标值减去测量值
        # 当测量值低于目标值时误差为正，控制器应增大输出
        error = target - measurement

        # 积分项：对误差进行时间积分，累积历史误差
        # 积分项的作用是消除稳态误差
        self._integral += error * dt

        # 对积分项进行限幅，防止积分饱和（anti-windup）
        # 如果积分项过大，会导致系统超调严重或恢复缓慢
        self._integral = max(-self.integral_limit, min(self.integral_limit, self._integral))

        # 微分项：计算误差变化率，预测误差发展趋势
        # 首次调用时没有历史误差，微分项为 0
        derivative = 0.0 if not self._has_last else (error - self._last_error) / dt

        # 更新上一次误差值与历史标记，供下一次调用使用
        self._last_error = error
        self._has_last = True

        # 根据位置式 PID 公式计算原始输出
        # 输出 = 比例项 + 积分项 + 微分项
        output = self.kp * error + self.ki * self._integral + self.kd * derivative

        # 对输出进行限幅，保护执行器并防止控制量过大
        return max(-self.output_limit, min(self.output_limit, output))
