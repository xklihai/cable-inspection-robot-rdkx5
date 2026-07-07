# -*- coding: utf-8 -*-
"""PID 基础行为测试。

本测试模块验证两个核心功能：
    1. PID 控制器的输出限幅功能。
    2. 串口速度帧的构造与解析一致性。

这些测试不依赖 ROS 环境，可在普通 Python 解释器中直接运行，
便于在桌面开发阶段快速验证基础模块正确性。

运行方式：
    python3 tests/test_pid.py

预期输出：
    PID 与串口协议测试通过。
"""

# 从本包导入 PID 控制器类
from cable_inspection_robot.pid import PID

# 从本包导入串口帧构造与解析函数
from cable_inspection_robot.serial_protocol import build_speed_frame, parse_wheel_feedback


def test_pid_output_limit() -> None:
    """测试 PID 输出限幅功能。

    测试思路：
        构造一个高比例增益的 PID 控制器，给定较大目标值与零测量值，
        使得比例项输出明显超过 output_limit。此时控制器输出应被限制在
        output_limit 处，而不是无限增大。

    验证条件：
        pid.update(5.0, 0.0, 0.02) 返回值必须等于 1.0。
    """

    # 构造 PID 控制器：Kp=10.0, Ki=1.0, Kd=0.0
    # 积分限幅 0.5，输出限幅 1.0
    pid = PID(10.0, 1.0, 0.0, integral_limit=0.5, output_limit=1.0)

    # 计算输出：目标 5.0，测量 0.0，时间间隔 0.02 秒
    # 比例项为 10.0 * 5.0 = 50.0，远超输出限幅 1.0
    # 因此输出应被限制为 1.0
    assert pid.update(5.0, 0.0, 0.02) == 1.0


def test_speed_frame_round_trip() -> None:
    """测试串口速度帧的构造与解析一致性。

    测试思路：
        使用 build_speed_frame 构造一个包含左右轮速度的控制帧，
        然后使用 parse_wheel_feedback 解析该帧，验证解析结果与原始速度
        在误差允许范围内一致。

    说明：
        由于帧中使用 int16 mm/s 表示速度，存在四舍五入误差，
        因此允许 0.001 m/s 的误差。
    """

    # 构造速度帧：左轮 0.123 m/s，右轮 -0.045 m/s
    frame = build_speed_frame(0.123, -0.045)

    # 解析帧，获取左右轮速度
    parsed = parse_wheel_feedback(frame)

    # 验证解析成功（非 None）
    assert parsed is not None

    # 解包解析结果
    left, right = parsed

    # 验证左轮速度与原始值误差小于 0.001 m/s
    assert abs(left - 0.123) < 0.001

    # 验证右轮速度与原始值误差小于 0.001 m/s
    assert abs(right + 0.045) < 0.001


# 当该脚本直接运行时执行测试
if __name__ == "__main__":
    # 执行 PID 输出限幅测试
    test_pid_output_limit()

    # 执行串口帧往返测试
    test_speed_frame_round_trip()

    # 打印测试通过提示
    print("PID 与串口协议测试通过。")
