# -*- coding: utf-8 -*-
"""PID 基础行为测试。

验证：
    1. PID 输出限幅功能：当比例项已超过 output_limit 时，输出应被限制在限幅值。
    2. 串口帧编解码一致性：构造的速度帧应能被正确解析回原始速度。

可在无 ROS 环境下直接运行：
    python3 tests/test_pid.py
"""

from cable_inspection_robot.pid import PID
from cable_inspection_robot.serial_protocol import build_speed_frame, parse_wheel_feedback


def test_pid_output_limit() -> None:
    """测试 PID 输出限幅。

    给定较大比例增益和较大目标值，比例输出会超过 output_limit=1.0，
    因此 update() 返回值应正好为 1.0。
    """
    pid = PID(10.0, 1.0, 0.0, integral_limit=0.5, output_limit=1.0)
    assert pid.update(5.0, 0.0, 0.02) == 1.0


def test_speed_frame_round_trip() -> None:
    """测试串口速度帧的构造与解析。

    构造左右轮速度帧后，通过 parse_wheel_feedback 解析，
    由于 mm/s 转换存在四舍五入，解析结果与原始值误差应小于 0.001 m/s。
    """
    frame = build_speed_frame(0.123, -0.045)
    parsed = parse_wheel_feedback(frame)
    assert parsed is not None
    left, right = parsed
    assert abs(left - 0.123) < 0.001
    assert abs(right + 0.045) < 0.001


if __name__ == "__main__":
    test_pid_output_limit()
    test_speed_frame_round_trip()
    print("PID 与串口协议测试通过。")

