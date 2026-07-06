# -*- coding: utf-8 -*-
"""PID 基础行为测试。"""

from cable_inspection_robot.pid import PID
from cable_inspection_robot.serial_protocol import build_speed_frame, parse_wheel_feedback


def test_pid_output_limit() -> None:
    pid = PID(10.0, 1.0, 0.0, integral_limit=0.5, output_limit=1.0)
    assert pid.update(5.0, 0.0, 0.02) == 1.0


def test_speed_frame_round_trip() -> None:
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

