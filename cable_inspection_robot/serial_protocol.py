# -*- coding: utf-8 -*-
"""WHEELTEC C30D 底层主控板串口协议封装。

实际项目中 C30D 可通过 UART/USB 串口接收左右轮目标速度。这里采用
短帧协议，便于调试和抓包：

    0xAA 0x55 int16(left_mm_s) int16(right_mm_s) uint8(checksum)

checksum 为前 6 个字节逐字节累加后的低 8 位。若底层固件协议不同，只需
替换本文件，不影响上层 ROS2/TROS 节点。
"""

from __future__ import annotations

import struct


HEADER = b"\xAA\x55"


def clamp_mm_s(value: float) -> int:
    """将 m/s 转换为 mm/s，并限制在 int16 安全范围内。"""

    mm_s = int(round(value * 1000.0))
    return max(-32768, min(32767, mm_s))


def build_speed_frame(left_m_s: float, right_m_s: float) -> bytes:
    """构造左右轮目标速度控制帧。"""

    payload = struct.pack("<hh", clamp_mm_s(left_m_s), clamp_mm_s(right_m_s))
    raw = HEADER + payload
    checksum = sum(raw) & 0xFF
    return raw + bytes([checksum])


def parse_wheel_feedback(frame: bytes) -> tuple[float, float] | None:
    """解析底盘返回的左右轮速度。

    返回值单位为 m/s；校验失败或长度不足时返回 None。
    """

    if len(frame) < 7 or frame[:2] != HEADER:
        return None
    if (sum(frame[:6]) & 0xFF) != frame[6]:
        return None
    left_mm_s, right_mm_s = struct.unpack("<hh", frame[2:6])
    return left_mm_s / 1000.0, right_mm_s / 1000.0

