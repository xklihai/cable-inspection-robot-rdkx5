# -*- coding: utf-8 -*-
"""WHEELTEC C30D 底层主控板串口协议封装。

实际项目中 C30D 可通过 UART/USB 串口接收左右轮目标速度。这里采用
短帧协议，便于调试和抓包：

    0xAA 0x55 int16(left_mm_s) int16(right_mm_s) uint8(checksum)

帧格式说明：
    - 帧头（2 B）：0xAA 0x55，用于帧同步。
    - 左轮速度（2 B）：有符号 int16，单位 mm/s，小端序。
    - 右轮速度（2 B）：有符号 int16，单位 mm/s，小端序。
    - 校验和（1 B）：前 6 字节累加和的低 8 位。

若底层固件协议不同，只需替换本文件，不影响上层 ROS2/TROS 节点。
"""

from __future__ import annotations

import struct


HEADER = b"\xAA\x55"  # 固定帧头，用于识别一帧数据的起始位置


def clamp_mm_s(value: float) -> int:
    """将 m/s 转换为 mm/s，并限制在 int16 安全范围内。

    C30D 串口帧使用 int16 表示轮速，范围 [-32768, 32767] mm/s，
    因此需将超过该范围的数值进行裁剪，防止数据溢出。

    Args:
        value: 轮速，单位 m/s。

    Returns:
        限制后的整数 mm/s 值。
    """

    mm_s = int(round(value * 1000.0))
    return max(-32768, min(32767, mm_s))


def build_speed_frame(left_m_s: float, right_m_s: float) -> bytes:
    """构造左右轮目标速度控制帧。

    Args:
        left_m_s: 左轮目标线速度，单位 m/s。
        right_m_s: 右轮目标线速度，单位 m/s。

    Returns:
        完整的 7 字节控制帧。
    """

    # 小端序打包两个 int16 速度值
    payload = struct.pack("<hh", clamp_mm_s(left_m_s), clamp_mm_s(right_m_s))
    raw = HEADER + payload
    # 校验和：帧头 + 有效载荷共 6 字节的累加和低 8 位
    checksum = sum(raw) & 0xFF
    return raw + bytes([checksum])


def parse_wheel_feedback(frame: bytes) -> tuple[float, float] | None:
    """解析底盘返回的左右轮速度。

    返回值单位为 m/s；校验失败或长度不足时返回 None。

    Args:
        frame: 从串口读取到的原始字节序列。

    Returns:
        (left_m_s, right_m_s) 或 None（校验失败/长度不足）。
    """

    # 基本长度与帧头校验
    if len(frame) < 7 or frame[:2] != HEADER:
        return None
    # 校验和校验：前 6 字节累加和低 8 位应等于第 7 字节
    if (sum(frame[:6]) & 0xFF) != frame[6]:
        return None
    # 解包并转换回 m/s
    left_mm_s, right_mm_s = struct.unpack("<hh", frame[2:6])
    return left_mm_s / 1000.0, right_mm_s / 1000.0

