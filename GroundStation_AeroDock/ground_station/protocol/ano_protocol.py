# -*- coding: utf-8 -*-
"""
ANO V4.5 协议实现。

本模块负责两件事：
1. 将上位机操作封装成 ANO 协议帧。
2. 将遥控器经 USB CDC 转发过来的飞控数据帧解析为结构化数据。
"""

import struct
from enum import IntEnum
from typing import Optional, Tuple


class FrameID(IntEnum):
    """ANO 协议功能码。"""

    VERSION = 0x00
    STATUS = 0x01
    SENSOR = 0x02
    RC_DATA = 0x03
    POWER = 0x05
    MOTOR_PWM = 0x06
    PID1 = 0x10
    PID2 = 0x11
    PID3 = 0x12
    PID4 = 0x13
    PID5 = 0x14
    PID6 = 0x15
    ACK = 0xEF


class CustomCommand(IntEnum):
    """课程设计补充命令。

    说明：
    - 0xF0：重置 PID，令飞控从 Flash 重新装载参数。
    - 0xF1：保存 PID，令飞控将当前参数写入 Flash。
    """

    RESET_PID = 0xF0
    SAVE_PID = 0xF1


class ANOProtocol:
    """ANO 协议编解码器。"""

    FRAME_HEADER_SEND = bytes([0xAA, 0xAA])
    FRAME_HEADER_RECV = bytes([0xAA, 0xAF])

    def __init__(self):
        self.rx_state = 0
        self.rx_buffer = bytearray()
        self.rx_func_id = 0
        self.rx_length = 0
        self.rx_checksum = 0

    @staticmethod
    def calculate_checksum(data: bytes) -> int:
        """计算协议校验和。"""
        return sum(data) & 0xFF

    def build_frame(self, func_id: int, data: bytes = b"") -> bytes:
        """构建发送帧。"""
        frame = bytearray(self.FRAME_HEADER_SEND)
        frame.append(func_id & 0xFF)
        frame.append(len(data) & 0xFF)
        frame.extend(data)
        frame.append(self.calculate_checksum(frame[2:]))
        return bytes(frame)

    def parse_byte(self, byte: int) -> Optional[Tuple[int, bytes]]:
        """按字节流方式解析飞控回传帧。"""
        if self.rx_state == 0:
            if byte == 0xAA:
                self.rx_state = 1
                self.rx_buffer = bytearray([byte])

        elif self.rx_state == 1:
            if byte == 0xAF:
                self.rx_state = 2
                self.rx_buffer.append(byte)
            else:
                self.rx_state = 0

        elif self.rx_state == 2:
            self.rx_func_id = byte
            self.rx_buffer.append(byte)
            self.rx_state = 3

        elif self.rx_state == 3:
            self.rx_length = byte
            self.rx_buffer.append(byte)
            self.rx_state = 5 if self.rx_length == 0 else 4

        elif self.rx_state == 4:
            self.rx_buffer.append(byte)
            if len(self.rx_buffer) >= 4 + self.rx_length:
                self.rx_state = 5

        elif self.rx_state == 5:
            self.rx_checksum = byte
            self.rx_state = 0

            calc_sum = self.calculate_checksum(self.rx_buffer[2:])
            if calc_sum == self.rx_checksum:
                data = bytes(self.rx_buffer[4:4 + self.rx_length])
                return self.rx_func_id, data

        return None

    @staticmethod
    def parse_status(data: bytes) -> dict:
        """解析姿态状态帧。"""
        if len(data) < 12:
            return {}

        roll, pitch, yaw = struct.unpack(">hhh", data[0:6])
        altitude = struct.unpack(">i", data[6:10])[0]
        fly_mode = data[10]
        locked = data[11]

        return {
            "roll": roll / 100.0,
            "pitch": pitch / 100.0,
            "yaw": yaw / 100.0,
            "altitude": altitude / 100.0,
            "fly_mode": fly_mode,
            "locked": locked,
        }

    @staticmethod
    def parse_sensor(data: bytes) -> dict:
        """解析传感器帧。"""
        if len(data) < 18:
            return {}

        values = struct.unpack(">hhhhhhhhh", data[:18])
        return {
            "acc_x": values[0] / 100.0,
            "acc_y": values[1] / 100.0,
            "acc_z": values[2] / 100.0,
            "gyro_x": values[3] / 100.0,
            "gyro_y": values[4] / 100.0,
            "gyro_z": values[5] / 100.0,
            "mag_x": values[6],
            "mag_y": values[7],
            "mag_z": values[8],
        }

    @staticmethod
    def parse_power(data: bytes) -> dict:
        """解析电源帧。"""
        if len(data) < 4:
            return {}

        voltage, current = struct.unpack(">HH", data[:4])
        return {
            "voltage": voltage / 100.0,
            "current": current / 100.0,
        }

    @staticmethod
    def parse_pid(data: bytes) -> dict:
        """解析 PID 参数帧。"""
        if len(data) < 18:
            return {}

        values = struct.unpack(">hhhhhhhhh", data[:18])
        return {
            "pid1": {"p": values[0] / 1000.0, "i": values[1] / 1000.0, "d": values[2] / 1000.0},
            "pid2": {"p": values[3] / 1000.0, "i": values[4] / 1000.0, "d": values[5] / 1000.0},
            "pid3": {"p": values[6] / 1000.0, "i": values[7] / 1000.0, "d": values[8] / 1000.0},
        }

    @staticmethod
    def parse_ack(data: bytes) -> dict:
        """解析 ACK 帧。

        常见格式：
        - data[0]：被确认的功能码
        - data[1]：结果码，0 表示成功
        """
        if not data:
            return {"target": None, "result": None, "raw": data}

        target = data[0]
        result = data[1] if len(data) > 1 else None
        return {
            "target": target,
            "result": result,
            "ok": result in (None, 0x00),
            "raw": data,
        }

    def send_request_pid(self, pid_group: int) -> bytes:
        """读取指定 PID 组。"""
        return self.build_frame(FrameID.PID1 + pid_group)

    def send_pid(self, pid_group: int, pid_data: dict) -> bytes:
        """写入指定 PID 组。"""
        data = bytearray()
        for pid_name in ("pid1", "pid2", "pid3"):
            pid = pid_data.get(pid_name, {"p": 0.0, "i": 0.0, "d": 0.0})
            data.extend(
                struct.pack(
                    ">hhh",
                    int(pid["p"] * 1000),
                    int(pid["i"] * 1000),
                    int(pid["d"] * 1000),
                )
            )
        return self.build_frame(FrameID.PID1 + pid_group, bytes(data))

    def send_command(self, cmd: int) -> bytes:
        """发送无数据体命令。"""
        return self.build_frame(cmd)

    def send_reset_pid(self) -> bytes:
        """发送自定义重置 PID 命令。"""
        return self.send_command(CustomCommand.RESET_PID)

    def send_save_pid(self) -> bytes:
        """发送自定义保存 PID 命令。"""
        return self.send_command(CustomCommand.SAVE_PID)
