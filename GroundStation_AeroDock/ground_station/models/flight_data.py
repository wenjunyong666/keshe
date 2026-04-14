# -*- coding: utf-8 -*-
"""
飞行数据模型
"""

from dataclasses import dataclass, field
from typing import Dict


@dataclass
class AttitudeData:
    """姿态数据"""
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    altitude: float = 0.0
    fly_mode: int = 0
    locked: bool = True


@dataclass
class SensorData:
    """传感器数据"""
    acc_x: float = 0.0
    acc_y: float = 0.0
    acc_z: float = 0.0
    gyro_x: float = 0.0
    gyro_y: float = 0.0
    gyro_z: float = 0.0
    mag_x: int = 0
    mag_y: int = 0
    mag_z: int = 0


@dataclass
class PowerData:
    """电源数据"""
    voltage: float = 0.0
    current: float = 0.0


@dataclass
class PIDParam:
    """单个PID参数"""
    p: float = 0.0
    i: float = 0.0
    d: float = 0.0


@dataclass
class PIDGroup:
    """PID参数组"""
    pid1: PIDParam = field(default_factory=PIDParam)
    pid2: PIDParam = field(default_factory=PIDParam)
    pid3: PIDParam = field(default_factory=PIDParam)


class FlightData:
    """飞行数据管理"""

    def __init__(self):
        self.attitude = AttitudeData()
        self.sensor = SensorData()
        self.power = PowerData()
        self.pid_groups: Dict[int, PIDGroup] = {i: PIDGroup() for i in range(6)}

    def update_attitude(self, data: dict):
        """更新姿态数据"""
        for key, value in data.items():
            if hasattr(self.attitude, key):
                setattr(self.attitude, key, value)

    def update_sensor(self, data: dict):
        """更新传感器数据"""
        for key, value in data.items():
            if hasattr(self.sensor, key):
                setattr(self.sensor, key, value)

    def update_power(self, data: dict):
        """更新电源数据"""
        for key, value in data.items():
            if hasattr(self.power, key):
                setattr(self.power, key, value)

    def update_pid(self, group: int, data: dict):
        """更新PID参数"""
        if group not in self.pid_groups:
            return

        pid_group = self.pid_groups[group]
        for pid_name, pid_values in data.items():
            if hasattr(pid_group, pid_name):
                pid_param = getattr(pid_group, pid_name)
                pid_param.p = pid_values.get('p', 0.0)
                pid_param.i = pid_values.get('i', 0.0)
                pid_param.d = pid_values.get('d', 0.0)
