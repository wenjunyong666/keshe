# -*- coding: utf-8 -*-
"""
串口通信模块
"""

import serial
import serial.tools.list_ports
from threading import Thread, Event
from typing import Callable, Optional, List, Dict
from protocol.ano_protocol import ANOProtocol


class SerialComm:
    """串口通信管理"""

    def __init__(self, callback: Callable[[int, bytes], None]):
        self.serial_port: Optional[serial.Serial] = None
        self.protocol = ANOProtocol()
        self.callback = callback
        self.running = False
        self.thread: Optional[Thread] = None
        self.reconnect_event = Event()

    @staticmethod
    def list_ports() -> List[Dict[str, str]]:
        """列出可用串口，并对常见 USB 串口桥做优先排序。"""
        result: List[Dict[str, str]] = []

        for port in serial.tools.list_ports.comports():
            description = (port.description or "").strip()
            manufacturer = (port.manufacturer or "").strip()
            hwid = (port.hwid or "").strip()
            details = " ".join(item for item in [description, manufacturer, hwid] if item)
            details_upper = details.upper()

            priority = 0
            if "JLINK CDC" in details_upper or "STLINK" in details_upper or "ST-LINK" in details_upper:
                priority = 300
            elif "CH340" in details_upper or "USB SERIAL" in details_upper or "USB-SERIAL" in details_upper:
                priority = 200
            elif "CDC" in details_upper or "VIRTUAL COM" in details_upper:
                priority = 100

            label = port.device
            if description and description != port.device:
                label = f"{port.device} - {description}"

            result.append({
                "device": port.device,
                "label": label,
                "description": description,
                "priority": priority,
            })

        result.sort(key=lambda item: (-item["priority"], item["device"]))
        return result

    def connect(self, port: str, baudrate: int = 115200) -> bool:
        """连接串口"""
        try:
            self.serial_port = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1
            )
            self.running = True
            self.thread = Thread(target=self._receive_loop, daemon=True)
            self.thread.start()
            return True
        except Exception as e:
            print(f"串口连接失败: {e}")
            return False

    def disconnect(self):
        """断开串口"""
        self.running = False
        if self.thread:
            self.thread.join(timeout=1)
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        self.serial_port = None

    def send(self, data: bytes) -> bool:
        """发送数据"""
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.write(data)
                return True
            except Exception as e:
                print(f"发送数据失败: {e}")
                return False
        return False

    def _receive_loop(self):
        """接收数据循环"""
        while self.running:
            try:
                if self.serial_port and self.serial_port.is_open:
                    if self.serial_port.in_waiting > 0:
                        byte = self.serial_port.read(1)[0]
                        result = self.protocol.parse_byte(byte)
                        if result:
                            func_id, data = result
                            self.callback(func_id, data)
            except Exception as e:
                print(f"接收数据错误: {e}")
                self.reconnect_event.set()

    def is_connected(self) -> bool:
        """检查连接状态"""
        return self.serial_port is not None and self.serial_port.is_open
