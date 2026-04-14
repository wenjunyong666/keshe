# -*- coding: utf-8 -*-
"""
USB HID通信模块
支持USB HID设备通信
"""

import hid
from threading import Thread, Event
from typing import Callable, Optional, List
from protocol.ano_protocol import ANOProtocol


class USBHIDComm:
    """USB HID通信管理"""

    # 匿名科创飞控的VID和PID（需要根据实际设备修改）
    VENDOR_ID = 0x0483   # STM32默认VID
    PRODUCT_ID = 0x5750  # 自定义HID PID

    def __init__(self, callback: Callable[[int, bytes], None]):
        self.device: Optional[hid.device] = None
        self.protocol = ANOProtocol()
        self.callback = callback
        self.running = False
        self.thread: Optional[Thread] = None

    @staticmethod
    def list_devices() -> List[dict]:
        """列出可用的HID设备"""
        devices = []
        for dev in hid.enumerate():
            # 过滤出可能的飞控设备
            if dev['vendor_id'] == USBHIDComm.VENDOR_ID or \
               'ANO' in dev['product_string'].upper() or \
               'FLIGHT' in dev['product_string'].upper():
                devices.append({
                    'path': dev['path'],
                    'vendor_id': dev['vendor_id'],
                    'product_id': dev['product_id'],
                    'product_string': dev['product_string'],
                    'manufacturer_string': dev['manufacturer_string']
                })
        return devices

    def connect(self, vendor_id: int = None, product_id: int = None, path: bytes = None) -> bool:
        """连接USB HID设备"""
        try:
            self.device = hid.device()

            if path:
                # 通过路径连接
                self.device.open_path(path)
            else:
                # 通过VID/PID连接
                vid = vendor_id or self.VENDOR_ID
                pid = product_id or self.PRODUCT_ID
                self.device.open(vid, pid)

            # 设置非阻塞模式
            self.device.set_nonblocking(1)

            self.running = True
            self.thread = Thread(target=self._receive_loop, daemon=True)
            self.thread.start()

            return True
        except Exception as e:
            print(f"USB HID连接失败: {e}")
            return False

    def disconnect(self):
        """断开USB HID设备"""
        self.running = False
        if self.thread:
            self.thread.join(timeout=1)
        if self.device:
            self.device.close()
        self.device = None

    def send(self, data: bytes) -> bool:
        """发送数据"""
        if self.device:
            try:
                # HID报告格式：第一个字节是报告ID（通常为0）
                report = bytes([0]) + data
                self.device.write(report)
                return True
            except Exception as e:
                print(f"发送数据失败: {e}")
                return False
        return False

    def _receive_loop(self):
        """接收数据循环"""
        while self.running:
            try:
                if self.device:
                    # 读取数据（非阻塞）
                    data = self.device.read(64, timeout_ms=100)
                    if data:
                        # 解析每个字节
                        for byte in data:
                            if byte == 0:  # 跳过填充字节
                                continue
                            result = self.protocol.parse_byte(byte)
                            if result:
                                func_id, payload = result
                                self.callback(func_id, payload)
            except Exception as e:
                print(f"接收数据错误: {e}")
                break

    def is_connected(self) -> bool:
        """检查连接状态"""
        return self.device is not None
