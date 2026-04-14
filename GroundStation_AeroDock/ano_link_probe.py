# -*- coding: utf-8 -*-
"""
遥控器 USB CDC <-> 地面站 联调脚本。

用途：
1. 直接监听遥控器透传出来的 ANO 帧。
2. 不开完整地面站界面也能测试 PID 命令链路。
3. 联调时快速判断问题出在“遥控器没透传”还是“飞控没回包”。

示例：
python ano_link_probe.py --list
python ano_link_probe.py --port COM8 --monitor
python ano_link_probe.py --port COM8 --read-pid 1
python ano_link_probe.py --port COM8 --reset-pid
python ano_link_probe.py --port COM8 --save-pid
"""

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "ground_station"))

from protocol.ano_protocol import ANOProtocol, CustomCommand, FrameID  # noqa: E402
from protocol.serial_comm import SerialComm  # noqa: E402


def print_ports() -> None:
    """打印可用串口。"""
    ports = SerialComm.list_ports()
    if not ports:
        print("未发现可用串口。")
        return

    print("可用串口：")
    for port in ports:
        print(f"  {port['device']:<8} {port['label']}")


def main() -> int:
    parser = argparse.ArgumentParser(description="ANO 链路联调脚本")
    parser.add_argument("--list", action="store_true", help="列出可用串口")
    parser.add_argument("--port", help="串口号，例如 COM8")
    parser.add_argument("--baud", type=int, default=115200, help="波特率，默认 115200")
    parser.add_argument("--monitor", action="store_true", help="持续监听并打印收到的 ANO 帧")
    parser.add_argument("--read-pid", type=int, choices=range(1, 7), metavar="N", help="读取 PID 组，范围 1~6")
    parser.add_argument("--reset-pid", action="store_true", help="发送重置 PID 命令")
    parser.add_argument("--save-pid", action="store_true", help="发送保存 PID 命令")
    parser.add_argument("--timeout", type=float, default=5.0, help="等待回包超时时间，默认 5 秒")
    args = parser.parse_args()

    if args.list:
        print_ports()
        return 0

    if not args.port:
        print("请先指定 --port，或者用 --list 查看可用串口。")
        return 1

    protocol = ANOProtocol()
    received = []

    def callback(func_id: int, data: bytes) -> None:
        received.append((func_id, bytes(data)))
        print_frame(protocol, func_id, data)

    comm = SerialComm(callback)
    if not comm.connect(args.port, args.baud):
        print(f"串口连接失败：{args.port}")
        return 2

    try:
        if args.read_pid is not None:
            frame = protocol.send_request_pid(args.read_pid - 1)
            print(f"发送读 PID{args.read_pid}: {frame.hex()}")
            comm.send(frame)
            wait_for_reply(received, args.timeout)

        elif args.reset_pid:
            frame = protocol.send_reset_pid()
            print(f"发送重置 PID: {frame.hex()}")
            comm.send(frame)
            wait_for_reply(received, args.timeout)

        elif args.save_pid:
            frame = protocol.send_save_pid()
            print(f"发送保存 PID: {frame.hex()}")
            comm.send(frame)
            wait_for_reply(received, args.timeout)

        elif args.monitor:
            print("开始监听，按 Ctrl+C 退出。")
            while True:
                time.sleep(0.1)

        else:
            print("未指定动作，可用 --monitor / --read-pid / --reset-pid / --save-pid。")
            return 1

    except KeyboardInterrupt:
        print("\n已停止监听。")
    finally:
        comm.disconnect()

    return 0


def wait_for_reply(received, timeout: float) -> None:
    """等待飞控经遥控器透传回包。"""
    start = time.time()
    while (time.time() - start) < timeout:
        if received:
            return
        time.sleep(0.05)
    print(f"在 {timeout:.1f}s 内未收到回包。")


def print_frame(protocol: ANOProtocol, func_id: int, data: bytes) -> None:
    """按功能码打印帧内容。"""
    print(f"收到 func=0x{func_id:02X}, len={len(data)}")

    if func_id == FrameID.STATUS:
        parsed = protocol.parse_status(data)
        print(
            "  姿态: "
            f"roll={parsed.get('roll', 0):.2f}, "
            f"pitch={parsed.get('pitch', 0):.2f}, "
            f"yaw={parsed.get('yaw', 0):.2f}, "
            f"alt={parsed.get('altitude', 0):.2f}"
        )
    elif func_id == FrameID.SENSOR:
        parsed = protocol.parse_sensor(data)
        print(
            "  传感器: "
            f"acc=({parsed.get('acc_x', 0):.2f}, {parsed.get('acc_y', 0):.2f}, {parsed.get('acc_z', 0):.2f}) "
            f"gyro=({parsed.get('gyro_x', 0):.2f}, {parsed.get('gyro_y', 0):.2f}, {parsed.get('gyro_z', 0):.2f})"
        )
    elif func_id == FrameID.POWER:
        parsed = protocol.parse_power(data)
        print(
            "  电源: "
            f"voltage={parsed.get('voltage', 0):.2f}V, "
            f"current={parsed.get('current', 0):.2f}A"
        )
    elif FrameID.PID1 <= func_id <= FrameID.PID6:
        group = int(func_id) - int(FrameID.PID1) + 1
        parsed = protocol.parse_pid(data)
        print(f"  PID{group}: {parsed}")
    elif func_id == FrameID.ACK:
        parsed = protocol.parse_ack(data)
        print(
            "  ACK: "
            f"target=0x{(parsed.get('target') or 0):02X}, "
            f"result={parsed.get('result')}, ok={parsed.get('ok')}"
        )
    else:
        print(f"  原始数据: {data.hex()}")


if __name__ == "__main__":
    raise SystemExit(main())
