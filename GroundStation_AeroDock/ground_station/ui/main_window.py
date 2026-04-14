# -*- coding: utf-8 -*-
"""
AeroDock 地面站主窗口。

说明：
底层仍沿用 ANO 协议、串口/USB HID 通信和 PID 命令，方便直接适配当前
遥控器 USB CDC 转发链路；本文件只重做窗口结构和视觉风格。
"""

import math
import time

from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtWidgets import (
    QButtonGroup,
    QComboBox,
    QFrame,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QRadioButton,
    QStackedWidget,
    QStatusBar,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from models.flight_data import FlightData
from protocol.ano_protocol import ANOProtocol, CustomCommand, FrameID
from protocol.serial_comm import SerialComm
from ui.pid_panel import PIDPanel
from ui.status_panel import StatusPanel

try:
    from protocol.usb_hid_comm import USBHIDComm

    HID_AVAILABLE = True
except ImportError:
    HID_AVAILABLE = False
    print("警告: hidapi 未安装，USB HID 功能不可用。")


AERODOCK_QSS = """
QMainWindow {
    background: #edf2ef;
}
QWidget {
    color: #13201b;
    font-family: "Microsoft YaHei UI", "Bahnschrift", "Segoe UI";
    font-size: 13px;
}
QFrame#shell {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:1,
        stop:0 #f8efe0, stop:0.45 #ecf5ee, stop:1 #dbeae7);
}
QFrame#linkDock {
    background: #17352f;
    border-radius: 28px;
}
QFrame#heroCard, QFrame#contentCard {
    background: rgba(255, 255, 255, 215);
    border: 1px solid rgba(31, 72, 63, 45);
    border-radius: 28px;
}
QLabel#brand {
    color: #f7c46c;
    font-size: 28px;
    font-weight: 800;
    letter-spacing: 2px;
}
QLabel#dockHint {
    color: #bcd7cf;
}
QLabel#sectionTitle {
    color: #24463d;
    font-size: 22px;
    font-weight: 800;
}
QLabel#linkState {
    color: #ffe3ad;
    background: rgba(255, 255, 255, 24);
    border-radius: 15px;
    padding: 8px 12px;
    font-weight: 700;
}
QComboBox, QLineEdit {
    background: #fbfcf8;
    border: 1px solid #bfd2cb;
    border-radius: 13px;
    padding: 8px 10px;
    min-height: 20px;
}
QComboBox:focus, QLineEdit:focus {
    border: 2px solid #d68b3c;
}
QPushButton {
    background: #f0b35f;
    color: #172721;
    border: 0;
    border-radius: 15px;
    padding: 10px 16px;
    font-weight: 800;
}
QPushButton:hover {
    background: #ffd083;
}
QPushButton:pressed {
    background: #c77931;
}
QPushButton#ghostButton {
    background: rgba(255, 255, 255, 30);
    color: #eaf7f2;
    border: 1px solid rgba(255, 255, 255, 70);
}
QPushButton#ghostButton:hover {
    background: rgba(255, 255, 255, 54);
}
QRadioButton {
    color: #e8f6f1;
    spacing: 8px;
}
QRadioButton::indicator {
    width: 15px;
    height: 15px;
}
QTabWidget::pane {
    border: 0;
}
QTabBar::tab {
    background: #dce9e2;
    border-radius: 16px;
    margin-right: 8px;
    padding: 11px 24px;
    color: #456157;
    font-weight: 800;
}
QTabBar::tab:selected {
    background: #17352f;
    color: #f9c976;
}
QStatusBar {
    background: #17352f;
    color: #d9f3ec;
}
"""


class MainWindow(QMainWindow):
    """AeroDock 主窗口。"""

    telemetry_received = pyqtSignal(int, bytes)
    TELEMETRY_TIMEOUT_S = 1.5

    def __init__(self):
        super().__init__()
        self.flight_data = FlightData()
        self.serial_comm = None
        self.usb_hid_comm = None
        self.protocol = ANOProtocol()
        self.comm_type = "serial"
        self.simulator_timer = None
        self.link_timer = None
        self.sim_time = 0.0
        self.last_rx_timestamp = 0.0
        self.pending_ack_name = None
        self.pending_ack_target = None
        self.current_serial_ports = []
        self.telemetry_received.connect(self.process_received_data)
        self.init_ui()
        self.init_comm()
        self.init_link_monitor()

    def init_ui(self):
        """初始化完全不同于参考工程的左右分舱界面。"""
        self.setWindowTitle("AeroDock 四轴飞行地面站")
        self.setGeometry(120, 80, 1280, 820)
        self.setStyleSheet(AERODOCK_QSS)

        shell = QFrame()
        shell.setObjectName("shell")
        self.setCentralWidget(shell)
        root_layout = QHBoxLayout(shell)
        root_layout.setContentsMargins(22, 22, 22, 22)
        root_layout.setSpacing(18)

        root_layout.addWidget(self.build_link_dock(), 0)
        root_layout.addWidget(self.build_workspace(), 1)

        self.statusBar = QStatusBar()
        self.setStatusBar(self.statusBar)
        self.statusBar.showMessage("AeroDock 已就绪，等待连接遥控器 USB CDC。")

        self.refresh_devices()

    def build_link_dock(self):
        """构建左侧通信控制坞。"""
        dock = QFrame()
        dock.setObjectName("linkDock")
        dock.setFixedWidth(330)
        layout = QVBoxLayout(dock)
        layout.setContentsMargins(24, 28, 24, 28)
        layout.setSpacing(16)

        brand = QLabel("AERODOCK")
        brand.setObjectName("brand")
        layout.addWidget(brand)

        hint = QLabel("四轴无人机遥测 / 参数写入 / 串口调试")
        hint.setObjectName("dockHint")
        hint.setWordWrap(True)
        layout.addWidget(hint)

        self.status_label = QLabel("链路未建立")
        self.status_label.setObjectName("linkState")
        self.status_label.setWordWrap(True)
        layout.addWidget(self.status_label)

        layout.addSpacing(8)
        self.comm_type_group = QButtonGroup()
        self.serial_radio = QRadioButton("串口 / USB CDC")
        self.usb_hid_radio = QRadioButton("USB HID")
        self.serial_radio.setChecked(True)
        self.comm_type_group.addButton(self.serial_radio)
        self.comm_type_group.addButton(self.usb_hid_radio)
        self.serial_radio.toggled.connect(self.on_comm_type_changed)
        layout.addWidget(self.serial_radio)
        layout.addWidget(self.usb_hid_radio)

        if not HID_AVAILABLE:
            self.usb_hid_radio.setEnabled(False)
            self.usb_hid_radio.setToolTip("需要安装 hidapi: pip install hidapi")

        layout.addWidget(QLabel("设备端口"))
        self.port_combo = QComboBox()
        layout.addWidget(self.port_combo)

        layout.addWidget(QLabel("波特率"))
        self.baudrate_combo = QComboBox()
        self.baudrate_combo.addItems(["9600", "115200", "230400", "460800"])
        self.baudrate_combo.setCurrentText("115200")
        layout.addWidget(self.baudrate_combo)

        self.connect_btn = QPushButton("建立链路")
        self.connect_btn.clicked.connect(self.toggle_connection)
        layout.addWidget(self.connect_btn)

        self.refresh_btn = QPushButton("扫描设备")
        self.refresh_btn.setObjectName("ghostButton")
        self.refresh_btn.clicked.connect(self.refresh_devices)
        layout.addWidget(self.refresh_btn)

        self.sim_btn = QPushButton("开启模拟飞行")
        self.sim_btn.setObjectName("ghostButton")
        self.sim_btn.clicked.connect(self.toggle_simulator)
        layout.addWidget(self.sim_btn)

        layout.addStretch()
        foot = QLabel("协议：ANO V4.5 兼容帧\n方向：遥控器 CDC 转发飞控遥测")
        foot.setObjectName("dockHint")
        foot.setWordWrap(True)
        layout.addWidget(foot)
        return dock

    def build_workspace(self):
        """构建右侧主工作区。"""
        workspace = QFrame()
        workspace.setObjectName("contentCard")
        layout = QVBoxLayout(workspace)
        layout.setContentsMargins(24, 24, 24, 24)
        layout.setSpacing(18)

        hero = QFrame()
        hero.setObjectName("heroCard")
        hero_layout = QHBoxLayout(hero)
        hero_layout.setContentsMargins(22, 18, 22, 18)

        title_box = QVBoxLayout()
        title = QLabel("飞行控制台")
        title.setObjectName("sectionTitle")
        subtitle = QLabel("保留原有通信协议，重新设计为仪表舱式地面站界面。")
        subtitle.setStyleSheet("color:#61766f; font-weight:600;")
        title_box.addWidget(title)
        title_box.addWidget(subtitle)
        hero_layout.addLayout(title_box)
        hero_layout.addStretch()
        self.quick_status = QLabel("NO LINK")
        self.quick_status.setStyleSheet(
            "background:#17352f; color:#f9c976; border-radius:18px; "
            "padding:10px 18px; font-weight:900; letter-spacing:1px;"
        )
        hero_layout.addWidget(self.quick_status)
        layout.addWidget(hero)

        self.tab_widget = QTabWidget()
        self.status_panel = StatusPanel()
        self.tab_widget.addTab(self.status_panel, "飞行仪表")

        self.pid_panel = PIDPanel()
        self.pid_panel.read_pid_signal.connect(self.on_read_pid)
        self.pid_panel.write_pid_signal.connect(self.on_write_pid)
        self.pid_panel.reset_pid_signal.connect(self.on_reset_pid)
        self.pid_panel.save_pid_signal.connect(self.on_save_pid)
        self.tab_widget.addTab(self.pid_panel, "参数仓")
        layout.addWidget(self.tab_widget, 1)
        return workspace

    def init_comm(self):
        """初始化通信实例。"""
        self.serial_comm = SerialComm(self.on_data_received)
        if HID_AVAILABLE:
            self.usb_hid_comm = USBHIDComm(self.on_data_received)

    def init_link_monitor(self):
        """定时检查数据链路状态。"""
        self.link_timer = QTimer(self)
        self.link_timer.timeout.connect(self.check_link_state)
        self.link_timer.start(300)

    def set_link_text(self, text: str, quick: str):
        """统一刷新左侧链路牌和右上角状态胶囊。"""
        self.status_label.setText(text)
        self.quick_status.setText(quick)

    def on_comm_type_changed(self):
        """切换通信方式。"""
        if self.serial_radio.isChecked():
            self.comm_type = "serial"
            self.baudrate_combo.setEnabled(True)
        else:
            self.comm_type = "usb_hid"
            self.baudrate_combo.setEnabled(False)
        self.refresh_devices()

    def refresh_devices(self):
        """刷新设备列表。"""
        self.port_combo.clear()

        if self.comm_type == "serial":
            self.current_serial_ports = SerialComm.list_ports()
            for port in self.current_serial_ports:
                self.port_combo.addItem(port["label"], port["device"])
        elif HID_AVAILABLE and self.usb_hid_comm:
            for dev in USBHIDComm.list_devices():
                label = (
                    f"{dev['product_string']} "
                    f"(VID:{dev['vendor_id']:04X} PID:{dev['product_id']:04X})"
                )
                self.port_combo.addItem(label, dev["path"])

    def refresh_ports(self):
        """兼容旧入口。"""
        self.refresh_devices()

    def toggle_simulator(self):
        """切换模拟器状态。"""
        if self.simulator_timer and self.simulator_timer.isActive():
            self.stop_simulator()
        else:
            self.start_simulator()

    def start_simulator(self):
        """启动内置模拟器。"""
        if self.is_connected():
            self.disconnect_device()

        self.sim_time = 0.0
        self.simulator_timer = QTimer(self)
        self.simulator_timer.timeout.connect(self.simulate_data)
        self.simulator_timer.start(50)
        self.sim_btn.setText("关闭模拟飞行")
        self.set_link_text("模拟数据流运行中", "SIM")
        self.statusBar.showMessage("已启动内置模拟器")

    def stop_simulator(self):
        """停止内置模拟器。"""
        if self.simulator_timer:
            self.simulator_timer.stop()
            self.simulator_timer.deleteLater()
            self.simulator_timer = None

        self.sim_btn.setText("开启模拟飞行")
        self.set_link_text("链路未建立", "NO LINK")
        self.statusBar.showMessage("已停止内置模拟器")

    def simulate_data(self):
        """产生模拟飞行数据。"""
        t = self.sim_time
        attitude_data = {
            "roll": 30.0 * math.sin(t * 0.5),
            "pitch": 20.0 * math.sin(t * 0.3),
            "yaw": 180.0 * math.sin(t * 0.1),
            "altitude": 100.0 + 50.0 * math.sin(t * 0.2),
            "fly_mode": 1,
            "locked": False,
        }
        self.flight_data.update_attitude(attitude_data)
        self.status_panel.update_attitude(attitude_data)

        sensor_data = {
            "acc_x": 0.1 * math.sin(t * 0.5),
            "acc_y": 0.1 * math.cos(t * 0.5),
            "acc_z": 9.8 + 0.2 * math.sin(t * 0.3),
            "gyro_x": 10.0 * math.sin(t * 0.4),
            "gyro_y": 10.0 * math.cos(t * 0.4),
            "gyro_z": 5.0 * math.sin(t * 0.2),
        }
        self.flight_data.update_sensor(sensor_data)
        self.status_panel.update_sensor(sensor_data)

        power_data = {
            "voltage": 11.1 + 0.5 * math.sin(t * 0.1),
            "current": 5.0 + 2.0 * math.sin(t * 0.15),
        }
        self.flight_data.update_power(power_data)
        self.status_panel.update_power(power_data)

        if int(t * 10) % 100 == 0:
            pid_data = {
                "pid1": {"p": 2.5, "i": 0.1, "d": 0.5},
                "pid2": {"p": 3.0, "i": 0.2, "d": 0.6},
                "pid3": {"p": 1.5, "i": 0.05, "d": 0.3},
            }
            self.flight_data.update_pid(0, pid_data)
            self.pid_panel.update_pid_display(pid_data)

        self.sim_time += 0.05

    def toggle_connection(self):
        """切换连接状态。"""
        if self.is_connected():
            self.disconnect_device()
        else:
            self.connect_device()

    def is_connected(self):
        """检查当前是否已连接。"""
        if self.comm_type == "serial":
            return self.serial_comm.is_connected()
        return self.usb_hid_comm and self.usb_hid_comm.is_connected()

    def connect_device(self):
        """连接选中的设备。"""
        if self.simulator_timer and self.simulator_timer.isActive():
            self.stop_simulator()

        if self.comm_type == "serial":
            port = self.port_combo.currentData()
            if not port:
                port = self.port_combo.currentText().split(" - ", 1)[0].strip()

            baudrate = int(self.baudrate_combo.currentText())
            if self.serial_comm.connect(port, baudrate):
                self.last_rx_timestamp = time.time()
                self.connect_btn.setText("断开链路")
                self.set_link_text(f"已连接：{port}", "LINK ON")
                self.statusBar.showMessage(f"串口已连接: {port}")
            else:
                QMessageBox.warning(self, "连接失败", f"无法连接到串口 {port}")
        else:
            if not HID_AVAILABLE or not self.usb_hid_comm:
                QMessageBox.warning(self, "错误", "USB HID 功能不可用，请先安装 hidapi")
                return

            device_path = self.port_combo.currentData()
            if device_path and self.usb_hid_comm.connect(path=device_path):
                self.last_rx_timestamp = time.time()
                self.connect_btn.setText("断开链路")
                self.set_link_text("已连接：USB HID", "LINK ON")
                self.statusBar.showMessage("USB HID 已连接")
            else:
                QMessageBox.warning(self, "连接失败", "无法连接到 USB HID 设备")

    def disconnect_device(self):
        """断开当前设备。"""
        if self.comm_type == "serial":
            self.serial_comm.disconnect()
        elif self.usb_hid_comm:
            self.usb_hid_comm.disconnect()

        self.pending_ack_name = None
        self.pending_ack_target = None
        self.connect_btn.setText("建立链路")
        self.set_link_text("链路未建立", "NO LINK")
        self.statusBar.showMessage("已断开连接")

    def check_link_state(self):
        """检测链路超时和端口异常。"""
        if not self.is_connected():
            return

        if self.comm_type == "serial" and self.serial_comm.reconnect_event.is_set():
            self.serial_comm.reconnect_event.clear()
            self.disconnect_device()
            self.statusBar.showMessage("串口异常断开，请重新连接")
            return

        if self.last_rx_timestamp > 0 and (time.time() - self.last_rx_timestamp) > self.TELEMETRY_TIMEOUT_S:
            self.set_link_text("链路已打开，等待飞控遥测", "WAITING")

    def on_data_received(self, func_id: int, data: bytes):
        """串口线程收到数据后切回 UI 线程处理。"""
        self.telemetry_received.emit(int(func_id), bytes(data))

    def process_received_data(self, func_id: int, data: bytes):
        """在 UI 主线程中处理飞控数据。"""
        try:
            self.last_rx_timestamp = time.time()
            if self.comm_type == "serial" and self.serial_comm.is_connected():
                current_port = self.port_combo.currentData() or self.port_combo.currentText()
                if current_port:
                    self.set_link_text(f"遥测接收中：{str(current_port).split(' - ', 1)[0]}", "LIVE")

            if func_id == FrameID.STATUS:
                parsed = self.protocol.parse_status(data)
                self.flight_data.update_attitude(parsed)
                self.status_panel.update_attitude(parsed)
            elif func_id == FrameID.SENSOR:
                parsed = self.protocol.parse_sensor(data)
                self.flight_data.update_sensor(parsed)
                self.status_panel.update_sensor(parsed)
            elif func_id == FrameID.POWER:
                parsed = self.protocol.parse_power(data)
                self.flight_data.update_power(parsed)
                self.status_panel.update_power(parsed)
            elif FrameID.PID1 <= func_id <= FrameID.PID6:
                group = func_id - FrameID.PID1
                parsed = self.protocol.parse_pid(data)
                self.flight_data.update_pid(group, parsed)
                self.pid_panel.update_pid_display(parsed)
                self.statusBar.showMessage(f"已接收 PID{group + 1} 参数")
            elif func_id == FrameID.ACK:
                self.handle_ack(data)
        except Exception as exc:
            print(f"数据解析错误: {exc}")

    def handle_ack(self, data: bytes):
        """处理飞控 ACK。"""
        ack = self.protocol.parse_ack(data)
        target = ack["target"]
        ok = ack["ok"]

        if self.pending_ack_target is not None and target == self.pending_ack_target:
            action_name = self.pending_ack_name or f"命令 0x{target:02X}"
        else:
            action_name = f"命令 0x{target:02X}" if target is not None else "命令"

        if ok:
            self.statusBar.showMessage(f"{action_name} 已确认")
        else:
            result = ack["result"]
            self.statusBar.showMessage(f"{action_name} 被拒绝，返回码: {result}")

        if target == self.pending_ack_target:
            self.pending_ack_name = None
            self.pending_ack_target = None

    def remember_pending_ack(self, target: int, action_name: str):
        """记录一个待确认命令。"""
        self.pending_ack_target = int(target)
        self.pending_ack_name = action_name

    def on_read_pid(self, group: int):
        """读取 PID 参数。"""
        if not self.is_connected():
            QMessageBox.warning(self, "错误", "请先连接设备")
            return

        frame = self.protocol.send_request_pid(group)
        self.send_data(frame)
        self.statusBar.showMessage(f"正在读取 PID{group + 1} 参数...")

    def on_write_pid(self, group: int, pid_data: dict):
        """写入 PID 参数。"""
        if not self.is_connected():
            QMessageBox.warning(self, "错误", "请先连接设备")
            return

        frame = self.protocol.send_pid(group, pid_data)
        self.remember_pending_ack(FrameID.PID1 + group, f"写入 PID{group + 1}")
        self.send_data(frame)
        self.statusBar.showMessage(f"已发送 PID{group + 1} 参数，等待飞控确认")

    def on_reset_pid(self):
        """重置 PID 参数。"""
        if not self.is_connected():
            QMessageBox.warning(self, "错误", "请先连接设备")
            return

        frame = self.protocol.send_reset_pid()
        self.remember_pending_ack(CustomCommand.RESET_PID, "重置 PID")
        self.send_data(frame)
        self.statusBar.showMessage("已发送重置 PID 命令")

    def on_save_pid(self):
        """保存 PID 到 Flash。"""
        if not self.is_connected():
            QMessageBox.warning(self, "错误", "请先连接设备")
            return

        frame = self.protocol.send_save_pid()
        self.remember_pending_ack(CustomCommand.SAVE_PID, "保存 PID")
        self.send_data(frame)
        self.statusBar.showMessage("已发送保存 PID 命令")

    def send_data(self, data: bytes):
        """发送数据到当前连接设备。"""
        if self.comm_type == "serial":
            self.serial_comm.send(data)
        elif self.usb_hid_comm:
            self.usb_hid_comm.send(data)

    def closeEvent(self, event):
        """关闭窗口时释放资源。"""
        if self.simulator_timer and self.simulator_timer.isActive():
            self.stop_simulator()

        if self.is_connected():
            self.disconnect_device()
        event.accept()
