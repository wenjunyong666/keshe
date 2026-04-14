# -*- coding: utf-8 -*-
"""
内置数据模拟器
直接在地面站内部生成测试数据，无需串口
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from PyQt5.QtWidgets import QApplication, QMessageBox
from PyQt5.QtCore import QTimer
from ui.main_window import MainWindow
from protocol.ano_protocol import ANOProtocol
import math


class SimulatorMainWindow(MainWindow):
    """带内置模拟器的主窗口"""

    def __init__(self):
        super().__init__()
        self.simulator_timer = None
        self.sim_time = 0
        self.protocol = ANOProtocol()

        # 添加模拟器按钮
        from PyQt5.QtWidgets import QPushButton
        self.sim_btn = QPushButton("启动模拟器")
        self.sim_btn.clicked.connect(self.toggle_simulator)

        # 将按钮添加到连接区域
        conn_layout = self.centralWidget().layout().itemAt(0).widget().layout()
        conn_layout.addWidget(self.sim_btn)

    def toggle_simulator(self):
        """切换模拟器状态"""
        if self.simulator_timer and self.simulator_timer.isActive():
            self.stop_simulator()
        else:
            self.start_simulator()

    def start_simulator(self):
        """启动内置模拟器"""
        self.sim_time = 0
        self.simulator_timer = QTimer()
        self.simulator_timer.timeout.connect(self.simulate_data)
        self.simulator_timer.start(50)  # 20Hz

        self.sim_btn.setText("停止模拟器")
        self.status_label.setText("模拟器运行中")
        self.statusBar.showMessage("内置模拟器已启动")

        QMessageBox.information(self, "模拟器",
            "内置模拟器已启动！\n\n"
            "无需连接串口，数据将自动显示。\n"
            "3D模型会实时旋转显示姿态变化。")

    def stop_simulator(self):
        """停止模拟器"""
        if self.simulator_timer:
            self.simulator_timer.stop()
            self.simulator_timer = None

        self.sim_btn.setText("启动模拟器")
        self.status_label.setText("模拟器已停止")
        self.statusBar.showMessage("内置模拟器已停止")

    def simulate_data(self):
        """生成模拟数据"""
        t = self.sim_time

        # 模拟姿态数据
        roll = 30 * math.sin(t * 0.5)
        pitch = 20 * math.sin(t * 0.3)
        yaw = 180 * math.sin(t * 0.1)
        altitude = 100 + 50 * math.sin(t * 0.2)

        attitude_data = {
            'roll': roll,
            'pitch': pitch,
            'yaw': yaw,
            'altitude': altitude,
            'fly_mode': 1,
            'locked': False
        }
        self.flight_data.update_attitude(attitude_data)
        self.status_panel.update_attitude(attitude_data)

        # 模拟传感器数据
        sensor_data = {
            'acc_x': 0.1 * math.sin(t * 0.5),
            'acc_y': 0.1 * math.cos(t * 0.5),
            'acc_z': 9.8 + 0.2 * math.sin(t * 0.3),
            'gyro_x': 10 * math.sin(t * 0.4),
            'gyro_y': 10 * math.cos(t * 0.4),
            'gyro_z': 5 * math.sin(t * 0.2)
        }
        self.flight_data.update_sensor(sensor_data)
        self.status_panel.update_sensor(sensor_data)

        # 模拟电源数据
        power_data = {
            'voltage': 11.1 + 0.5 * math.sin(t * 0.1),
            'current': 5.0 + 2.0 * math.sin(t * 0.15)
        }
        self.flight_data.update_power(power_data)
        self.status_panel.update_power(power_data)

        # 每5秒更新一次PID
        if int(t * 10) % 100 == 0:
            pid_data = {
                'pid1': {'p': 2.5, 'i': 0.1, 'd': 0.5},
                'pid2': {'p': 3.0, 'i': 0.2, 'd': 0.6},
                'pid3': {'p': 1.5, 'i': 0.05, 'd': 0.3}
            }
            self.flight_data.update_pid(0, pid_data)
            self.pid_panel.update_pid_display(pid_data)

        self.sim_time += 0.05


def main():
    """主函数"""
    app = QApplication(sys.argv)
    app.setStyle('Fusion')

    window = SimulatorMainWindow()
    window.show()

    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
