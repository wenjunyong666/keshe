# -*- coding: utf-8 -*-
"""
AeroDock 飞行仪表面板。

保留原来的姿态、传感器、电源数据入口，重新排版为卡片式驾驶舱。
"""

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QFrame, QGridLayout, QHBoxLayout, QLabel, QVBoxLayout, QWidget

from ui.model_3d import Model3DWidget


class MetricCard(QFrame):
    """用于显示单个遥测值的仪表卡。"""

    def __init__(self, title: str, unit: str = "", accent: str = "#f0b35f"):
        super().__init__()
        self.unit = unit
        self.setObjectName("metricCard")
        self.setStyleSheet(
            f"""
            QFrame#metricCard {{
                background: #fbfcf8;
                border: 1px solid #d2e0da;
                border-radius: 22px;
            }}
            QLabel#metricTitle {{
                color: #667a72;
                font-weight: 700;
            }}
            QLabel#metricValue {{
                color: #17352f;
                font-size: 28px;
                font-weight: 900;
            }}
            QLabel#accentLine {{
                background: {accent};
                border-radius: 3px;
                min-height: 6px;
                max-height: 6px;
            }}
            """
        )

        layout = QVBoxLayout(self)
        layout.setContentsMargins(18, 16, 18, 16)
        layout.setSpacing(8)

        title_label = QLabel(title)
        title_label.setObjectName("metricTitle")
        self.value_label = QLabel(f"0.00 {unit}".strip())
        self.value_label.setObjectName("metricValue")
        accent_line = QLabel()
        accent_line.setObjectName("accentLine")

        layout.addWidget(title_label)
        layout.addWidget(self.value_label)
        layout.addStretch()
        layout.addWidget(accent_line)

    def set_value(self, value: float, precision: int = 2):
        """刷新卡片数值。"""
        self.value_label.setText(f"{value:.{precision}f} {self.unit}".strip())


class VectorCard(QFrame):
    """用于显示三轴向量的仪表卡。"""

    def __init__(self, title: str, unit: str):
        super().__init__()
        self.unit = unit
        self.setObjectName("vectorCard")
        self.setStyleSheet(
            """
            QFrame#vectorCard {
                background: #17352f;
                border-radius: 22px;
            }
            QLabel#vectorTitle {
                color: #f9c976;
                font-size: 15px;
                font-weight: 900;
            }
            QLabel#vectorValue {
                color: #e9f6f1;
                font-size: 18px;
                font-weight: 800;
            }
            """
        )

        layout = QVBoxLayout(self)
        layout.setContentsMargins(18, 16, 18, 16)
        title_label = QLabel(title)
        title_label.setObjectName("vectorTitle")
        self.value_label = QLabel(f"X 0.00  Y 0.00  Z 0.00 {unit}")
        self.value_label.setObjectName("vectorValue")
        self.value_label.setWordWrap(True)
        layout.addWidget(title_label)
        layout.addWidget(self.value_label)

    def set_vector(self, x: float, y: float, z: float):
        """刷新三轴数值。"""
        self.value_label.setText(f"X {x:.2f}   Y {y:.2f}   Z {z:.2f} {self.unit}")


class StatusPanel(QWidget):
    """飞控状态面板。"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.init_ui()

    def init_ui(self):
        """初始化仪表舱布局。"""
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(18)

        model_shell = QFrame()
        model_shell.setStyleSheet(
            """
            QFrame {
                background: #13201b;
                border-radius: 28px;
            }
            QLabel {
                color: #f9c976;
                font-size: 20px;
                font-weight: 900;
            }
            """
        )
        model_layout = QVBoxLayout(model_shell)
        model_layout.setContentsMargins(18, 18, 18, 18)
        title = QLabel("姿态雷达")
        title.setAlignment(Qt.AlignCenter)
        model_layout.addWidget(title)
        self.model_3d = Model3DWidget()
        self.model_3d.setMinimumSize(470, 470)
        model_layout.addWidget(self.model_3d, 1)
        layout.addWidget(model_shell, 3)

        data_shell = QFrame()
        data_shell.setStyleSheet("QFrame { background: transparent; }")
        data_layout = QVBoxLayout(data_shell)
        data_layout.setContentsMargins(0, 0, 0, 0)
        data_layout.setSpacing(14)

        grid = QGridLayout()
        grid.setSpacing(14)
        self.roll_card = MetricCard("横滚 ROLL", "deg", "#e26d5c")
        self.pitch_card = MetricCard("俯仰 PITCH", "deg", "#4f9d8f")
        self.yaw_card = MetricCard("航向 YAW", "deg", "#f0b35f")
        self.altitude_card = MetricCard("高度 ALT", "cm", "#71b871")
        self.voltage_card = MetricCard("电压 BAT", "V", "#d68b3c")
        self.current_card = MetricCard("电流 CUR", "A", "#7ba7c7")
        grid.addWidget(self.roll_card, 0, 0)
        grid.addWidget(self.pitch_card, 0, 1)
        grid.addWidget(self.yaw_card, 1, 0)
        grid.addWidget(self.altitude_card, 1, 1)
        grid.addWidget(self.voltage_card, 2, 0)
        grid.addWidget(self.current_card, 2, 1)
        data_layout.addLayout(grid)

        self.acc_card = VectorCard("加速度计", "m/s²")
        self.gyro_card = VectorCard("陀螺仪", "deg/s")
        data_layout.addWidget(self.acc_card)
        data_layout.addWidget(self.gyro_card)
        data_layout.addStretch()
        layout.addWidget(data_shell, 2)

    def update_attitude(self, data: dict):
        """更新姿态数据。"""
        roll = data.get("roll", 0.0)
        pitch = data.get("pitch", 0.0)
        yaw = data.get("yaw", 0.0)
        altitude = data.get("altitude", 0.0)

        self.roll_card.set_value(roll)
        self.pitch_card.set_value(pitch)
        self.yaw_card.set_value(yaw)
        self.altitude_card.set_value(altitude)
        self.model_3d.update_attitude(roll, pitch, yaw)

    def update_sensor(self, data: dict):
        """更新传感器数据。"""
        self.acc_card.set_vector(
            data.get("acc_x", 0.0),
            data.get("acc_y", 0.0),
            data.get("acc_z", 0.0),
        )
        self.gyro_card.set_vector(
            data.get("gyro_x", 0.0),
            data.get("gyro_y", 0.0),
            data.get("gyro_z", 0.0),
        )

    def update_power(self, data: dict):
        """更新电源数据。"""
        self.voltage_card.set_value(data.get("voltage", 0.0))
        self.current_card.set_value(data.get("current", 0.0))
