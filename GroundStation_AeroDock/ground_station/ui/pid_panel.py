# -*- coding: utf-8 -*-
"""
AeroDock PID 参数仓。

信号名称和数据结构保持不变，便于 MainWindow 继续复用原协议命令。
"""

from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import (
    QComboBox,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPushButton,
    QVBoxLayout,
    QWidget,
)


class PIDPanel(QWidget):
    """PID 参数设置面板。"""

    read_pid_signal = pyqtSignal(int)
    write_pid_signal = pyqtSignal(int, dict)
    reset_pid_signal = pyqtSignal()
    save_pid_signal = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.pid_inputs = {}
        self.init_ui()

    def init_ui(self):
        """初始化参数仓界面。"""
        self.setStyleSheet(
            """
            QFrame#pidShell {
                background: #fbfcf8;
                border: 1px solid #d2e0da;
                border-radius: 28px;
            }
            QLabel#pidTitle {
                color: #17352f;
                font-size: 24px;
                font-weight: 900;
            }
            QLabel#pidHint {
                color: #63766f;
                font-weight: 600;
            }
            QLabel#axisName {
                color: #17352f;
                font-size: 18px;
                font-weight: 900;
            }
            QLabel#columnName {
                color: #d68b3c;
                font-weight: 900;
            }
            QLineEdit {
                background: #f3f7f2;
                border: 1px solid #cbd9d3;
                border-radius: 13px;
                padding: 10px 12px;
                font-size: 16px;
                font-weight: 800;
            }
            QLineEdit:focus {
                border: 2px solid #d68b3c;
            }
            """
        )

        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(16)

        shell = QFrame()
        shell.setObjectName("pidShell")
        layout = QVBoxLayout(shell)
        layout.setContentsMargins(26, 24, 26, 24)
        layout.setSpacing(18)
        root.addWidget(shell)

        header = QHBoxLayout()
        title_box = QVBoxLayout()
        title = QLabel("参数仓")
        title.setObjectName("pidTitle")
        hint = QLabel("读取、临时写入、重置和保存 PID，命令帧仍走 ANO 协议。")
        hint.setObjectName("pidHint")
        title_box.addWidget(title)
        title_box.addWidget(hint)
        header.addLayout(title_box)
        header.addStretch()

        self.pid_group_combo = QComboBox()
        self.pid_group_combo.addItems([
            "姿态主环 PID1",
            "辅助参数 PID2",
            "辅助参数 PID3",
            "辅助参数 PID4",
            "辅助参数 PID5",
            "辅助参数 PID6",
        ])
        self.pid_group_combo.setMinimumWidth(220)
        header.addWidget(self.pid_group_combo)
        layout.addLayout(header)

        grid = QGridLayout()
        grid.setHorizontalSpacing(16)
        grid.setVerticalSpacing(14)
        grid.addWidget(QLabel(""), 0, 0)
        for col, name in enumerate(("P 比例", "I 积分", "D 微分"), 1):
            label = QLabel(name)
            label.setObjectName("columnName")
            grid.addWidget(label, 0, col)

        for row, name in enumerate(("PID1", "PID2", "PID3"), 1):
            axis = QLabel(name)
            axis.setObjectName("axisName")
            grid.addWidget(axis, row, 0)

            p_input = QLineEdit("0.000")
            i_input = QLineEdit("0.000")
            d_input = QLineEdit("0.000")
            grid.addWidget(p_input, row, 1)
            grid.addWidget(i_input, row, 2)
            grid.addWidget(d_input, row, 3)

            self.pid_inputs[name.lower()] = {
                "p": p_input,
                "i": i_input,
                "d": d_input,
            }

        layout.addLayout(grid)

        button_layout = QHBoxLayout()
        button_layout.setSpacing(12)
        self.read_btn = QPushButton("读取当前组")
        self.write_btn = QPushButton("写入飞控")
        self.reset_btn = QPushButton("从 Flash 重载")
        self.clear_btn = QPushButton("界面清零")
        self.save_btn = QPushButton("保存到 Flash")

        self.read_btn.setToolTip("从飞控读取当前 PID 参数")
        self.write_btn.setToolTip("将界面参数写入飞控，断电前需要保存才不会丢失")
        self.reset_btn.setToolTip("让飞控从 Flash 重新加载 PID 参数")
        self.clear_btn.setToolTip("只清空界面输入框，不影响飞控")
        self.save_btn.setToolTip("将飞控当前 PID 参数保存到 Flash")

        self.read_btn.clicked.connect(self.on_read_pid)
        self.write_btn.clicked.connect(self.on_write_pid)
        self.reset_btn.clicked.connect(self.on_reset_pid)
        self.clear_btn.clicked.connect(self.on_clear_pid)
        self.save_btn.clicked.connect(self.on_save_pid)

        for btn in (self.read_btn, self.write_btn, self.reset_btn, self.clear_btn, self.save_btn):
            button_layout.addWidget(btn)
        layout.addLayout(button_layout)
        root.addStretch()

    def on_read_pid(self):
        """读取 PID 参数。"""
        self.read_pid_signal.emit(self.pid_group_combo.currentIndex())

    def on_write_pid(self):
        """写入 PID 参数。"""
        self.write_pid_signal.emit(self.pid_group_combo.currentIndex(), self.get_pid_data())

    def on_reset_pid(self):
        """重置 PID 参数。"""
        reply = QMessageBox.question(
            self,
            "确认",
            "确定要让飞控从 Flash 重新加载 PID 参数吗？",
            QMessageBox.Yes | QMessageBox.No,
        )
        if reply == QMessageBox.Yes:
            self.reset_pid_signal.emit()

    def on_clear_pid(self):
        """清空界面 PID 参数。"""
        for pid_name in self.pid_inputs:
            self.pid_inputs[pid_name]["p"].setText("0.000")
            self.pid_inputs[pid_name]["i"].setText("0.000")
            self.pid_inputs[pid_name]["d"].setText("0.000")

    def on_save_pid(self):
        """保存 PID 到 Flash。"""
        reply = QMessageBox.question(
            self,
            "确认",
            "确定要将飞控当前 PID 参数保存到 Flash 吗？\n会覆盖之前保存的参数。",
            QMessageBox.Yes | QMessageBox.No,
        )
        if reply == QMessageBox.Yes:
            self.save_pid_signal.emit()

    def get_pid_data(self) -> dict:
        """获取界面 PID 数据。"""
        pid_data = {}
        for pid_name, inputs in self.pid_inputs.items():
            try:
                p = float(inputs["p"].text())
                i = float(inputs["i"].text())
                d = float(inputs["d"].text())
                pid_data[pid_name] = {"p": p, "i": i, "d": d}
            except ValueError:
                pid_data[pid_name] = {"p": 0.0, "i": 0.0, "d": 0.0}
        return pid_data

    def update_pid_display(self, pid_data: dict):
        """更新 PID 显示。"""
        for pid_name, pid_values in pid_data.items():
            if pid_name in self.pid_inputs:
                self.pid_inputs[pid_name]["p"].setText(f"{pid_values['p']:.3f}")
                self.pid_inputs[pid_name]["i"].setText(f"{pid_values['i']:.3f}")
                self.pid_inputs[pid_name]["d"].setText(f"{pid_values['d']:.3f}")
