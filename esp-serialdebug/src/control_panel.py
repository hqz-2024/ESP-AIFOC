"""
控制面板模块
包含串口设置、控制模式、PID参数、限幅设置等
"""
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox, QLabel,
    QComboBox, QPushButton, QLineEdit, QFormLayout, QDoubleSpinBox,
    QTabWidget, QMessageBox, QCheckBox, QSpinBox
)
from PyQt5.QtCore import pyqtSignal
from serial_manager import SerialManager


class SerialSettingsWidget(QGroupBox):
    """串口设置组件"""
    connect_requested = pyqtSignal(str, int)  # 请求连接信号
    disconnect_requested = pyqtSignal()       # 请求断开信号
    
    def __init__(self, parent=None):
        super().__init__("串口设置", parent)
        self._init_ui()
        self._connected = False
    
    def _init_ui(self):
        layout = QFormLayout(self)
        
        # 串口选择
        port_layout = QHBoxLayout()
        self.port_combo = QComboBox()
        self.refresh_btn = QPushButton("刷新")
        self.refresh_btn.clicked.connect(self.refresh_ports)
        port_layout.addWidget(self.port_combo)
        port_layout.addWidget(self.refresh_btn)
        layout.addRow("端口:", port_layout)
        
        # 波特率
        self.baudrate_combo = QComboBox()
        self.baudrate_combo.addItems(['9600', '115200', '230400', '460800', '921600'])
        self.baudrate_combo.setCurrentText('115200')
        layout.addRow("波特率:", self.baudrate_combo)
        
        # 连接按钮
        self.connect_btn = QPushButton("连接")
        self.connect_btn.clicked.connect(self._on_connect_clicked)
        layout.addRow(self.connect_btn)
        
        # 初始化刷新端口
        self.refresh_ports()
    
    def refresh_ports(self):
        """刷新可用串口列表"""
        self.port_combo.clear()
        ports = SerialManager.list_ports()
        self.port_combo.addItems(ports)
    
    def _on_connect_clicked(self):
        """连接按钮点击"""
        if self._connected:
            self.disconnect_requested.emit()
        else:
            port = self.port_combo.currentText()
            baudrate = int(self.baudrate_combo.currentText())
            if port:
                self.connect_requested.emit(port, baudrate)
    
    def set_connected(self, connected: bool):
        """设置连接状态"""
        self._connected = connected
        self.connect_btn.setText("断开" if connected else "连接")
        self.port_combo.setEnabled(not connected)
        self.baudrate_combo.setEnabled(not connected)


class ModeControlWidget(QGroupBox):
    """控制模式设置组件"""
    mode_changed = pyqtSignal(str)      # 模式改变信号
    target_changed = pyqtSignal(float)  # 目标值改变信号
    vibration_changed = pyqtSignal(float, float, float)  # 震动参数改变信号

    def __init__(self, parent=None):
        super().__init__("控制模式", parent)
        self._velocity_limit = 50.0  # 默认速度限制
        self._init_ui()

    def _init_ui(self):
        layout = QFormLayout(self)

        # 模式选择
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(['VEL (速度)', 'POS (位置)', 'CURR (电流)', 'VIB (震动)'])
        self.mode_combo.currentIndexChanged.connect(self._on_mode_changed)
        layout.addRow("模式:", self.mode_combo)

        # 目标值
        target_layout = QHBoxLayout()
        self.target_spin = QDoubleSpinBox()
        self.target_spin.setRange(-1000, 1000)
        self.target_spin.setDecimals(3)
        self.target_spin.setValue(0)
        self.target_apply_btn = QPushButton("应用")
        self.target_apply_btn.clicked.connect(self._on_target_apply)
        target_layout.addWidget(self.target_spin)
        target_layout.addWidget(self.target_apply_btn)
        layout.addRow("目标值:", target_layout)

        # 模式应用
        self.mode_apply_btn = QPushButton("设置模式")
        self.mode_apply_btn.clicked.connect(self._on_mode_apply)
        layout.addRow(self.mode_apply_btn)

        # 震动模式参数
        self.vib_group = QGroupBox("震动参数")
        vib_layout = QFormLayout()

        self.vib_amp_spin = QDoubleSpinBox()
        self.vib_amp_spin.setRange(0, 10)
        self.vib_amp_spin.setDecimals(2)
        self.vib_amp_spin.setValue(0.5)
        self.vib_amp_spin.setSuffix(" rad")
        vib_layout.addRow("振幅:", self.vib_amp_spin)

        self.vib_freq_spin = QDoubleSpinBox()
        self.vib_freq_spin.setRange(0.1, 100)
        self.vib_freq_spin.setDecimals(2)
        self.vib_freq_spin.setValue(1.0)
        self.vib_freq_spin.setSuffix(" Hz")
        vib_layout.addRow("频率:", self.vib_freq_spin)

        self.vib_torque_spin = QDoubleSpinBox()
        self.vib_torque_spin.setRange(0, 50)
        self.vib_torque_spin.setDecimals(2)
        self.vib_torque_spin.setValue(1.0)
        self.vib_torque_spin.setSuffix(" A/V")
        vib_layout.addRow("扭矩限制:", self.vib_torque_spin)

        self.vib_apply_btn = QPushButton("应用震动参数")
        self.vib_apply_btn.clicked.connect(self._on_vibration_apply)
        vib_layout.addRow(self.vib_apply_btn)

        self.vib_group.setLayout(vib_layout)
        self.vib_group.setVisible(False)
        layout.addRow(self.vib_group)

    def _on_mode_apply(self):
        """应用模式"""
        mode_text = self.mode_combo.currentText()
        mode = mode_text.split()[0]  # 提取 VEL/POS/CURR
        self.mode_changed.emit(mode)

    def _on_target_apply(self):
        """应用目标值"""
        self.target_changed.emit(self.target_spin.value())

    def _on_mode_changed(self):
        """模式改变时更新目标值范围"""
        self._update_target_range()
        mode_text = self.mode_combo.currentText()
        mode = mode_text.split()[0]
        self.vib_group.setVisible(mode == "VIB")

    def _on_vibration_apply(self):
        """应用震动参数"""
        amplitude = self.vib_amp_spin.value()
        frequency = self.vib_freq_spin.value()
        torque = self.vib_torque_spin.value()
        self.vibration_changed.emit(amplitude, frequency, torque)

    def set_velocity_limit(self, limit: float):
        """设置速度限制，并更新目标速度范围"""
        self._velocity_limit = limit
        self._update_target_range()

    def _update_target_range(self):
        """根据当前模式和限制参数更新目标值范围"""
        mode_text = self.mode_combo.currentText()
        mode = mode_text.split()[0]

        if mode == "VEL":
            self.target_spin.setEnabled(True)
            self.target_spin.setRange(-self._velocity_limit, self._velocity_limit)
            self.target_spin.setSuffix(" rad/s")
        elif mode == "POS":
            self.target_spin.setEnabled(True)
            self.target_spin.setRange(-1000, 1000)
            self.target_spin.setSuffix(" rad")
        elif mode == "CURR":
            self.target_spin.setEnabled(True)
            self.target_spin.setRange(-50, 50)
            self.target_spin.setSuffix(" A")
        elif mode == "VIB":
            self.target_spin.setEnabled(False)


class PIDWidget(QGroupBox):
    """PID参数设置组件"""
    pid_changed = pyqtSignal(str, float, float, float, float)  # loop, P, I, D, LPF
    
    def __init__(self, title: str, loop_type: str, parent=None):
        super().__init__(title, parent)
        self.loop_type = loop_type
        self._init_ui()
    
    def _init_ui(self):
        layout = QFormLayout(self)
        
        # P参数
        self.p_spin = QDoubleSpinBox()
        self.p_spin.setRange(0, 100)
        self.p_spin.setDecimals(4)
        self.p_spin.setSingleStep(0.01)
        layout.addRow("P:", self.p_spin)
        
        # I参数
        self.i_spin = QDoubleSpinBox()
        self.i_spin.setRange(0, 100)
        self.i_spin.setDecimals(4)
        self.i_spin.setSingleStep(0.01)
        layout.addRow("I:", self.i_spin)
        
        # D参数
        self.d_spin = QDoubleSpinBox()
        self.d_spin.setRange(0, 100)
        self.d_spin.setDecimals(4)
        self.d_spin.setSingleStep(0.001)
        layout.addRow("D:", self.d_spin)
        
        # 低通滤波时间常数
        self.lpf_spin = QDoubleSpinBox()
        self.lpf_spin.setRange(0, 10)
        self.lpf_spin.setDecimals(4)
        self.lpf_spin.setSingleStep(0.001)
        layout.addRow("LPF:", self.lpf_spin)
        
        # 应用按钮
        self.apply_btn = QPushButton("应用")
        self.apply_btn.clicked.connect(self._on_apply)
        layout.addRow(self.apply_btn)

    def _on_apply(self):
        """应用PID参数"""
        self.pid_changed.emit(
            self.loop_type,
            self.p_spin.value(),
            self.i_spin.value(),
            self.d_spin.value(),
            self.lpf_spin.value()
        )

    def set_values(self, p: float, i: float, d: float, lpf: float):
        """设置参数值"""
        self.p_spin.setValue(p)
        self.i_spin.setValue(i)
        self.d_spin.setValue(d)
        self.lpf_spin.setValue(lpf)


class LimitWidget(QGroupBox):
    """限制参数设置组件"""
    limit_changed = pyqtSignal(str, float)  # type, value
    velocity_limit_changed = pyqtSignal(float)  # 速度限制改变信号（用于更新目标速度范围）

    def __init__(self, parent=None):
        super().__init__("限制参数", parent)
        self._init_ui()

    def _init_ui(self):
        layout = QFormLayout(self)

        # 电压限制
        volt_layout = QHBoxLayout()
        self.volt_spin = QDoubleSpinBox()
        self.volt_spin.setRange(0, 50)
        self.volt_spin.setDecimals(2)
        self.volt_spin.setValue(11.0)
        self.volt_spin.setSuffix(" V")
        volt_btn = QPushButton("应用")
        volt_btn.clicked.connect(lambda: self.limit_changed.emit("VOLT", self.volt_spin.value()))
        volt_layout.addWidget(self.volt_spin)
        volt_layout.addWidget(volt_btn)
        layout.addRow("电压:", volt_layout)

        # 速度限制
        vel_layout = QHBoxLayout()
        self.vel_spin = QDoubleSpinBox()
        self.vel_spin.setRange(0, 500)
        self.vel_spin.setDecimals(2)
        self.vel_spin.setValue(50.0)
        self.vel_spin.setSuffix(" rad/s")
        vel_btn = QPushButton("应用")
        vel_btn.clicked.connect(self._on_velocity_limit_apply)
        vel_layout.addWidget(self.vel_spin)
        vel_layout.addWidget(vel_btn)
        layout.addRow("速度:", vel_layout)

        # 电流限制
        curr_layout = QHBoxLayout()
        self.curr_spin = QDoubleSpinBox()
        self.curr_spin.setRange(0, 50)
        self.curr_spin.setDecimals(3)
        self.curr_spin.setValue(1.0)
        self.curr_spin.setSuffix(" A")
        curr_btn = QPushButton("应用")
        curr_btn.clicked.connect(lambda: self.limit_changed.emit("CURR", self.curr_spin.value()))
        curr_layout.addWidget(self.curr_spin)
        curr_layout.addWidget(curr_btn)
        layout.addRow("电流:", curr_layout)

    def _on_velocity_limit_apply(self):
        """速度限制应用"""
        value = self.vel_spin.value()
        self.limit_changed.emit("VEL", value)
        self.velocity_limit_changed.emit(value)  # 发出速度限制改变信号


class SyncWidget(QGroupBox):
    """参数同步设置组件"""
    sync_enabled_changed = pyqtSignal(bool)  # 同步开关状态改变
    sync_interval_changed = pyqtSignal(int)  # 同步间隔改变（秒）
    sync_now_requested = pyqtSignal()        # 立即同步请求

    def __init__(self, parent=None):
        super().__init__("参数同步", parent)
        self._init_ui()

    def _init_ui(self):
        layout = QFormLayout(self)

        # 自动同步开关
        self.auto_sync_checkbox = QCheckBox("启用自动同步")
        self.auto_sync_checkbox.setChecked(True)
        self.auto_sync_checkbox.stateChanged.connect(
            lambda state: self.sync_enabled_changed.emit(state == 2))
        layout.addRow(self.auto_sync_checkbox)

        # 同步间隔
        interval_layout = QHBoxLayout()
        self.interval_spin = QSpinBox()
        self.interval_spin.setRange(5, 300)  # 5秒到5分钟
        self.interval_spin.setValue(10)
        self.interval_spin.setSuffix(" 秒")
        self.interval_spin.valueChanged.connect(self.sync_interval_changed.emit)
        interval_layout.addWidget(self.interval_spin)
        layout.addRow("同步间隔:", interval_layout)

        # 立即同步按钮
        self.sync_now_btn = QPushButton("立即同步")
        self.sync_now_btn.clicked.connect(self.sync_now_requested.emit)
        layout.addRow(self.sync_now_btn)

        # 同步状态显示
        self.sync_status_label = QLabel("等待同步...")
        self.sync_status_label.setStyleSheet("color: gray; font-size: 9pt;")
        layout.addRow("状态:", self.sync_status_label)

    def set_sync_status(self, status: str, color: str = "gray"):
        """设置同步状态显示"""
        self.sync_status_label.setText(status)
        self.sync_status_label.setStyleSheet(f"color: {color}; font-size: 9pt;")

    def is_auto_sync_enabled(self) -> bool:
        """获取自动同步是否启用"""
        return self.auto_sync_checkbox.isChecked()

    def get_sync_interval(self) -> int:
        """获取同步间隔（秒）"""
        return self.interval_spin.value()


class ControlPanel(QWidget):
    """控制面板主组件"""
    # 信号转发
    connect_requested = pyqtSignal(str, int)
    disconnect_requested = pyqtSignal()
    send_command = pyqtSignal(str)  # 发送命令信号
    sync_params_requested = pyqtSignal()  # 请求同步参数信号

    def __init__(self, parent=None):
        super().__init__(parent)
        self._init_ui()
        self._connect_signals()

    def _init_ui(self):
        layout = QVBoxLayout(self)

        # 串口设置
        self.serial_widget = SerialSettingsWidget()
        layout.addWidget(self.serial_widget)

        # 控制模式
        self.mode_widget = ModeControlWidget()
        layout.addWidget(self.mode_widget)

        # PID设置（使用Tab）
        pid_tab = QTabWidget()
        self.pid_vel = PIDWidget("速度环 PID", "VEL")
        self.pid_vel.set_values(0.5, 20.0, 0.0, 0.01)  # 更新默认值
        self.pid_ang = PIDWidget("位置环 PID", "ANG")
        self.pid_ang.set_values(20.0, 0.0, 0.0, 0.05)  # 更新默认值
        self.pid_iq = PIDWidget("Q轴电流环 PID", "IQ")
        self.pid_iq.set_values(1.0, 3.0, 0.0, 0.005)  # 更新默认值
        self.pid_id = PIDWidget("D轴电流环 PID", "ID")
        self.pid_id.set_values(1.0, 3.0, 0.0, 0.005)  # 更新默认值

        pid_tab.addTab(self.pid_vel, "速度")
        pid_tab.addTab(self.pid_ang, "位置")
        pid_tab.addTab(self.pid_iq, "Iq")
        pid_tab.addTab(self.pid_id, "Id")
        layout.addWidget(pid_tab)

        # 限制参数
        self.limit_widget = LimitWidget()
        layout.addWidget(self.limit_widget)

        # 参数同步
        self.sync_widget = SyncWidget()
        layout.addWidget(self.sync_widget)

        layout.addStretch()

    def _connect_signals(self):
        """连接信号"""
        # 串口信号转发
        self.serial_widget.connect_requested.connect(self.connect_requested)
        self.serial_widget.disconnect_requested.connect(self.disconnect_requested)

        # 模式控制
        self.mode_widget.mode_changed.connect(self._on_mode_changed)
        self.mode_widget.target_changed.connect(self._on_target_changed)
        self.mode_widget.vibration_changed.connect(self._on_vibration_changed)

        # PID控制
        for pid_widget in [self.pid_vel, self.pid_ang, self.pid_iq, self.pid_id]:
            pid_widget.pid_changed.connect(self._on_pid_changed)

        # 限制参数
        self.limit_widget.limit_changed.connect(self._on_limit_changed)

        # 参数同步
        self.sync_widget.sync_now_requested.connect(self.sync_params_requested)

    def _on_mode_changed(self, mode: str):
        """模式改变"""
        cmd = f"SET_MODE,{mode}"
        self.send_command.emit(cmd)

    def _on_target_changed(self, value: float):
        """目标值改变"""
        cmd = f"SET_TARGET,{value}"
        self.send_command.emit(cmd)

    def _on_vibration_changed(self, amplitude: float, frequency: float, torque: float):
        """震动参数改变"""
        cmd = f"SET_VIBRATION,{amplitude},{frequency},{torque}"
        self.send_command.emit(cmd)

    def _on_pid_changed(self, loop: str, p: float, i: float, d: float, lpf: float):
        """PID参数改变"""
        cmd = f"SET_PID,{loop},{p},{i},{d},{lpf}"
        self.send_command.emit(cmd)

    def _on_limit_changed(self, limit_type: str, value: float):
        """限制参数改变"""
        cmd = f"SET_LIMIT,{limit_type},{value}"
        self.send_command.emit(cmd)

    def set_connected(self, connected: bool):
        """设置连接状态"""
        self.serial_widget.set_connected(connected)

    def get_all_params(self) -> dict:
        """获取所有当前参数"""
        return {
            'pid_vel': {
                'p': self.pid_vel.p_spin.value(),
                'i': self.pid_vel.i_spin.value(),
                'd': self.pid_vel.d_spin.value(),
                'lpf': self.pid_vel.lpf_spin.value()
            },
            'pid_ang': {
                'p': self.pid_ang.p_spin.value(),
                'i': self.pid_ang.i_spin.value(),
                'd': self.pid_ang.d_spin.value(),
                'lpf': self.pid_ang.lpf_spin.value()
            },
            'pid_iq': {
                'p': self.pid_iq.p_spin.value(),
                'i': self.pid_iq.i_spin.value(),
                'd': self.pid_iq.d_spin.value(),
                'lpf': self.pid_iq.lpf_spin.value()
            },
            'pid_id': {
                'p': self.pid_id.p_spin.value(),
                'i': self.pid_id.i_spin.value(),
                'd': self.pid_id.d_spin.value(),
                'lpf': self.pid_id.lpf_spin.value()
            },
            'limits': {
                'volt': self.limit_widget.volt_spin.value(),
                'vel': self.limit_widget.vel_spin.value(),
                'curr': self.limit_widget.curr_spin.value()
            }
        }

    def set_sync_status(self, status: str, color: str = "gray"):
        """设置同步状态"""
        self.sync_widget.set_sync_status(status, color)

    def is_auto_sync_enabled(self) -> bool:
        """获取自动同步是否启用"""
        return self.sync_widget.is_auto_sync_enabled()

    def get_sync_interval(self) -> int:
        """获取同步间隔（秒）"""
        return self.sync_widget.get_sync_interval()

