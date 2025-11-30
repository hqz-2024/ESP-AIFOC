"""
主窗口模块
整合所有组件，构建完整的应用程序界面
"""
import json
import os
from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QHBoxLayout, QVBoxLayout, QSplitter,
    QTextEdit, QDockWidget, QAction, QFileDialog, QMessageBox,
    QStatusBar, QLabel
)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont

from serial_manager import SerialManager
from protocol_parser import ProtocolParser
from data_model import DataModel
from plot_widget import PlotWidget
from control_panel import ControlPanel


class LogWidget(QTextEdit):
    """日志显示组件"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setReadOnly(True)
        self.setFont(QFont("Consolas", 9))
        self.setMaximumHeight(150)
        self._max_lines = 500
    
    def append_log(self, text: str, color: str = None):
        """追加日志"""
        if color:
            self.append(f'<span style="color:{color}">{text}</span>')
        else:
            self.append(text)
        
        # 限制行数
        doc = self.document()
        if doc.blockCount() > self._max_lines:
            cursor = self.textCursor()
            cursor.movePosition(cursor.Start)
            cursor.movePosition(cursor.Down, cursor.KeepAnchor, 
                              doc.blockCount() - self._max_lines)
            cursor.removeSelectedText()
        
        # 滚动到底部
        scrollbar = self.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())


class MainWindow(QMainWindow):
    """主窗口"""
    
    def __init__(self):
        super().__init__()
        self.setWindowTitle("FOC 串口数据分析工具")
        self.setMinimumSize(1200, 800)
        
        # 初始化组件
        self.serial_manager = SerialManager()
        self.protocol_parser = ProtocolParser()
        self.data_model = DataModel(max_points=2000)
        
        self._init_ui()
        self._connect_signals()
        self._init_menu()
        self._init_statusbar()
        
        # 加载配置
        self._load_config()
    
    def _init_ui(self):
        """初始化UI布局"""
        # 主部件
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)
        
        # 左侧控制面板
        self.control_panel = ControlPanel()
        self.control_panel.setMaximumWidth(350)
        main_layout.addWidget(self.control_panel)
        
        # 右侧区域（绘图+日志）
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        right_layout.setContentsMargins(0, 0, 0, 0)
        
        # 绘图区域
        self.plot_widget = PlotWidget(self.data_model)
        right_layout.addWidget(self.plot_widget, stretch=1)
        
        # 日志区域
        self.log_widget = LogWidget()
        right_layout.addWidget(self.log_widget)
        
        main_layout.addWidget(right_widget, stretch=1)
    
    def _connect_signals(self):
        """连接信号与槽"""
        # 串口管理
        self.control_panel.connect_requested.connect(self._on_connect)
        self.control_panel.disconnect_requested.connect(self._on_disconnect)
        self.control_panel.send_command.connect(self._send_command)
        
        self.serial_manager.connected.connect(self._on_connected)
        self.serial_manager.disconnected.connect(self._on_disconnected)
        self.serial_manager.data_received.connect(self.protocol_parser.parse_line)
        self.serial_manager.error_occurred.connect(self._on_error)
        
        # 协议解析
        self.protocol_parser.sensor_received.connect(self.data_model.update_sensor)
        self.protocol_parser.motor_received.connect(self.data_model.update_motor)
        self.protocol_parser.current_received.connect(self.data_model.update_current)
        self.protocol_parser.raw_data.connect(lambda s: self.log_widget.append_log(s))
        self.protocol_parser.parse_error.connect(
            lambda s: self.log_widget.append_log(s, "red"))
        self.protocol_parser.ack_received.connect(self._on_ack_received)
    
    def _init_menu(self):
        """初始化菜单栏"""
        menubar = self.menuBar()
        
        # 文件菜单
        file_menu = menubar.addMenu("文件")
        
        save_config_action = QAction("保存配置", self)
        save_config_action.triggered.connect(self._save_config)
        file_menu.addAction(save_config_action)
        
        load_config_action = QAction("加载配置", self)
        load_config_action.triggered.connect(self._load_config_dialog)
        file_menu.addAction(load_config_action)
        
        file_menu.addSeparator()
        
        exit_action = QAction("退出", self)
        exit_action.triggered.connect(self.close)
        file_menu.addAction(exit_action)
        
        # 视图菜单
        view_menu = menubar.addMenu("视图")
        
        clear_log_action = QAction("清空日志", self)
        clear_log_action.triggered.connect(self.log_widget.clear)
        view_menu.addAction(clear_log_action)
        
        clear_data_action = QAction("清空数据", self)
        clear_data_action.triggered.connect(self.data_model.clear_all)
        view_menu.addAction(clear_data_action)
    
    def _init_statusbar(self):
        """初始化状态栏"""
        self.statusbar = QStatusBar()
        self.setStatusBar(self.statusbar)

        self.status_label = QLabel("未连接")
        self.statusbar.addWidget(self.status_label)

        self.data_rate_label = QLabel("")
        self.statusbar.addPermanentWidget(self.data_rate_label)

    def _on_connect(self, port: str, baudrate: int):
        """处理连接请求"""
        self.log_widget.append_log(f"正在连接 {port} @ {baudrate}...", "blue")
        success = self.serial_manager.connect(port, baudrate)
        if not success:
            self.log_widget.append_log("连接失败", "red")

    def _on_disconnect(self):
        """处理断开请求"""
        self.serial_manager.disconnect()

    def _on_connected(self):
        """连接成功回调"""
        self.control_panel.set_connected(True)
        self.status_label.setText("已连接")
        self.status_label.setStyleSheet("color: green")
        self.log_widget.append_log("串口连接成功", "green")

    def _on_disconnected(self):
        """断开连接回调"""
        self.control_panel.set_connected(False)
        self.status_label.setText("未连接")
        self.status_label.setStyleSheet("color: red")
        self.log_widget.append_log("串口已断开", "orange")

    def _on_error(self, error_msg: str):
        """错误回调"""
        self.log_widget.append_log(f"错误: {error_msg}", "red")

    def _send_command(self, cmd: str):
        """发送命令"""
        if self.serial_manager.send(cmd):
            self.log_widget.append_log(f"发送: {cmd}", "blue")
        else:
            self.log_widget.append_log(f"发送失败: {cmd}", "red")

    def _on_ack_received(self, ack):
        """处理ACK响应"""
        if ack.success:
            self.log_widget.append_log(f"ACK: {ack.command} OK", "green")
        else:
            self.log_widget.append_log(
                f"ACK: {ack.command} 失败 - {ack.error_code}", "red")

    def _save_config(self):
        """保存配置到文件"""
        config = {
            'pid_vel': {
                'p': self.control_panel.pid_vel.p_spin.value(),
                'i': self.control_panel.pid_vel.i_spin.value(),
                'd': self.control_panel.pid_vel.d_spin.value(),
                'lpf': self.control_panel.pid_vel.lpf_spin.value()
            },
            'pid_ang': {
                'p': self.control_panel.pid_ang.p_spin.value(),
                'i': self.control_panel.pid_ang.i_spin.value(),
                'd': self.control_panel.pid_ang.d_spin.value(),
                'lpf': self.control_panel.pid_ang.lpf_spin.value()
            },
            'pid_iq': {
                'p': self.control_panel.pid_iq.p_spin.value(),
                'i': self.control_panel.pid_iq.i_spin.value(),
                'd': self.control_panel.pid_iq.d_spin.value(),
                'lpf': self.control_panel.pid_iq.lpf_spin.value()
            },
            'pid_id': {
                'p': self.control_panel.pid_id.p_spin.value(),
                'i': self.control_panel.pid_id.i_spin.value(),
                'd': self.control_panel.pid_id.d_spin.value(),
                'lpf': self.control_panel.pid_id.lpf_spin.value()
            },
            'limits': {
                'volt': self.control_panel.limit_widget.volt_spin.value(),
                'vel': self.control_panel.limit_widget.vel_spin.value(),
                'curr': self.control_panel.limit_widget.curr_spin.value()
            }
        }

        config_path = os.path.join(os.path.dirname(__file__), 'config.json')
        try:
            with open(config_path, 'w', encoding='utf-8') as f:
                json.dump(config, f, indent=2)
            self.log_widget.append_log("配置已保存", "green")
        except Exception as e:
            self.log_widget.append_log(f"保存配置失败: {e}", "red")

    def _load_config(self):
        """加载配置文件"""
        config_path = os.path.join(os.path.dirname(__file__), 'config.json')
        if not os.path.exists(config_path):
            return

        try:
            with open(config_path, 'r', encoding='utf-8') as f:
                config = json.load(f)

            # 加载PID参数
            if 'pid_vel' in config:
                c = config['pid_vel']
                self.control_panel.pid_vel.set_values(c['p'], c['i'], c['d'], c['lpf'])
            if 'pid_ang' in config:
                c = config['pid_ang']
                self.control_panel.pid_ang.set_values(c['p'], c['i'], c['d'], c['lpf'])
            if 'pid_iq' in config:
                c = config['pid_iq']
                self.control_panel.pid_iq.set_values(c['p'], c['i'], c['d'], c['lpf'])
            if 'pid_id' in config:
                c = config['pid_id']
                self.control_panel.pid_id.set_values(c['p'], c['i'], c['d'], c['lpf'])

            # 加载限制参数
            if 'limits' in config:
                lim = config['limits']
                self.control_panel.limit_widget.volt_spin.setValue(lim.get('volt', 11.0))
                self.control_panel.limit_widget.vel_spin.setValue(lim.get('vel', 50.0))
                self.control_panel.limit_widget.curr_spin.setValue(lim.get('curr', 1.0))

            self.log_widget.append_log("配置已加载", "green")
        except Exception as e:
            self.log_widget.append_log(f"加载配置失败: {e}", "red")

    def _load_config_dialog(self):
        """打开配置文件对话框"""
        file_path, _ = QFileDialog.getOpenFileName(
            self, "选择配置文件", "", "JSON文件 (*.json)")
        if file_path:
            try:
                with open(file_path, 'r', encoding='utf-8') as f:
                    config = json.load(f)
                # 这里可以扩展加载逻辑
                self.log_widget.append_log(f"已加载: {file_path}", "green")
            except Exception as e:
                QMessageBox.warning(self, "错误", f"加载失败: {e}")

    def closeEvent(self, event):
        """窗口关闭事件"""
        # 断开串口
        self.serial_manager.disconnect()
        # 保存配置
        self._save_config()
        event.accept()

