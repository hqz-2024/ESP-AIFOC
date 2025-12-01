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


class UserConfig:
    """用户配置管理类"""

    DEFAULT_CONFIG = {
        "serial": {
            "baudrate": 115200,
            "default_port": ""
        },
        "pid_velocity": {
            "p": 0.3,
            "i": 1.8,
            "d": 0.0,
            "lpf": 0.3
        },
        "pid_angle": {
            "p": 20.0,
            "i": 0.0,
            "d": 0.0,
            "lpf": 0.05
        },
        "pid_current_q": {
            "p": 1.0,
            "i": 3.0,
            "d": 0.0,
            "lpf": 0.005
        },
        "pid_current_d": {
            "p": 1.0,
            "i": 3.0,
            "d": 0.0,
            "lpf": 0.005
        },
        "limits": {
            "voltage": 11.0,
            "velocity": 50.0,
            "current": 3.0
        },
        "sync": {
            "enabled": True,
            "interval": 10
        },
        "control": {
            "default_mode": "VEL",
            "default_target": 0.0
        },
        "plot": {
            "max_points": 2000,
            "update_interval": 100
        }
    }

    @staticmethod
    def get_user_config_path():
        """获取用户配置文件路径"""
        # 优先使用项目根目录的 user_config.json
        root_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'user_config.json')
        if os.path.exists(root_path):
            return root_path
        # 否则使用 src 目录下的
        return os.path.join(os.path.dirname(__file__), 'user_config.json')

    @staticmethod
    def load():
        """加载用户配置"""
        config_path = UserConfig.get_user_config_path()

        if not os.path.exists(config_path):
            print(f"用户配置文件不存在，使用默认配置: {config_path}")
            return UserConfig.DEFAULT_CONFIG.copy()

        try:
            with open(config_path, 'r', encoding='utf-8') as f:
                user_config = json.load(f)

            # 移除注释字段
            def remove_comments(d):
                if isinstance(d, dict):
                    return {k: remove_comments(v) for k, v in d.items() if not k.startswith('_')}
                return d

            user_config = remove_comments(user_config)

            # 合并默认配置（确保所有字段都存在）
            config = UserConfig.DEFAULT_CONFIG.copy()
            for key in config:
                if key in user_config and isinstance(config[key], dict):
                    config[key].update(user_config[key])
                elif key in user_config:
                    config[key] = user_config[key]

            print(f"用户配置已加载: {config_path}")
            return config

        except Exception as e:
            print(f"加载用户配置失败: {e}，使用默认配置")
            return UserConfig.DEFAULT_CONFIG.copy()


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

        # 加载用户配置
        self.user_config = UserConfig.load()

        # 初始化组件
        self.serial_manager = SerialManager()
        self.protocol_parser = ProtocolParser()
        max_points = self.user_config.get('plot', {}).get('max_points', 2000)
        self.data_model = DataModel(max_points=max_points)

        self._init_ui()
        self._connect_signals()
        self._init_menu()
        self._init_statusbar()
        self._init_sync_timer()

        # 应用用户配置
        self._apply_user_config()

        # 加载运行时配置
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
        self.control_panel.sync_params_requested.connect(self._sync_params_now)

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

        # 参数同步
        self.control_panel.sync_widget.sync_enabled_changed.connect(self._on_sync_enabled_changed)
        self.control_panel.sync_widget.sync_interval_changed.connect(self._on_sync_interval_changed)

        # 速度限制改变时更新目标速度范围
        self.control_panel.limit_widget.velocity_limit_changed.connect(
            self.control_panel.mode_widget.set_velocity_limit)
    
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

    def _init_sync_timer(self):
        """初始化参数同步定时器"""
        self.sync_timer = QTimer(self)
        self.sync_timer.timeout.connect(self._on_sync_timer)
        # 默认10秒间隔
        interval = self.control_panel.get_sync_interval()
        self.sync_timer.setInterval(interval * 1000)
        # 如果启用了自动同步，启动定时器
        if self.control_panel.is_auto_sync_enabled():
            self.sync_timer.start()

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
            },
            'sync': {
                'enabled': self.control_panel.sync_widget.is_auto_sync_enabled(),
                'interval': self.control_panel.sync_widget.get_sync_interval()
            }
        }

        config_path = os.path.join(os.path.dirname(__file__), 'config.json')
        try:
            with open(config_path, 'w', encoding='utf-8') as f:
                json.dump(config, f, indent=2)
            self.log_widget.append_log("配置已保存", "green")
        except Exception as e:
            self.log_widget.append_log(f"保存配置失败: {e}", "red")

    def _apply_user_config(self):
        """应用用户配置到界面"""
        try:
            # 应用PID参数
            vel = self.user_config.get('pid_velocity', {})
            self.control_panel.pid_vel.set_values(
                vel.get('p', 0.3),
                vel.get('i', 1.8),
                vel.get('d', 0.0),
                vel.get('lpf', 0.3)
            )

            ang = self.user_config.get('pid_angle', {})
            self.control_panel.pid_ang.set_values(
                ang.get('p', 20.0),
                ang.get('i', 0.0),
                ang.get('d', 0.0),
                ang.get('lpf', 0.05)
            )

            iq = self.user_config.get('pid_current_q', {})
            self.control_panel.pid_iq.set_values(
                iq.get('p', 1.0),
                iq.get('i', 3.0),
                iq.get('d', 0.0),
                iq.get('lpf', 0.005)
            )

            id_params = self.user_config.get('pid_current_d', {})
            self.control_panel.pid_id.set_values(
                id_params.get('p', 1.0),
                id_params.get('i', 3.0),
                id_params.get('d', 0.0),
                id_params.get('lpf', 0.005)
            )

            # 应用限制参数
            lim = self.user_config.get('limits', {})
            self.control_panel.limit_widget.volt_spin.setValue(lim.get('voltage', 11.0))
            vel_limit = lim.get('velocity', 50.0)
            self.control_panel.limit_widget.vel_spin.setValue(vel_limit)
            # 更新目标速度范围
            self.control_panel.mode_widget.set_velocity_limit(vel_limit)
            self.control_panel.limit_widget.curr_spin.setValue(lim.get('current', 3.0))

            # 应用同步设置
            sync = self.user_config.get('sync', {})
            self.control_panel.sync_widget.auto_sync_checkbox.setChecked(sync.get('enabled', True))
            self.control_panel.sync_widget.interval_spin.setValue(sync.get('interval', 10))

            self.log_widget.append_log("用户配置已应用", "green")
        except Exception as e:
            self.log_widget.append_log(f"应用用户配置失败: {e}", "red")

    def _load_config(self):
        """加载运行时配置文件（config.json）"""
        config_path = os.path.join(os.path.dirname(__file__), 'config.json')
        if not os.path.exists(config_path):
            return

        try:
            with open(config_path, 'r', encoding='utf-8') as f:
                config = json.load(f)

            # 加载PID参数（覆盖用户配置）
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
                vel_limit = lim.get('vel', 50.0)
                self.control_panel.limit_widget.vel_spin.setValue(vel_limit)
                # 更新目标速度范围
                self.control_panel.mode_widget.set_velocity_limit(vel_limit)
                self.control_panel.limit_widget.curr_spin.setValue(lim.get('curr', 3.0))

            # 加载同步设置
            if 'sync' in config:
                sync = config['sync']
                self.control_panel.sync_widget.auto_sync_checkbox.setChecked(sync.get('enabled', True))
                self.control_panel.sync_widget.interval_spin.setValue(sync.get('interval', 10))

            self.log_widget.append_log("运行时配置已加载", "green")
        except Exception as e:
            self.log_widget.append_log(f"加载运行时配置失败: {e}", "red")

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

    def _on_sync_timer(self):
        """定时器触发，执行参数同步"""
        if not self.serial_manager.is_connected():
            return

        if not self.control_panel.is_auto_sync_enabled():
            return

        self._sync_params_now()

    def _sync_params_now(self):
        """立即同步所有参数到下位机"""
        if not self.serial_manager.is_connected():
            self.log_widget.append_log("同步失败: 串口未连接", "red")
            self.control_panel.set_sync_status("未连接", "red")
            return

        self.log_widget.append_log("开始同步参数...", "blue")
        self.control_panel.set_sync_status("同步中...", "orange")

        params = self.control_panel.get_all_params()

        # 同步PID参数
        commands = []

        # 速度环PID
        vel = params['pid_vel']
        commands.append(f"SET_PID,VEL,{vel['p']},{vel['i']},{vel['d']},{vel['lpf']}")

        # 位置环PID
        ang = params['pid_ang']
        commands.append(f"SET_PID,ANG,{ang['p']},{ang['i']},{ang['d']},{ang['lpf']}")

        # Q轴电流环PID
        iq = params['pid_iq']
        commands.append(f"SET_PID,IQ,{iq['p']},{iq['i']},{iq['d']},{iq['lpf']}")

        # D轴电流环PID
        id_params = params['pid_id']
        commands.append(f"SET_PID,ID,{id_params['p']},{id_params['i']},{id_params['d']},{id_params['lpf']}")

        # 限制参数
        lim = params['limits']
        commands.append(f"SET_LIMIT,VOLT,{lim['volt']}")
        commands.append(f"SET_LIMIT,VEL,{lim['vel']}")
        commands.append(f"SET_LIMIT,CURR,{lim['curr']}")

        # 发送所有命令
        success_count = 0
        for cmd in commands:
            if self.serial_manager.send(cmd):
                success_count += 1
                self.msleep(50)  # 每条命令间隔50ms，避免下位机处理不过来
            else:
                self.log_widget.append_log(f"发送失败: {cmd}", "red")

        if success_count == len(commands):
            self.log_widget.append_log(f"参数同步完成 ({success_count}/{len(commands)})", "green")
            self.control_panel.set_sync_status("同步成功", "green")

            # 同步后验证参数
            self._verify_params_after_sync(params)
        else:
            self.log_widget.append_log(f"参数同步部分失败 ({success_count}/{len(commands)})", "orange")
            self.control_panel.set_sync_status("部分失败", "orange")

    def msleep(self, ms: int):
        """毫秒级延迟"""
        from PyQt5.QtCore import QEventLoop
        loop = QEventLoop()
        QTimer.singleShot(ms, loop.quit)
        loop.exec_()

    def _verify_params_after_sync(self, expected_params: dict):
        """同步后验证所有参数"""
        # 等待一段时间让下位机处理完命令
        self.msleep(200)

        # 临时存储验证状态
        self._verify_expected_params = expected_params
        self._verify_received_pids = {}
        self._verify_received_limits = None

        # 连接PID和LIMIT信号
        self.protocol_parser.pid_received.connect(self._on_pid_data_for_verify)
        self.protocol_parser.limit_received.connect(self._on_limit_data_for_verify)

        # 设置超时定时器
        self._verify_timeout_timer = QTimer(self)
        self._verify_timeout_timer.setSingleShot(True)
        self._verify_timeout_timer.timeout.connect(self._on_verify_timeout)
        self._verify_timeout_timer.start(3000)  # 3秒超时

        self.log_widget.append_log("正在查询下位机参数...", "blue")

        # 发送查询命令
        self.serial_manager.send("GET_LIMIT,")
        self.msleep(50)
        # 注意：GET_PID不带参数会返回所有PID参数（4条）
        # 为了简化，我们分别查询
        self.serial_manager.send("GET_PID,VEL")
        self.msleep(50)
        self.serial_manager.send("GET_PID,ANG")
        self.msleep(50)
        self.serial_manager.send("GET_PID,IQ")
        self.msleep(50)
        self.serial_manager.send("GET_PID,ID")

    def _on_pid_data_for_verify(self, pid_data):
        """接收到PID数据，用于验证"""
        # 存储接收到的PID数据
        self._verify_received_pids[pid_data.type] = pid_data

        # 检查是否收集完所有数据
        self._check_verify_complete()

    def _on_limit_data_for_verify(self, limit_data):
        """接收到LIMIT数据，用于验证"""
        # 存储接收到的LIMIT数据
        self._verify_received_limits = limit_data

        # 检查是否收集完所有数据
        self._check_verify_complete()

    def _check_verify_complete(self):
        """检查是否收集完所有验证数据"""
        # 需要收集：4个PID + 1个LIMIT
        if len(self._verify_received_pids) >= 4 and self._verify_received_limits is not None:
            # 停止超时定时器
            if hasattr(self, '_verify_timeout_timer'):
                self._verify_timeout_timer.stop()

            # 断开临时连接
            try:
                self.protocol_parser.pid_received.disconnect(self._on_pid_data_for_verify)
                self.protocol_parser.limit_received.disconnect(self._on_limit_data_for_verify)
            except:
                pass

            # 显示验证结果
            self._show_verify_results()

    def _show_verify_results(self):
        """显示验证结果"""
        expected = getattr(self, '_verify_expected_params', {})
        received_pids = getattr(self, '_verify_received_pids', {})
        received_limits = getattr(self, '_verify_received_limits', None)

        self.log_widget.append_log("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━", "blue")
        self.log_widget.append_log("📊 参数验证结果：", "blue")
        self.log_widget.append_log("", "blue")

        # 验证PID参数
        self.log_widget.append_log("🔧 PID参数：", "blue")

        pid_types = [
            ('VEL', '速度环', 'pid_vel'),
            ('ANG', '位置环', 'pid_ang'),
            ('IQ', 'Q轴电流环', 'pid_iq'),
            ('ID', 'D轴电流环', 'pid_id')
        ]

        for pid_type, name, key in pid_types:
            if pid_type in received_pids:
                exp = expected.get(key, {})
                act = received_pids[pid_type]

                p_match = abs(exp.get('p', 0) - act.p) < 0.001
                i_match = abs(exp.get('i', 0) - act.i) < 0.001
                d_match = abs(exp.get('d', 0) - act.d) < 0.001
                lpf_match = abs(exp.get('lpf', 0) - act.ramp) < 0.001

                all_match = p_match and i_match and d_match and lpf_match
                status = "✅" if all_match else "❌"
                color = "green" if all_match else "red"

                self.log_widget.append_log(f"  {name} {status}", color)
                if not all_match:
                    if not p_match:
                        self.log_widget.append_log(f"    P: 设置={exp.get('p', 0):.6f}, 实际={act.p:.6f}", "red")
                    if not i_match:
                        self.log_widget.append_log(f"    I: 设置={exp.get('i', 0):.6f}, 实际={act.i:.6f}", "red")
                    if not d_match:
                        self.log_widget.append_log(f"    D: 设置={exp.get('d', 0):.6f}, 实际={act.d:.6f}", "red")
                    if not lpf_match:
                        self.log_widget.append_log(f"    LPF: 设置={exp.get('lpf', 0):.6f}, 实际={act.ramp:.6f}", "red")
            else:
                self.log_widget.append_log(f"  {name} ⚠️ 未收到数据", "orange")

        self.log_widget.append_log("", "blue")

        # 验证限制参数
        self.log_widget.append_log("⚙️ 限制参数：", "blue")

        if received_limits:
            exp_lim = expected.get('limits', {})

            volt_match = abs(exp_lim.get('volt', 0) - received_limits.volt) < 0.01
            vel_match = abs(exp_lim.get('vel', 0) - received_limits.vel) < 0.01
            curr_match = abs(exp_lim.get('curr', 0) - received_limits.curr) < 0.01

            self.log_widget.append_log(
                f"  电压限制: 设置={exp_lim.get('volt', 0):.2f}V, 实际={received_limits.volt:.2f}V {'✅' if volt_match else '❌'}",
                "green" if volt_match else "red"
            )
            self.log_widget.append_log(
                f"  速度限制: 设置={exp_lim.get('vel', 0):.2f}rad/s, 实际={received_limits.vel:.2f}rad/s {'✅' if vel_match else '❌'}",
                "green" if vel_match else "red"
            )
            self.log_widget.append_log(
                f"  电流限制: 设置={exp_lim.get('curr', 0):.3f}A, 实际={received_limits.curr:.3f}A {'✅' if curr_match else '❌'}",
                "green" if curr_match else "red"
            )
        else:
            self.log_widget.append_log("  ⚠️ 未收到限制参数数据", "orange")

        self.log_widget.append_log("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━", "blue")

    def _on_verify_timeout(self):
        """验证超时"""
        # 断开临时连接
        try:
            self.protocol_parser.pid_received.disconnect(self._on_pid_data_for_verify)
            self.protocol_parser.limit_received.disconnect(self._on_limit_data_for_verify)
        except:
            pass

        received_pids = getattr(self, '_verify_received_pids', {})
        received_limits = getattr(self, '_verify_received_limits', None)

        self.log_widget.append_log("⚠️ 验证超时：3秒内未收到完整数据", "orange")
        self.log_widget.append_log(f"  已收到PID数据: {len(received_pids)}/4", "orange")
        self.log_widget.append_log(f"  已收到LIMIT数据: {'是' if received_limits else '否'}", "orange")
        self.log_widget.append_log("💡 可能原因：下位机未响应GET命令或串口通信异常", "orange")

        # 如果收到部分数据，也显示验证结果
        if len(received_pids) > 0 or received_limits is not None:
            self._show_verify_results()

    def _on_sync_enabled_changed(self, enabled: bool):
        """自动同步开关状态改变"""
        if enabled:
            self.sync_timer.start()
            self.log_widget.append_log("自动同步已启用", "green")
        else:
            self.sync_timer.stop()
            self.log_widget.append_log("自动同步已禁用", "orange")

    def _on_sync_interval_changed(self, interval: int):
        """同步间隔改变"""
        self.sync_timer.setInterval(interval * 1000)
        self.log_widget.append_log(f"同步间隔已设置为 {interval} 秒", "blue")
        # 如果定时器正在运行，重启以应用新间隔
        if self.sync_timer.isActive():
            self.sync_timer.stop()
            self.sync_timer.start()

    def closeEvent(self, event):
        """窗口关闭事件"""
        # 停止定时器
        if hasattr(self, 'sync_timer'):
            self.sync_timer.stop()
        # 断开串口
        self.serial_manager.disconnect()
        # 保存配置
        self._save_config()
        event.accept()

