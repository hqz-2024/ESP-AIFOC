"""
绘图组件模块
使用pyqtgraph实现实时数据绘图
"""
import pyqtgraph as pg
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QTabWidget, QHBoxLayout, QPushButton
from PyQt5.QtCore import QTimer
from data_model import DataModel


# 设置pyqtgraph全局配置
pg.setConfigOptions(antialias=True)


class PlotCanvas(pg.PlotWidget):
    """单个绘图画布"""
    
    def __init__(self, title: str = "", y_label: str = "", parent=None):
        super().__init__(parent)
        self.setTitle(title)
        self.setLabel('left', y_label)
        self.setLabel('bottom', '时间', 's')
        self.showGrid(x=True, y=True, alpha=0.3)
        self.addLegend()
        self.curves = {}
        
        # 颜色表
        self.colors = [
            (255, 100, 100),   # 红
            (100, 255, 100),   # 绿
            (100, 100, 255),   # 蓝
            (255, 255, 100),   # 黄
            (255, 100, 255),   # 紫
            (100, 255, 255),   # 青
        ]
        self._color_index = 0
    
    def add_curve(self, name: str, color=None) -> pg.PlotDataItem:
        """添加一条曲线"""
        if color is None:
            color = self.colors[self._color_index % len(self.colors)]
            self._color_index += 1
        
        pen = pg.mkPen(color=color, width=2)
        curve = self.plot([], [], name=name, pen=pen)
        self.curves[name] = curve
        return curve
    
    def update_curve(self, name: str, x_data, y_data):
        """更新曲线数据"""
        if name in self.curves:
            self.curves[name].setData(x_data, y_data)


class PlotWidget(QWidget):
    """绘图组件，包含多个绘图区域"""
    
    def __init__(self, data_model: DataModel, parent=None):
        super().__init__(parent)
        self.data_model = data_model
        self._paused = False
        self._init_ui()
        self._init_timer()
    
    def _init_ui(self):
        """初始化UI"""
        layout = QVBoxLayout(self)
        
        # 控制按钮
        btn_layout = QHBoxLayout()
        self.pause_btn = QPushButton("暂停")
        self.pause_btn.clicked.connect(self._toggle_pause)
        self.clear_btn = QPushButton("清空")
        self.clear_btn.clicked.connect(self._clear_data)
        btn_layout.addWidget(self.pause_btn)
        btn_layout.addWidget(self.clear_btn)
        btn_layout.addStretch()
        layout.addLayout(btn_layout)
        
        # 创建Tab页
        self.tab_widget = QTabWidget()
        layout.addWidget(self.tab_widget)
        
        # 角度/速度页
        self.angle_velocity_plot = self._create_angle_velocity_tab()
        self.tab_widget.addTab(self.angle_velocity_plot, "角度/速度")
        
        # 电流页
        self.current_plot = self._create_current_tab()
        self.tab_widget.addTab(self.current_plot, "电流")
        
        # 电压页
        self.voltage_plot = self._create_voltage_tab()
        self.tab_widget.addTab(self.voltage_plot, "电压")
        
        # 三相电流页
        self.phase_current_plot = self._create_phase_current_tab()
        self.tab_widget.addTab(self.phase_current_plot, "三相电流")
    
    def _create_angle_velocity_tab(self) -> QWidget:
        """创建角度/速度绘图页"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        # 角度图
        self.angle_canvas = PlotCanvas("角度", "角度 (°)")
        self.angle_canvas.add_curve("电机角度", (255, 100, 100))
        self.angle_canvas.add_curve("传感器角度", (100, 100, 255))
        layout.addWidget(self.angle_canvas)
        
        # 速度图
        self.velocity_canvas = PlotCanvas("速度", "速度 (rad/s)")
        self.velocity_canvas.add_curve("实际速度", (100, 255, 100))
        self.velocity_canvas.add_curve("目标速度", (255, 100, 100))
        layout.addWidget(self.velocity_canvas)
        
        return widget
    
    def _create_current_tab(self) -> QWidget:
        """创建电流绘图页（Q/D轴）"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        self.qd_current_canvas = PlotCanvas("Q/D轴电流", "电流 (A)")
        self.qd_current_canvas.add_curve("Iq (转矩)", (255, 100, 100))
        self.qd_current_canvas.add_curve("Id (励磁)", (100, 100, 255))
        layout.addWidget(self.qd_current_canvas)
        
        return widget
    
    def _create_voltage_tab(self) -> QWidget:
        """创建电压绘图页"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        self.voltage_canvas = PlotCanvas("Q/D轴电压", "电压 (V)")
        self.voltage_canvas.add_curve("Vq", (255, 100, 100))
        self.voltage_canvas.add_curve("Vd", (100, 100, 255))
        layout.addWidget(self.voltage_canvas)
        
        return widget
    
    def _create_phase_current_tab(self) -> QWidget:
        """创建三相电流绘图页"""
        widget = QWidget()
        layout = QVBoxLayout(widget)

        self.phase_canvas = PlotCanvas("三相电流", "电流 (A)")
        self.phase_canvas.add_curve("Ia", (255, 100, 100))
        self.phase_canvas.add_curve("Ib", (100, 255, 100))
        self.phase_canvas.add_curve("Ic", (100, 100, 255))
        layout.addWidget(self.phase_canvas)

        return widget

    def _init_timer(self):
        """初始化刷新定时器"""
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self._update_plots)
        self.update_timer.start(50)  # 50ms刷新一次（20Hz）

    def _toggle_pause(self):
        """切换暂停状态"""
        self._paused = not self._paused
        self.pause_btn.setText("继续" if self._paused else "暂停")

    def _clear_data(self):
        """清空数据"""
        self.data_model.clear_all()

    def _update_plots(self):
        """更新所有绘图"""
        if self._paused:
            return

        dm = self.data_model

        # 更新角度图
        t, v = dm.motor_angle.get_data()
        self.angle_canvas.update_curve("电机角度", t, v)
        t, v = dm.sensor_angle.get_data()
        self.angle_canvas.update_curve("传感器角度", t, v)

        # 更新速度图
        t, v = dm.motor_velocity.get_data()
        self.velocity_canvas.update_curve("实际速度", t, v)
        t, v = dm.target_velocity.get_data()
        self.velocity_canvas.update_curve("目标速度", t, v)

        # 更新Q/D电流图
        t, v = dm.current_iq.get_data()
        self.qd_current_canvas.update_curve("Iq (转矩)", t, v)
        t, v = dm.current_id.get_data()
        self.qd_current_canvas.update_curve("Id (励磁)", t, v)

        # 更新电压图
        t, v = dm.voltage_q.get_data()
        self.voltage_canvas.update_curve("Vq", t, v)
        t, v = dm.voltage_d.get_data()
        self.voltage_canvas.update_curve("Vd", t, v)

        # 更新三相电流图
        t, v = dm.current_ia.get_data()
        self.phase_canvas.update_curve("Ia", t, v)
        t, v = dm.current_ib.get_data()
        self.phase_canvas.update_curve("Ib", t, v)
        t, v = dm.current_ic.get_data()
        self.phase_canvas.update_curve("Ic", t, v)

