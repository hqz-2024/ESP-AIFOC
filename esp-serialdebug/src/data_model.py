"""
数据模型层
管理历史数据缓存，供绘图使用
"""
import time
from collections import deque
from typing import Deque, List, Tuple
from dataclasses import dataclass, field
import numpy as np
from PyQt5.QtCore import QObject, pyqtSignal


@dataclass
class DataBuffer:
    """单个数据序列的缓冲区"""
    max_length: int = 2000
    timestamps: Deque[float] = field(default_factory=lambda: deque(maxlen=2000))
    values: Deque[float] = field(default_factory=lambda: deque(maxlen=2000))
    
    def __post_init__(self):
        self.timestamps = deque(maxlen=self.max_length)
        self.values = deque(maxlen=self.max_length)
    
    def append(self, value: float, timestamp: float = None):
        """添加数据点"""
        if timestamp is None:
            timestamp = time.time()
        self.timestamps.append(timestamp)
        self.values.append(value)
    
    def get_data(self) -> Tuple[np.ndarray, np.ndarray]:
        """获取数据数组"""
        if len(self.timestamps) == 0:
            return np.array([]), np.array([])
        
        t0 = self.timestamps[0] if self.timestamps else 0
        times = np.array([t - t0 for t in self.timestamps])
        values = np.array(list(self.values))
        return times, values
    
    def clear(self):
        """清空缓冲区"""
        self.timestamps.clear()
        self.values.clear()


class DataModel(QObject):
    """数据模型，管理所有数据缓存"""
    data_updated = pyqtSignal()  # 数据更新信号
    
    def __init__(self, max_points: int = 2000):
        super().__init__()
        self.max_points = max_points
        self._start_time = time.time()
        
        # 传感器数据
        self.sensor_angle = DataBuffer(max_points)      # 传感器角度（度）
        self.sensor_angle_rad = DataBuffer(max_points)  # 传感器角度（弧度）
        self.sensor_velocity = DataBuffer(max_points)   # 传感器速度
        self.sensor_agc = DataBuffer(max_points)        # AGC值
        self.sensor_magnitude = DataBuffer(max_points)  # 磁场幅值
        
        # 电机数据
        self.motor_angle = DataBuffer(max_points)       # 电机角度（度）
        self.motor_velocity = DataBuffer(max_points)    # 电机实际速度
        self.target_velocity = DataBuffer(max_points)   # 目标速度
        self.voltage_q = DataBuffer(max_points)         # Q轴电压
        self.voltage_d = DataBuffer(max_points)         # D轴电压
        
        # 电流数据
        self.current_ia = DataBuffer(max_points)        # Phase A电流
        self.current_ib = DataBuffer(max_points)        # Phase B电流
        self.current_ic = DataBuffer(max_points)        # Phase C电流
        self.current_iq = DataBuffer(max_points)        # Q轴电流
        self.current_id = DataBuffer(max_points)        # D轴电流
        
        # 最新数据值（用于显示）
        self.latest_sensor = None
        self.latest_motor = None
        self.latest_current = None

    def update_sensor(self, data):
        """更新传感器数据"""
        ts = time.time()
        self.sensor_angle.append(data.deg, ts)
        self.sensor_angle_rad.append(data.angle_rad, ts)
        self.sensor_velocity.append(data.vel_rad_s, ts)
        self.sensor_agc.append(data.agc, ts)
        self.sensor_magnitude.append(data.magnitude, ts)
        self.latest_sensor = data
        self.data_updated.emit()

    def update_motor(self, data):
        """更新电机数据"""
        ts = time.time()
        self.motor_angle.append(data.motor_deg_total, ts)
        self.motor_velocity.append(data.motor_vel_rad_s, ts)
        self.target_velocity.append(data.target_vel_rad_s, ts)
        self.voltage_q.append(data.voltage_q, ts)
        self.voltage_d.append(data.voltage_d, ts)
        self.latest_motor = data
        self.data_updated.emit()

    def update_current(self, data):
        """更新电流数据"""
        ts = time.time()
        self.current_ia.append(data.ia, ts)
        self.current_ib.append(data.ib, ts)
        self.current_ic.append(data.ic, ts)
        self.current_iq.append(data.iq, ts)
        self.current_id.append(data.id, ts)
        self.latest_current = data
        self.data_updated.emit()

    def clear_all(self):
        """清空所有缓冲区"""
        for attr_name in dir(self):
            attr = getattr(self, attr_name)
            if isinstance(attr, DataBuffer):
                attr.clear()
        self.latest_sensor = None
        self.latest_motor = None
        self.latest_current = None
        self._start_time = time.time()

