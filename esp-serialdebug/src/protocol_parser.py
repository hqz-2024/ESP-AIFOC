"""
协议解析模块
负责解析下位机发送的数据帧
"""
from dataclasses import dataclass
from typing import Optional, Union
from PyQt5.QtCore import QObject, pyqtSignal


@dataclass
class SensorData:
    """传感器数据"""
    raw_angle: int = 0          # 原始角度值
    raw_max: int = 4096         # 满量程计数
    deg: float = 0.0            # 当前角度（度）
    status: str = "0x00"        # 状态字节
    agc: int = 0                # AGC值
    magnitude: int = 0          # 磁场幅值
    angle_rad: float = 0.0      # 累计角度（弧度）
    angle_deg_total: float = 0.0  # 累计角度（度）
    vel_rad_s: float = 0.0      # 角速度（rad/s）


@dataclass
class MotorData:
    """电机数据"""
    motor_angle_rad: float = 0.0    # 电机当前机械角（rad）
    motor_deg_total: float = 0.0    # 电机当前角度（deg）
    motor_vel_rad_s: float = 0.0    # 电机实际速度（rad/s）
    target_vel_rad_s: float = 0.0   # 目标速度（rad/s）
    voltage_q: float = 0.0          # Q轴电压（V）
    voltage_d: float = 0.0          # D轴电压（V）
    current_sp: float = 0.0         # 当前设定电流（A）
    current_limit: float = 0.0      # 当前电流限制（A）


@dataclass
class CurrentData:
    """电流数据"""
    ia: float = 0.0     # Phase A电流（A）
    ib: float = 0.0     # Phase B电流（A）
    ic: float = 0.0     # Phase C电流（A）
    iq: float = 0.0     # Q轴电流（A）
    id: float = 0.0     # D轴电流（A）


@dataclass
class AckData:
    """ACK响应数据"""
    command: str = ""       # 命令名称
    success: bool = False   # 是否成功
    error_code: str = ""    # 错误码（如果失败）


class ProtocolParser(QObject):
    """协议解析器"""
    sensor_received = pyqtSignal(object)   # 接收到传感器数据
    motor_received = pyqtSignal(object)    # 接收到电机数据
    current_received = pyqtSignal(object)  # 接收到电流数据
    ack_received = pyqtSignal(object)      # 接收到ACK响应
    raw_data = pyqtSignal(str)             # 原始数据（用于日志显示）
    parse_error = pyqtSignal(str)          # 解析错误

    def __init__(self):
        super().__init__()

    def parse_line(self, line: str):
        """
        解析一行数据
        :param line: 一行文本数据
        """
        self.raw_data.emit(line)
        
        if not line or ',' not in line:
            return
        
        try:
            parts = line.split(',')
            msg_type = parts[0].strip().upper()
            
            if msg_type == 'SENSOR':
                self._parse_sensor(parts[1:])
            elif msg_type == 'MOTOR':
                self._parse_motor(parts[1:])
            elif msg_type == 'CURRENT':
                self._parse_current(parts[1:])
            elif msg_type == 'ACK':
                self._parse_ack(parts[1:])
            # 其他类型暂时忽略
        except Exception as e:
            self.parse_error.emit(f"解析错误: {line} - {str(e)}")

    def _parse_sensor(self, fields: list):
        """解析传感器数据帧"""
        if len(fields) < 9:
            self.parse_error.emit(f"SENSOR帧字段不足: {len(fields)}")
            return
        
        try:
            data = SensorData(
                raw_angle=int(fields[0]),
                raw_max=int(fields[1]),
                deg=float(fields[2]),
                status=fields[3].strip(),
                agc=int(fields[4]),
                magnitude=int(fields[5]),
                angle_rad=float(fields[6]),
                angle_deg_total=float(fields[7]),
                vel_rad_s=float(fields[8])
            )
            self.sensor_received.emit(data)
        except (ValueError, IndexError) as e:
            self.parse_error.emit(f"SENSOR解析失败: {str(e)}")

    def _parse_motor(self, fields: list):
        """解析电机数据帧"""
        if len(fields) < 8:
            self.parse_error.emit(f"MOTOR帧字段不足: {len(fields)}")
            return
        
        try:
            data = MotorData(
                motor_angle_rad=float(fields[0]),
                motor_deg_total=float(fields[1]),
                motor_vel_rad_s=float(fields[2]),
                target_vel_rad_s=float(fields[3]),
                voltage_q=float(fields[4]),
                voltage_d=float(fields[5]),
                current_sp=float(fields[6]),
                current_limit=float(fields[7])
            )
            self.motor_received.emit(data)
        except (ValueError, IndexError) as e:
            self.parse_error.emit(f"MOTOR解析失败: {str(e)}")

    def _parse_current(self, fields: list):
        """解析电流数据帧"""
        if len(fields) < 5:
            self.parse_error.emit(f"CURRENT帧字段不足: {len(fields)}")
            return
        
        try:
            data = CurrentData(
                ia=float(fields[0]),
                ib=float(fields[1]),
                ic=float(fields[2]),
                iq=float(fields[3]),
                id=float(fields[4])
            )
            self.current_received.emit(data)
        except (ValueError, IndexError) as e:
            self.parse_error.emit(f"CURRENT解析失败: {str(e)}")

    def _parse_ack(self, fields: list):
        """解析ACK响应"""
        if len(fields) < 2:
            return
        
        try:
            cmd = fields[0].strip()
            status = fields[1].strip().upper()
            error_code = fields[2].strip() if len(fields) > 2 else ""
            
            data = AckData(
                command=cmd,
                success=(status == "OK"),
                error_code=error_code
            )
            self.ack_received.emit(data)
        except Exception as e:
            self.parse_error.emit(f"ACK解析失败: {str(e)}")

