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


@dataclass
class PIDData:
    """PID参数数据"""
    type: str = ""          # PID类型（VEL/ANG/IQ/ID）
    p: float = 0.0          # P参数
    i: float = 0.0          # I参数
    d: float = 0.0          # D参数
    ramp: float = 0.0       # 输出斜坡


@dataclass
class LimitData:
    """限制参数数据"""
    volt: float = 0.0       # 电压限制（V）
    vel: float = 0.0        # 速度限制（rad/s）
    curr: float = 0.0       # 电流限制（A）


class ProtocolParser(QObject):
    """协议解析器"""
    sensor_received = pyqtSignal(object)   # 接收到传感器数据
    motor_received = pyqtSignal(object)    # 接收到电机数据
    current_received = pyqtSignal(object)  # 接收到电流数据
    ack_received = pyqtSignal(object)      # 接收到ACK响应
    pid_received = pyqtSignal(object)      # 接收到PID参数
    limit_received = pyqtSignal(object)    # 接收到限制参数
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
            elif msg_type == 'PID':
                self._parse_pid(parts[1:])
            elif msg_type == 'LIMIT':
                self._parse_limit(parts[1:])
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

    def _parse_pid(self, fields: list):
        """解析PID参数帧"""
        if len(fields) < 5:
            self.parse_error.emit(f"PID帧字段不足: {len(fields)}")
            return

        try:
            data = PIDData(
                type=fields[0].strip(),
                p=float(fields[1]),
                i=float(fields[2]),
                d=float(fields[3]),
                ramp=float(fields[4])
            )
            self.pid_received.emit(data)
        except (ValueError, IndexError) as e:
            self.parse_error.emit(f"PID解析失败: {str(e)}")

    def _parse_limit(self, fields: list):
        """解析限制参数帧"""
        try:
            # 支持两种格式：
            # 1. LIMIT,VOLT,12.0 (单个参数)
            # 2. LIMIT,VOLT,12.0,VEL,50.0,CURR,3.0 (所有参数)

            data = LimitData()

            i = 0
            while i < len(fields) - 1:
                param_type = fields[i].strip()
                param_value = float(fields[i + 1])

                if param_type == "VOLT":
                    data.volt = param_value
                elif param_type == "VEL":
                    data.vel = param_value
                elif param_type == "CURR":
                    data.curr = param_value

                i += 2

            self.limit_received.emit(data)
        except (ValueError, IndexError) as e:
            self.parse_error.emit(f"LIMIT解析失败: {str(e)}")

