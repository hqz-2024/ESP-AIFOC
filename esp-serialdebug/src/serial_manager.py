"""
串口管理模块
负责串口的打开、关闭、数据收发
"""
import serial
import serial.tools.list_ports
from PyQt5.QtCore import QObject, QThread, pyqtSignal
from typing import Optional, List


class SerialReaderThread(QThread):
    """串口读取线程，持续读取串口数据"""
    data_received = pyqtSignal(str)  # 接收到一行数据时发出信号
    error_occurred = pyqtSignal(str)  # 发生错误时发出信号

    def __init__(self, serial_port: serial.Serial):
        super().__init__()
        self.serial_port = serial_port
        self._running = True
        self._buffer = ""

    def run(self):
        """线程主循环，持续读取串口数据"""
        while self._running and self.serial_port and self.serial_port.is_open:
            try:
                if self.serial_port.in_waiting > 0:
                    # 读取可用数据
                    data = self.serial_port.read(self.serial_port.in_waiting)
                    try:
                        text = data.decode('utf-8', errors='ignore')
                        self._buffer += text
                        # 按行分割
                        while '\n' in self._buffer:
                            line, self._buffer = self._buffer.split('\n', 1)
                            line = line.strip('\r')
                            if line:
                                self.data_received.emit(line)
                    except Exception as e:
                        self.error_occurred.emit(f"解码错误: {str(e)}")
                else:
                    self.msleep(1)  # 短暂休眠避免CPU占用过高
            except serial.SerialException as e:
                self.error_occurred.emit(f"串口读取错误: {str(e)}")
                break
            except Exception as e:
                self.error_occurred.emit(f"未知错误: {str(e)}")
                break

    def stop(self):
        """停止线程"""
        self._running = False
        self.wait(1000)


class SerialManager(QObject):
    """串口管理器，封装串口操作"""
    connected = pyqtSignal()           # 连接成功信号
    disconnected = pyqtSignal()        # 断开连接信号
    data_received = pyqtSignal(str)    # 接收数据信号
    error_occurred = pyqtSignal(str)   # 错误信号

    def __init__(self):
        super().__init__()
        self.serial_port: Optional[serial.Serial] = None
        self.reader_thread: Optional[SerialReaderThread] = None

    @staticmethod
    def list_ports() -> List[str]:
        """列举可用串口"""
        ports = serial.tools.list_ports.comports()
        return [port.device for port in ports]

    def connect(self, port: str, baudrate: int = 115200,
                bytesize: int = 8, parity: str = 'N', stopbits: float = 1) -> bool:
        """
        连接串口
        :param port: 串口号，如 'COM3'
        :param baudrate: 波特率
        :param bytesize: 数据位
        :param parity: 校验位 ('N', 'E', 'O')
        :param stopbits: 停止位
        :return: 是否连接成功
        """
        try:
            self.disconnect()  # 先断开已有连接
            
            self.serial_port = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=bytesize,
                parity=parity,
                stopbits=stopbits,
                timeout=0.1
            )
            
            # 启动读取线程
            self.reader_thread = SerialReaderThread(self.serial_port)
            self.reader_thread.data_received.connect(self.data_received)
            self.reader_thread.error_occurred.connect(self._on_error)
            self.reader_thread.start()
            
            self.connected.emit()
            return True
        except serial.SerialException as e:
            self.error_occurred.emit(f"连接失败: {str(e)}")
            return False

    def disconnect(self):
        """断开串口连接"""
        if self.reader_thread:
            self.reader_thread.stop()
            self.reader_thread = None
        
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            self.serial_port = None
            self.disconnected.emit()

    def is_connected(self) -> bool:
        """检查是否已连接"""
        return self.serial_port is not None and self.serial_port.is_open

    def send(self, data: str) -> bool:
        """
        发送数据
        :param data: 要发送的字符串（会自动添加\\r\\n）
        :return: 是否发送成功
        """
        if not self.is_connected():
            self.error_occurred.emit("串口未连接")
            return False
        
        try:
            # 确保以\r\n结尾
            if not data.endswith('\r\n'):
                data = data.rstrip('\r\n') + '\r\n'
            self.serial_port.write(data.encode('utf-8'))
            return True
        except serial.SerialException as e:
            self.error_occurred.emit(f"发送失败: {str(e)}")
            return False

    def _on_error(self, error_msg: str):
        """处理错误"""
        self.error_occurred.emit(error_msg)
        self.disconnect()

