"""
FOC 串口数据分析工具
程序入口
"""
import sys
import os

# 添加src目录到路径
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# 修复conda环境下Qt插件路径问题
if hasattr(sys, 'base_prefix'):
    conda_prefix = os.environ.get('CONDA_PREFIX', sys.base_prefix)
    qt_plugin_path = os.path.join(conda_prefix, 'Library', 'plugins')
    if os.path.exists(qt_plugin_path):
        os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = qt_plugin_path

from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import Qt
from main_window import MainWindow


def main():
    """程序入口函数"""
    # 启用高DPI支持
    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)
    
    app = QApplication(sys.argv)
    app.setApplicationName("FOC串口数据分析工具")
    app.setOrganizationName("FOC Debug")
    
    # 设置全局样式
    app.setStyle('Fusion')
    
    # 创建主窗口
    window = MainWindow()
    window.show()
    
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()

