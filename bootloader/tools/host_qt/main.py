# -*- coding: utf-8 -*-
"""
OTA 上位机入口（PySide6）。

启动：
    py tools/host_qt/main.py

通道（三条都走同一套帧协议，boot 侧汇入同一个接收环）：
    UART 串口   —— 直连 boot 的 USART3（有线直连 115200，挂蓝牙透传则降到 9600~19200）
    ETH 网口    —— TCP client 连 boot 侧 lwip/tcp server，默认 192.168.1.2:5001
    wifi 模组   —— TCP client 连 wifi 模组透传目标（设备侧仍是同一个 boot）
"""
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)                                    # host_qt 自身
sys.path.insert(0, os.path.dirname(_HERE))                   # tools 目录（ota_proto）

from PySide6.QtWidgets import QApplication
from mainwindow import MainWindow


def main():
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
