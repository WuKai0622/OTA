# -*- coding: utf-8 -*-
"""
OTA 上位机入口（PySide6）。

启动：
    py tools/host_qt/main.py

通道：
    UART 串口   —— 直连 boot 的 USART1，走帧协议（当前唯一可联调的通道）
    ETH 网口    —— TCP client 连 boot 侧 lwip/tcp server（界面先行）
    wifi 模组   —— TCP client 连 wifi 模组透传目标（界面先行）
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
