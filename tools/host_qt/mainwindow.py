# -*- coding: utf-8 -*-
"""上位机主窗口：三通道选项卡 + 固件选择 + 升级控制 + 日志。"""
import os
import sys

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (QApplication, QWidget, QMainWindow, QGroupBox,
                               QPushButton, QVBoxLayout, QHBoxLayout, QGridLayout,
                               QLabel, QLineEdit, QComboBox, QPlainTextEdit,
                               QProgressBar, QFileDialog, QTabWidget, QMessageBox)

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from host_qt.transport import SerialTransport, TcpTransport
from host_qt.ota_worker import OtaWorker


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("OTA 上位机 —— UART / ETH / wifi 三通道")
        self.resize(680, 560)
        self._worker = None

        # ---------- 通道选项卡 ----------
        self.tabs = QTabWidget()
        self.serial_tab = SerialTab()
        self.eth_tab = NetTab("ETH (lwip/tcp)", "eth_host", "eth_port")
        self.wifi_tab = NetTab("wifi 模组 (SPI+OTA)", "wifi_host", "wifi_port")
        self.tabs.addTab(self.serial_tab, "UART 串口")
        self.tabs.addTab(self.eth_tab, "ETH 网口")
        self.tabs.addTab(self.wifi_tab, "wifi 模组")

        # ---------- 固件选择 ----------
        fw_box = QGroupBox("固件")
        fw_lay = QHBoxLayout()
        self.fw_path = QLineEdit()
        self.fw_path.setPlaceholderText("选择要升级的 .bin 固件")
        btn_browse = QPushButton("浏览...")
        btn_browse.clicked.connect(self._browse)
        fw_lay.addWidget(self.fw_path, 1)
        fw_lay.addWidget(btn_browse)
        fw_box.setLayout(fw_lay)

        # ---------- 控制与进度 ----------
        ctl = QHBoxLayout()
        self.btn_start = QPushButton("开始升级")
        self.btn_start.clicked.connect(self._start)
        self.btn_stop = QPushButton("中止")
        self.btn_stop.setEnabled(False)
        self.btn_stop.clicked.connect(self._stop)
        self.btn_clear = QPushButton("清空日志")
        self.btn_clear.clicked.connect(self._clear_log)
        ctl.addWidget(self.btn_start)
        ctl.addWidget(self.btn_stop)
        ctl.addStretch(1)
        ctl.addWidget(self.btn_clear)

        self.progress = QProgressBar()
        self.progress.setRange(0, 100)

        # ---------- 日志 ----------
        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)

        root = QVBoxLayout()
        root.addWidget(self.tabs)
        root.addWidget(fw_box)
        root.addLayout(ctl)
        root.addWidget(self.progress)
        root.addWidget(QLabel("日志："), 0)
        root.addWidget(self.log_view, 1)

        w = QWidget()
        w.setLayout(root)
        self.setCentralWidget(w)
        self.log("就绪。选择通道参数与固件后点“开始升级”。")

    # ---------- 日志/进度 ----------
    def log(self, msg: str):
        self.log_view.appendPlainText(msg)

    def _log_slot(self, msg: str):
        self.log(msg)

    def _clear_log(self):
        self.log_view.clear()

    def _browse(self):
        default = os.path.normpath(os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            "..", "..", "MDK-ARM", "SLOTA.bin"))
        path, _ = QFileDialog.getOpenFileName(self, "选择固件", default, "固件 (*.bin)")
        if path:
            self.fw_path.setText(path)

    # ---------- 升级控制 ----------
    def _make_transport(self):
        idx = self.tabs.currentIndex()
        if idx == 0:
            tab = self.serial_tab
            port, baud = tab.current_config()
            if not port:
                QMessageBox.warning(self, "提示", "请选择串口 COM 口")
                return None
            return SerialTransport(port, baud)
        elif idx == 1:
            cfg = self.eth_tab.current_config()
            return TcpTransport(*cfg)
        else:
            cfg = self.wifi_tab.current_config()
            return TcpTransport(*cfg)

    def _start(self):
        if self._worker and self._worker.isRunning():
            return
        bin_path = self.fw_path.text().strip()
        if not bin_path or not os.path.exists(bin_path):
            QMessageBox.warning(self, "提示", "请先选择存在的固件文件")
            return
        transport = self._make_transport()
        if transport is None:
            return
        if isinstance(transport, SerialTransport):
            self.log("通道=UART  COM=%s @%d" % (transport.port, transport.baud))
        else:
            self.log("通道=%s  %s:%d" % (transport.label, transport.host, transport.port))

        self._worker = OtaWorker(transport, bin_path)
        self._worker.log.connect(self._log_slot)
        self._worker.err.connect(lambda m: self._log_slot("[错误] " + m))
        self._worker.progress.connect(self._on_progress)
        self._worker.done.connect(self._on_done)
        self.btn_start.setEnabled(False)
        self.btn_stop.setEnabled(True)
        self.progress.setValue(0)
        self._worker.start()

    def _stop(self):
        if self._worker:
            self._worker.stop()

    def _on_progress(self, sent, total, text):
        self.progress.setValue(0 if total == 0 else int(sent * 100 / total))
        self.log(text)

    def _on_done(self, ok: bool, summary: str):
        self.btn_start.setEnabled(True)
        self.btn_stop.setEnabled(False)
        self.log("== %s（%s）==" % ("成功" if ok else "失败", summary))
        if not ok:
            QMessageBox.warning(self, "升级结果", "失败：" + summary)
        else:
            QMessageBox.information(self, "升级结果", "成功：" + summary)


class SerialTab(QWidget):
    def __init__(self):
        super().__init__()
        lay = QGridLayout()
        lay.addWidget(QLabel("串口:"), 0, 0)
        self.combo = QComboBox()
        self.combo.setEditable(True)
        self._refresh_ports()
        btn_refresh = QPushButton("刷新")
        btn_refresh.clicked.connect(self._refresh_ports)
        lay.addWidget(self.combo, 0, 1)
        lay.addWidget(btn_refresh, 0, 2)
        lay.addWidget(QLabel("波特率:"), 1, 0)
        self.baud = QLineEdit("115200")
        lay.addWidget(self.baud, 1, 1)
        note = QLabel("说明：当前 boot 侧帧协议走此 UART 通道（115200 8N1）。")
        note.setStyleSheet("color:#888;")
        lay.addWidget(note, 2, 0, 1, 3)
        self.setLayout(lay)

    def _refresh_ports(self):
        import serial.tools.list_ports
        cur = self.combo.currentText()
        self.combo.clear()
        for p in serial.tools.list_ports.comports():
            self.combo.addItem("%s - %s" % (p.device, p.description or ""))
        if cur:
            self.combo.setCurrentText(cur)

    def current_config(self):
        text = self.combo.currentText().strip()
        port = text.split(" - ")[0] if text else ""
        try:
            baud = int(self.baud.text().strip() or "115200")
        except ValueError:
            baud = 115200
        return port, baud


class NetTab(QWidget):
    """ETH 与 wifi 模组共用：目标是某 IP:port 的 TCP 服务器（boot 侧后续挂网口）。"""
    def __init__(self, title: str, host_key: str, port_key: str):
        super().__init__()
        self.title = title
        lay = QGridLayout()
        lay.addWidget(QLabel("目标 IP:"), 0, 0)
        self.host = QLineEdit("192.168.1.10")
        lay.addWidget(self.host, 0, 1)
        lay.addWidget(QLabel("端口:"), 1, 0)
        self.port = QLineEdit("8080")
        lay.addWidget(self.port, 1, 1)
        note = QLabel("本通道传输层已就绪（TCP client，复用同帧协议）。\n"
                      "boot 侧 ETH(lwip) / wifi(SPI) 驱动接入后可直接使用。")
        note.setStyleSheet("color:#888;")
        lay.addWidget(note, 2, 0, 1, 2)
        self.setLayout(lay)

    def current_config(self):
        host = self.host.text().strip() or "127.0.0.1"
        try:
            port = int(self.port.text().strip() or "8080")
        except ValueError:
            port = 8080
        return host, port
