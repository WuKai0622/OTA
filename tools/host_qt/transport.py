# -*- coding: utf-8 -*-
"""
统一传输通道抽象 —— UART / ETH(TCP) / wifi模组(TCP) 三个选项卡都落到同一接口。
当前 boot 侧只有 UART；ETH/wifi 通道等 boot 侧网口驱动接入后直接复用同帧协议。
"""


class Transport:
    """三通道统一接口。所有方法在升级 Worker 线程中被调用。"""
    def open(self):
        raise NotImplementedError

    def send(self, data: bytes):
        raise NotImplementedError

    def recv(self, n: int, timeout: float) -> bytes:
        """最多读 n 字节，超时返回已读部分（可能为空 bytes）。"""
        raise NotImplementedError

    def flush_rx(self):
        raise NotImplementedError

    def close(self):
        raise NotImplementedError


class SerialTransport(Transport):
    def __init__(self, port: str, baud: int = 115200):
        self.port = port
        self.baud = baud
        self._ser = None

    def open(self):
        import serial
        self._ser = serial.Serial(self.port, self.baud, timeout=0.05)
        self._ser.reset_input_buffer()

    def send(self, data: bytes):
        self._ser.write(data)

    def recv(self, n: int, timeout: float) -> bytes:
        self._ser.timeout = timeout
        return self._ser.read(n)

    def flush_rx(self):
        self._ser.reset_input_buffer()

    def close(self):
        if self._ser is not None:
            try:
                self._ser.close()
            finally:
                self._ser = None


class TcpTransport(Transport):
    """ETH(lwip/tcp) 与 wifi模组透传共用：PC 作为 TCP client 连目标 IP:port。"""
    def __init__(self, host: str, port: int, label: str = "TCP"):
        self.host = host
        self.port = int(port)
        self.label = label
        self._sock = None

    def open(self):
        import socket
        s = socket.create_connection((self.host, self.port), timeout=3.0)
        s.settimeout(0.05)
        self._sock = s

    def send(self, data: bytes):
        self._sock.sendall(data)

    def recv(self, n: int, timeout: float) -> bytes:
        self._sock.settimeout(timeout)
        try:
            return self._sock.recv(n)
        except socket.timeout:
            return b""
        except OSError:
            return b""

    def flush_rx(self):
        try:
            import socket
            self._sock.settimeout(0.05)
            while True:
                if not self._sock.recv(4096):
                    break
        except Exception:
            pass

    def close(self):
        if self._sock is not None:
            try:
                self._sock.close()
            finally:
                self._sock = None
