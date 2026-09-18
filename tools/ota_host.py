# -*- coding: utf-8 -*-
"""
OTA 无界面联调 CLI —— 对 Bootloader 帧协议做真机验证。

用法：
    py tools/ota_host.py <COM> <bin文件>                     # UART 串口
    py tools/ota_host.py --tcp 192.168.1.2:5001 <bin文件>   # ETH 网口（连 boot 的 TCP 服务器）
    py tools/ota_host.py --tcp 192.168.1.50:5001 <bin文件>  # wifi 模组透传

    <bin文件> 后还可加：
      --probe        只发 OTA_START 探测通道是否通（不发数据）
      --inject-err   首帧注入坏 CRC（验证重发）
      --oversize     伪造越界固件（验证上限拒绝）
      --selftest     仅做 CRC 一致性自检（不连设备）

依赖：pyserial（仅串口模式需要）。串口 8N1，波特率见 --baud。
"""
import argparse
import os
import random
import socket
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ota_proto as P


# ==================== 通道适配 ====================
# run_ota()/wait_ack() 只认 poll()/send()/close() 三个方法，不关心底下是串口还是 TCP。

class _SerialLink:
    def __init__(self, port, baud=115200):
        import serial
        self._ser = serial.Serial(port, baud, timeout=0.05)
        self._ser.reset_input_buffer()

    def poll(self) -> bytes:
        """返回当前可读的全部字节；无数据返回 b""，不阻塞。"""
        n = self._ser.in_waiting
        return self._ser.read(n) if n > 0 else b""

    def send(self, data: bytes):
        self._ser.write(data)

    def close(self):
        try:
            self._ser.close()
        except Exception:
            pass


class _TcpLink:
    """ETH 直连 / wifi 模组透传。设备做 TCP 服务器，PC 做 client。

    ★ socket 必须是模块级导入：写成函数内 import 的话，下面那句 except socket.timeout
      在求值时就会抛 NameError —— 而它恰好落在"每次读超时"这条最常走的路径上。
      GUI 的 transport.py 就踩过这个坑（同样是函数内 import）。
    """
    def __init__(self, host, port):
        self._sock = socket.create_connection((host, port), timeout=3.0)
        self._sock.settimeout(0.05)

    def poll(self) -> bytes:
        try:
            return self._sock.recv(4096)
        except socket.timeout:
            return b""
        except OSError:
            return b""

    def send(self, data: bytes):
        self._sock.sendall(data)

    def close(self):
        try:
            self._sock.close()
        except Exception:
            pass


def wait_ack(link, timeout=3.0, expect_rsv=None):
    """阻塞读取一个 ACK 应答帧。
    expect_rsv 给定则只认 RSV 与之相等的应答（DATA 帧应答 = 该帧偏移，START/END = 0），
    其余偏移的旧 ACK 忽略继续等。超时返回 None。"""
    scanner = P.FrameScanner()
    deadline = time.time() + timeout
    while time.time() < deadline:
        chunk = link.poll()
        if chunk:
            scanner.push(chunk)
            frm = scanner.next_frame()
            while frm is not None:
                cmd, data, rsv = P.parse_frame(frm)
                if cmd == P.CMD_RSP_ACK and len(data) >= 1:
                    code = data[0]
                    if expect_rsv is None or rsv == expect_rsv:
                        return P.Ack(ok=(code == P.RSP_OK), code=code, cmd=cmd, raw=frm, rsv=rsv)
                frm = scanner.next_frame()
        else:
            time.sleep(0.005)
    return None  # 超时


# 逐帧确认与重发参数
# ★ START_TIMEOUT 必须是 8s：F407 的最小擦除单位是扇区（最小 128KB），收到 OTA_START
#   后 boot 要先把整个下载区擦掉才会回 ACK。即使只传几十 KB 的固件，也要擦掉一整块
#   128KB 扇区 —— 典型 1s、最坏 4s，且擦除期间 Flash 总线 stall、CPU 冻结。
#   F103 时代的 3.0s 会把这段正常擦除误判成"无应答"而重发，重发又触发一次全擦，
#   来回几次都成功不了。
START_TIMEOUT = 8.0
START_RETRY   = 3
DATA_TIMEOUT  = 3.0   # 单帧等 ACK。UART@115200 下 1036B 帧约 90ms；ETH 下是瞬时的
DATA_RETRY    = 10
# 超时后的静默退避，下标=第几次失败后的静默秒数（只走错误路径）。
# ★ 原值 [4,5,8,8...] 是给 HC-05 弱链路留 RF 恢复时间的；现在走有线 UART / ETH，
#   偶发一次超时基本都是瞬时拥塞，退避 4~8s 纯属浪费（ETH 上 TCP 自己还会重传）。
RETRY_QUIET  = [0.5, 1.0, 2.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0]
INTER_FRAME_GAP = 0.02  # 帧间最小停顿：防极端背靠背，几乎不耗时
END_TIMEOUT   = 3.0
END_RETRY     = 3


def run_ota(link, fw: bytes, inject_err: bool):
    """执行一次完整升级流程（逐帧确认 + 超时重发），返回 (ok, 总帧数, 说明)。"""
    chunks = P.split_firmware(fw)
    total_len = len(fw)
    print("[1/4] OTA_START  total=%d B  帧数=%d(每帧 %d B)" % (total_len, len(chunks), P.OTA_DATA_CHUNK))

    # START：等 ACK；应答丢失/坏帧则整帧重发
    start_ok = False
    for attempt in range(1, START_RETRY + 1):
        link.send(P.build_ota_start(total_len, target=0))
        ack = wait_ack(link, timeout=START_TIMEOUT, expect_rsv=0)
        if ack is not None:
            if ack.ok:
                start_ok = True
                break
            return False, 0, "OTA_START 被拒(RSP_ERR)"
        print("    START 无应答, 重试 %d/%d（boot 可能正在擦除下载区）" % (attempt, START_RETRY))
    if not start_ok:
        return False, 0, "OTA_START 无应答(重发 %d 次超时)" % START_RETRY

    print("[2/4] 开始发送 %d 帧数据(逐帧确认)..." % len(chunks))
    sent = 0
    for idx, (off, chunk) in enumerate(chunks):
        frm = P.build_ota_data(off, chunk)
        if inject_err and idx == 0:
            # 翻转帧内第 6 字节（数据区开头），破坏 CRC -> 该帧被丢弃、触发重发
            mid = bytearray(frm)
            mid[6] ^= 0xFF
            frm = bytes(mid)
            print("    [坏帧注入] 第1个数据帧 CRC 已破坏(应触发重发)")

        accepted = False
        for attempt in range(1, DATA_RETRY + 1):
            link.send(frm)
            ack = wait_ack(link, timeout=DATA_TIMEOUT, expect_rsv=off)
            if ack is not None and ack.ok:
                accepted = True
                break
            # 无应答(超时)或 NACK：打印原因后【静默退避】再重发本帧
            if ack is not None:
                print("    [重试] 帧%d 第%d次: NACK(code=%d, rsv=%d) —— 单片机拒绝/错位" % (idx + 1, attempt, ack.code, ack.rsv))
            else:
                print("    [重试] 帧%d 第%d次: 超时无应答(%.1fs) —— 静默退避中" % (idx + 1, attempt, DATA_TIMEOUT))
            quiet = RETRY_QUIET[attempt - 1] if attempt - 1 < len(RETRY_QUIET) else RETRY_QUIET[-1]
            time.sleep(quiet)
        if not accepted:
            return False, len(chunks), "第%d帧(off=%d) 重发%d次仍无确认" % (idx + 1, off, DATA_RETRY)
        sent += len(chunk)
        time.sleep(INTER_FRAME_GAP)
    print("[3/4] 已发送并确认 %d B" % sent)

    # END：等 ACK 验收（boot 应答后即搬运+跳转）
    for attempt in range(1, END_RETRY + 1):
        link.send(P.build_ota_end(total_len))
        ack = wait_ack(link, timeout=END_TIMEOUT, expect_rsv=0)
        if ack is not None:
            if ack.ok:
                return True, len(chunks), "OK"
            return False, len(chunks), "OTA_END 验收失败(RSP_ERR, 有丢帧/坏帧)"
        print("    END 无应答, 重试 %d/%d" % (attempt, END_RETRY))
    return False, len(chunks), "OTA_END 应答丢失(若已跳转到 app 属正常，否则请整包重发)"


def main():
    ap = argparse.ArgumentParser(description="OTA 帧协议联调 CLI")
    ap.add_argument("arg1", nargs="?",
                    help="串口(如 COM5)；配合 --tcp 使用时这一栏改填 bin 路径")
    ap.add_argument("arg2", nargs="?", help="固件 bin 文件（串口模式下需要）")
    ap.add_argument("--tcp", metavar="HOST:PORT", default=None,
                    help="改走 TCP 通道(ETH/wifi)，如 192.168.1.2:5001；省略则走串口")
    ap.add_argument("--baud", type=int, default=115200,
                    help="串口波特率(默认 115200)。有线链路不必降速；"
                         "仅当串口那一头挂的是蓝牙透传模组时才需要降到 9600~19200")
    ap.add_argument("--selftest", action="store_true", help="CRC 一致性自检（不连设备）")
    ap.add_argument("--inject-err", action="store_true", help="注入坏帧")
    ap.add_argument("--oversize", action="store_true", help="伪造越界固件")
    ap.add_argument("--probe", action="store_true", help="只发 OTA_START 探测")
    args = ap.parse_args()

    if args.selftest:
        # 用标准校验串验证 CRC 实现，并输出 C 表首项供人工比对
        print("CRC16-MODBUS('123456789') = 0x%04X (期望 0x4B37)" % P.crc16_modbus(b"123456789"))
        print("CRC 表[0..5] =", ["0x%04X" % v for v in P._CRC_TABLE[:6]])
        return

    # 位置参数的含义随 --tcp 变化：串口模式是 <COM> <bin>，TCP 模式只有 <bin>
    if args.tcp:
        if args.arg2:
            ap.error("--tcp 模式下只需给 bin 路径（串口参数不适用）")
        bin_path = args.arg1
    else:
        if not args.arg1:
            ap.error("串口模式用法: ota_host.py <COM> <bin>；或改用 --tcp HOST:PORT <bin>")
        bin_path = args.arg2

    if not bin_path or not os.path.exists(bin_path):
        print("错误: 需要有效的 bin 文件路径")
        sys.exit(1)

    fw = open(bin_path, "rb").read()
    if len(fw) > P.APP_MAX_SIZE:
        print("错误: bin %d B 超过固件上限 %d B" % (len(fw), P.APP_MAX_SIZE))
        sys.exit(1)

    if args.oversize:
        fw = bytes(random.getrandbits(8) for _ in range(P.APP_MAX_SIZE + 64))  # 越界
        print("警告: 伪造 %d B 越界固件（上限 %d B）" % (len(fw), P.APP_MAX_SIZE))

    # 打开通道
    try:
        if args.tcp:
            host, _, port_s = args.tcp.partition(":")
            port_s = port_s or "5001"
            link = _TcpLink(host, int(port_s))
            print("TCP 已连接 %s:%s" % (host, port_s))
        else:
            link = _SerialLink(args.arg1, args.baud)
            print("串口 %s 已打开 @%d 8N1" % (args.arg1, args.baud))
    except Exception as e:
        print("错误: 打开通道失败: %s" % e)
        sys.exit(1)

    try:
        if args.probe:
            link.send(P.build_ota_start(min(len(fw), P.APP_MAX_SIZE), 0))
            ack = wait_ack(link, timeout=START_TIMEOUT)
            print("OTA_START 应答:", "RSP_OK" if ack and ack.ok else ("RSP_ERR" if ack else "无应答"))
            return
        ok, nframes, msg = run_ota(link, fw, inject_err=args.inject_err)
        print("[4/4] 结果: %s  (帧数=%d, 消息=%s)" % ("成功" if ok else "失败", nframes, msg))
        sys.exit(0 if ok else 2)
    finally:
        link.close()


if __name__ == "__main__":
    main()
