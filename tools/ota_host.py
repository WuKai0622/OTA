# -*- coding: utf-8 -*-
"""
OTA 无界面联调 CLI —— 对 Bootloader 帧协议做真机验证。

用法：
    py tools/ota_host.py <COM> <bin文件>                # 正常升级（OTA_START->DATA->END）
    py tools/ota_host.py <COM> <bin文件> --selftest      # 仅 CRC 一致性自检（不连串口）
    py tools/ota_host.py <COM> <bin文件> --inject-err     # 随机翻转一个数据帧字节（坏帧测试）
    py tools/ota_host.py <COM> <bin文件> --oversize       # 伪造 >15KB 固件（越界拒绝测试）
    py tools/ota_host.py <COM> <bin文件> --probe          # 只发 OTA_START 探测

依赖：pyserial（已装）。波特率固定 115200 8N1。
"""
import argparse
import os
import random
import struct
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ota_proto as P


def _serial(port, baud=115200, timeout=0.2):
    import serial
    ser = serial.Serial(port, baud, timeout=timeout)
    ser.reset_input_buffer()
    return ser


def wait_ack(ser, timeout=3.0, expect_rsv=None):
    """阻塞读取一个 ACK 应答帧。
    expect_rsv 给定则只认 RSV 与之相等的应答（DATA 帧应答 = 该帧偏移，START/END = 0），
    其余偏移的旧 ACK 忽略继续等。超时返回 None。"""
    scanner = P.FrameScanner()
    deadline = time.time() + timeout
    while time.time() < deadline:
        avail = ser.in_waiting
        if avail > 0:
            scanner.push(ser.read(avail))
            frm = scanner.next_frame()
            while frm is not None:
                cmd, data, rsv = P.parse_frame(frm)
                if cmd == P.CMD_RSP_ACK and len(data) >= 1:
                    code = data[0]
                    if expect_rsv is None or rsv == expect_rsv:
                        return P.Ack(ok=(code == P.RSP_OK), code=code, cmd=cmd, raw=frm, rsv=rsv)
                frm = scanner.next_frame()
        else:
            time.sleep(0.02)
    return None  # 超时


# 逐帧确认与重发参数（HC-05 已降到 9600，黑障基本消失；重发只是兜底）
START_TIMEOUT = 3.0   # START 应答：含 A 区整擦时间(~几百 ms)
START_RETRY   = 3
DATA_TIMEOUT  = 3.0   # 单帧等 ACK(9600 下 512B 帧上串口约 0.55s，3s 足够宽裕)
DATA_RETRY    = 10
# 偶发 RF 卡顿：一次超时≈链路正在卡，先静默较久再探，别 0.5s 连撞浪费。
# 下标=第几次失败后的静默秒数；只走错误路径，几乎不影响干净路径速度。
RETRY_QUIET  = [4.0, 5.0, 8.0, 8.0, 8.0, 8.0, 8.0, 8.0, 8.0]
INTER_FRAME_GAP = 0.02  # 帧间最小停顿：9600 干净链路下几乎不耗时，仅防极端背靠背
END_TIMEOUT   = 3.0
END_RETRY     = 3


def run_ota(ser, fw: bytes, inject_err: bool, delay_per_frame=0.0):
    """执行一次完整升级流程（逐帧确认 + 超时重发），返回 (ok, 总帧数, 说明)。"""
    chunks = P.split_firmware(fw)
    total_len = len(fw)
    print("[1/4] OTA_START  total=%d B  帧数=%d(每帧 %d B)" % (total_len, len(chunks), P.OTA_DATA_CHUNK))

    # START：等 ACK；应答丢失/坏帧则整帧重发
    start_ok = False
    for attempt in range(1, START_RETRY + 1):
        ser.write(P.build_ota_start(total_len, target=0))
        ack = wait_ack(ser, timeout=START_TIMEOUT, expect_rsv=0)
        if ack is not None:
            if ack.ok:
                start_ok = True
                break
            return False, 0, "OTA_START 被拒(RSP_ERR)"
        print("    START 无应答, 重试 %d/%d" % (attempt, START_RETRY))
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
            ser.write(frm)
            ack = wait_ack(ser, timeout=DATA_TIMEOUT, expect_rsv=off)
            if ack is not None and ack.ok:
                accepted = True
                break
            # 无应答(超时)或 NACK：打印原因后【静默退避】再重发本帧
            if ack is not None:
                print("    [重试] 帧%d 第%d次: NACK(code=%d, rsv=%d) —— 单片机拒绝/错位" % (idx + 1, attempt, ack.code, ack.rsv))
            else:
                print("    [重试] 帧%d 第%d次: 超时无应答(%.1fs) —— 链路卡顿，静默退避中" % (idx + 1, attempt, DATA_TIMEOUT))
            quiet = RETRY_QUIET[attempt - 1] if attempt - 1 < len(RETRY_QUIET) else RETRY_QUIET[-1]
            time.sleep(quiet)
        if not accepted:
            return False, len(chunks), "第%d帧(off=%d) 重发%d次仍无确认(链路黑障超60s)" % (idx + 1, off, DATA_RETRY)
        sent += len(chunk)
        time.sleep(INTER_FRAME_GAP)
        if delay_per_frame > 0:
            time.sleep(delay_per_frame)
    print("[3/4] 已发送并确认 %d B" % sent)

    # END：等 ACK 验收（boot 应答后即搬运+跳转）
    for attempt in range(1, END_RETRY + 1):
        ser.write(P.build_ota_end(total_len))
        ack = wait_ack(ser, timeout=END_TIMEOUT, expect_rsv=0)
        if ack is not None:
            if ack.ok:
                return True, len(chunks), "OK"
            return False, len(chunks), "OTA_END 验收失败(RSP_ERR, 有丢帧/坏帧)"
        print("    END 无应答, 重试 %d/%d" % (attempt, END_RETRY))
    return False, len(chunks), "OTA_END 应答丢失(若已跳转到 app 属正常，否则请整包重发)"


def main():
    ap = argparse.ArgumentParser(description="OTA 帧协议联调 CLI")
    ap.add_argument("port", help="串口，如 COM5")
    ap.add_argument("bin", nargs="?", help="固件 bin 文件")
    ap.add_argument("--selftest", action="store_true", help="CRC 一致性自检")
    ap.add_argument("--inject-err", action="store_true", help="注入坏帧")
    ap.add_argument("--oversize", action="store_true", help="伪造越界固件")
    ap.add_argument("--probe", action="store_true", help="只发 OTA_START 探测")
    ap.add_argument("--baud", type=int, default=115200,
                    help="串口波特率(默认 115200；HC-05 弱链路建议 9600~19200)")
    args = ap.parse_args()

    if args.selftest:
        # 用标准校验串验证 CRC 实现，并输出 C 表首项供人工比对
        print("CRC16-MODBUS('123456789') = 0x%04X (期望 0x4B37)" % P.crc16_modbus(b"123456789"))
        print("CRC 表[0..5] =", ["0x%04X" % v for v in P._CRC_TABLE[:6]])
        return

    if not args.bin or not os.path.exists(args.bin):
        print("错误: 需要有效的 bin 文件路径")
        sys.exit(1)

    fw = open(args.bin, "rb").read()
    if len(fw) > P.APP_MAX_SIZE:
        print("错误: bin %d B 超过固件上限 %d B" % (len(fw), P.APP_MAX_SIZE))
        sys.exit(1)

    if args.oversize:
        fw = bytes(random.getrandbits(8) for _ in range(P.APP_MAX_SIZE + 64))  # 越界
        print("警告: 伪造 %d B 越界固件" % len(fw))

    ser = _serial(args.port, args.baud)
    print("串口 %s 已打开 @%d 8N1" % (args.port, args.baud))

    if args.probe:
        ser.write(P.build_ota_start(len(fw) if len(fw) <= P.APP_MAX_SIZE else P.APP_MAX_SIZE, 0))
        ack = wait_ack(ser)
        print("OTA_START 应答:", "RSP_OK" if ack and ack.ok else ("RSP_ERR" if ack else "无应答"))
        ser.close()
        return

    ok, nframes, msg = run_ota(ser, fw, inject_err=args.inject_err)
    print("[4/4] 结果: %s  (帧数=%d, 消息=%s)" % ("成功" if ok else "失败", nframes, msg))
    ser.close()
    sys.exit(0 if ok else 2)


if __name__ == "__main__":
    main()
