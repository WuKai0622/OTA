# -*- coding: utf-8 -*-
"""
OTA 无界面联调 CLI —— 对 Bootloader 帧协议做真机验证。

用法：
    py tools/ota_host.py <COM> <bin文件>                     # UART 串口
    py tools/ota_host.py --tcp 192.168.1.2:5001 <bin文件>   # ETH 网口（连 boot 的 TCP 服务器）
    py tools/ota_host.py --tcp 192.168.4.1:5001 <bin文件>   # wifi 透传（ESP32 默认 AP 模式）

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
# ★★【2026-10-10 定值：8.0 → 3.0 → 4.5；2026-11-03 第一次等待改为 2.0】判据如下
#   （都是 tools/ota_timing_sim.py 扫出来的，不是推的 —— 改完请重跑那个脚本核对；
#   它扫的是第 2/3 次的 START_TIMEOUT，第一次固定用 START_FIRST_TIMEOUT）：
#
#   ① 重发节奏。它真正约束的是一个延迟 D = **"设备吃掉这一帧 → boot 窗口重新打开"**
#      （APP 轮询 → 复位 → boot 初始化）。被吃掉的每一帧都会把窗口推到"吃掉时刻 + D"，
#      而下一次重发在 T 之后，所以能不能连上的判据就是 **T ≥ D**
#      （上界 T ≤ D + 5s 由 BOOT_WAIT_MS 给出）。
#      · 原值 8.0 覆盖的 D 是 [3, 8]s —— 恰好把 D < 3s 那一半漏在洞外，而"APP 一看
#        到东西就复位"正是 D ≈ 0.5s 这种最常见的情况。于是症状是"复位后第一次连不上，
#        之后就再也没有一次能连上"。4.5 覆盖 D ∈ [0, 4.5]s。
#      · ★ 反直觉的一点：**T 不是越大越好**。T 超过约 5s（窗口宽度）之后，覆盖区间会
#        从 [0, T] 变成 {0} ∪ [T-5, T]，即"复位越快越连不上"。所以 T 要压在窗口宽度以内。
#      · START_RETRY 基本无用：每次被吃掉都会重新把窗口推到 +D，多发的几次重发只是
#        把同一个实验重做一遍（实测 3 次与 5 次的覆盖区间完全相同）。
#
#   ② 等 START 的 ACK 的耐心。那个 ACK 要等 boot 把整个下载区擦完才回来：F407 单 bank，
#      最小擦除单位是 128KB 扇区（与固件大小无关），实测 1.03~1.20s、ST 规格最坏 4s，
#      且擦除期间 Flash 总线 stall、CPU 冻结。约束是 **T ≥ 擦除耗时**，否则重发会砸在
#      正在擦除的 boot 上：它擦完才处理那一帧 → **再擦一遍**（1~4s 白花），且已经发出去
#      的 DATA 帧会被"擦除中"顶掉、走一次 DATA 重发。实测擦除 1.2s 时 3.0s 就有 2.5 倍
#      余量；但若真出现 ST 规格的 4s 擦除，3.0s / 4.0s 都会触发上面这套（脚本里
#      --erase 4.0 可复现），4.5 才干净。
#
#   ③ ★ 2026-11-03：**第一次等待单独取 2.0s**（START_FIRST_TIMEOUT），其后仍用 4.5s。
#      原因：走 WiFi 通道时首帧必被正在运行的 APP 吃掉（APP 在 SPI 上看到字节就复位），
#      D 通常只有 0.5s 左右。4.5s 虽然能覆盖，但重发点已经贴到 boot 那条 5s 窗口
#      [D, D+5] 的后段；压到 2.0s 后重发落在窗口前段，余量最大。
#      代价要清楚：这 2.0s 同时是"等擦除 ACK"的耐心，只覆盖实测 1.03~1.20s
#      （本板 1.11s，约 1.8 倍余量），不再覆盖 ST 规格的 4s 最坏值。真遇到慢擦除，
#      重发会砸在擦除中的 boot 上再擦一遍 —— 第 2/3 次的 4.5s 仍按最坏值留了余量。
#      若 APP 刚启动（它自己有 2.5s 的首轮延时），2.0s 那次也会被吃掉；第二次在
#      2.0 + 4.5 = 6.5s 发出，仍落在 D≈3s 打开的窗口 [3, 8] 内。
#
#   ⇒ 合计：等待值 T 的可用区间是 [max(D, 擦除耗时), D + 5s]。现取 **第一次 2.0s、
#     后续 4.5s**：第一次按 D≈0.5s 的场景优化，后续 4.5 同时覆盖 D ≤ 4.5s 与
#     擦除 ≤ 4s。★ 2 和 3 的取舍都验证过：T=3 覆盖 D ≤ 3s；T=5 已贴到窗口边缘。
#   ★ 长期方案仍是让 boot 对"长度相同的重复 START"去重（不再二次擦除），那时 ② 消失。
START_TIMEOUT       = 4.5   # 第 2/3 次等 ACK（要同时覆盖 boot 初始化与最坏擦除）
START_FIRST_TIMEOUT = 2.0   # ★ 第 1 次只等 2s：WiFi 首帧被 APP 吃掉后尽快重发
START_RETRY   = 3
DATA_TIMEOUT  = 3.0   # 单帧等 ACK。UART@115200 下 1036B 帧约 90ms；ETH 下是瞬时的
DATA_RETRY    = 10
# 超时后的静默退避，下标=第几次失败后的静默秒数（只走错误路径）。
# ★ 原值 [4,5,8,8...] 是给弱链路留恢复时间的；现在走有线 UART / ETH，
#   偶发一次超时基本都是瞬时拥塞，退避 4~8s 纯属浪费（ETH 上 TCP 自己还会重传）。
RETRY_QUIET  = [0.5, 1.0, 2.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0]
# 帧间停顿：原为 0.02s（防极端背靠背）。实测它吃掉以太网通道约 29% 的传输时间 ——
#   1024B/帧时，0.02s 单独就把上限锁死在约 410 kbps，而 TCP 自身已有 Nagle 与拥塞
#   控制兜底，故改为 0。调用处用 >0 判断，跳过 sleep 系统调用本身。
#   ★ 若在弱链路上出现丢帧/超时，可调回 0.002 ~ 0.02 之间。
INTER_FRAME_GAP = 0.0
END_TIMEOUT   = 3.0
END_RETRY     = 3

# 最近一次 run_ota() 的分段耗时（秒），供 main() 打印。
# ★ 为什么必须拆开:boot 收到 OTA_START 后要先把整个下载区擦掉才回 ACK,而这是【固定成本】
#   ——F407 单 bank,最小擦除单位就是 128KB 扇区(典型 1s、最坏 4s),与固件大小完全无关。
#   固件越小,它占总耗时比例越高,于是"用总耗时算出来的 kbps"会随固件大小剧烈漂移
#   (同一块板子:传 26KB 时约 168 kbps,传 79KB 时约 497 kbps),那不是链路能力。
LAST_TIMING = {}   # {"erase": 秒, "data": 秒}


def run_ota(link, fw: bytes, inject_err: bool, token: bytes):
    """执行一次完整升级流程（逐帧确认 + 超时重发），返回 (ok, 总帧数, 说明)。
    token = 已校验过的 8 字节令牌（见 P.parse_token），随 OTA_START 一起发出。"""
    chunks = P.split_firmware(fw)
    total_len = len(fw)
    LAST_TIMING.clear()
    t0 = time.time()
    print("[1/4] OTA_START  total=%d B  帧数=%d(每帧 %d B)" % (total_len, len(chunks), P.OTA_DATA_CHUNK))

    # START：等 ACK；应答丢失/坏帧则整帧重发
    start_ok = False
    for attempt in range(1, START_RETRY + 1):
        link.send(P.build_ota_start(total_len, target=0, token=token))
        # ★ 第一次用较短的 2.0s：WiFi 通道首帧会被正在运行的 APP 吃掉，
        #   尽快重发才能落进 APP 复位后 boot 新开的窗口。后续用 4.5s，
        #   覆盖 boot 初始化 + ST 规格的最坏擦除（见文件顶部说明 ③）。
        wait_s = START_FIRST_TIMEOUT if attempt == 1 else START_TIMEOUT
        ack = wait_ack(link, timeout=wait_s, expect_rsv=0)
        if ack is not None:
            if ack.ok:
                start_ok = True
                break
            return False, 0, "OTA_START 被拒(RSP_ERR)"
        print("    START 无应答, 重试 %d/%d（boot 可能正在擦除下载区）" % (attempt, START_RETRY))
    if not start_ok:
        return False, 0, "OTA_START 无应答(重发 %d 次超时)" % START_RETRY
    # START 的 ACK 到手 = 下载区擦除完毕（另外含发送 START + 一个往返，毫秒级可忽略）
    LAST_TIMING["erase"] = time.time() - t0

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
        if INTER_FRAME_GAP > 0:
            time.sleep(INTER_FRAME_GAP)
    print("[3/4] 已发送并确认 %d B" % sent)
    LAST_TIMING["data"] = time.time() - t0 - LAST_TIMING["erase"]

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
                    help="串口波特率(默认 115200，与 boot 的 USART3 一致)；"
                         "改了 boot 侧才需要同步改这里")
    ap.add_argument("--selftest", action="store_true", help="CRC 一致性自检（不连设备）")
    ap.add_argument("--inject-err", action="store_true", help="注入坏帧")
    ap.add_argument("--oversize", action="store_true", help="伪造越界固件")
    ap.add_argument("--probe", action="store_true", help="只发 OTA_START 探测")
    # ★ 必填，而且**不用 argparse 的 required=True**：那样 --selftest 也会被拦住，
    #   而自检根本不连设备、不需要令牌。改成下面手工校验（见 selftest 分支之后）。
    ap.add_argument("--token", metavar="HEX16", default=None,
                    help="OTA_START 令牌，必填 16 位 hex（= 8 字节）。取值见 boot 侧 protocol.h "
                         "的 OTA_TOKEN_W0/W1（与 tools/ota_proto.py 的同一对常量一致），"
                         "本文档与本工具的提示里都不给出具体值。"
                         "空格分隔亦可：--token \"01 23 45 67 89 AB CD EF\"")
    args = ap.parse_args()

    if args.selftest:
        # 用标准校验串验证 CRC 实现，并输出 C 表首项供人工比对
        print("CRC16-MODBUS('123456789') = 0x%04X (期望 0x4B37)" % P.crc16_modbus(b"123456789"))
        print("CRC 表[0..5] =", ["0x%04X" % v for v in P._CRC_TABLE[:6]])
        return

    # 令牌必填：发第一个字节之前就校验掉，避免现场把"令牌错"误读成"链路/固件有问题"
    try:
        token = P.parse_token(args.token.strip() if args.token else "")
    except ValueError as e:
        ap.error("令牌无效: %s" % e)

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
        # 打出来是为了让"START 被拒"可诊断：令牌错和链路/固件问题都表现为 RSP_ERR，
        # 回显本次实际发出的令牌，至少能排除"是不是我填错了"。
        # ★ 用 hex 不用 decode：令牌是二进制（含 0x00），decode 出来是控制字符。
        print("令牌: %s" % P.token_hex(token))

        if args.probe:
            link.send(P.build_ota_start(min(len(fw), P.APP_MAX_SIZE), 0, token=token))
            ack = wait_ack(link, timeout=START_TIMEOUT)
            print("OTA_START 应答:", "RSP_OK" if ack and ack.ok else ("RSP_ERR" if ack else "无应答"))
            return
        # 耗时统计：计时区间 = 发出 OTA_START 起到收到 END 的 ACK 止，
        # 因此【包含 boot 擦除下载区】那 1~4s（擦除完成才回 START 的 ACK）。
        t0 = time.time()
        ok, nframes, msg = run_ota(link, fw, inject_err=args.inject_err, token=token)
        elapsed = time.time() - t0

        print("[4/4] 结果: %s  (帧数=%d, 消息=%s)" % ("成功" if ok else "失败", nframes, msg))
        # ★ 只用 ASCII 前缀，不用 ⏱/⚡：stdout 重定向到文件时编码是 GBK(cp936)，
        #   那两个符号不在 GBK 里，会直接 UnicodeEncodeError 把整条命令搞崩。
        er = LAST_TIMING.get("erase")
        da = LAST_TIMING.get("data")
        if er is None or da is None:
            print("      升级耗时: %.2f 秒" % elapsed)
        else:
            print("      升级耗时: %.2f 秒  = 擦除下载区 %.2f + 传输 %.2f" % (elapsed, er, da))
            if da > 0:
                print("      传输段吞吐: %.1f kbps (%.2f KB/s, 每帧 %.1f ms)"
                      % (len(fw) * 8.0 / da / 1000.0, len(fw) / da / 1024.0,
                         da * 1000.0 / max(nframes, 1)))
        # 含擦除的总吞吐：这才是"用户按下按钮到能跑 app"的实际感受，但会随固件大小漂移
        if ok and elapsed > 0:
            print("      含擦除总吞吐: %.1f kbps (%.2f KB/s)"
                  % (len(fw) * 8.0 / elapsed / 1000.0, len(fw) / elapsed / 1024.0))
        sys.exit(0 if ok else 2)
    finally:
        link.close()


if __name__ == "__main__":
    main()
