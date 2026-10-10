# -*- coding: utf-8 -*-
"""
OTA_START 重发时序离线验证 —— 不接板子，用虚拟时钟跑【真实】的 ota_host.run_ota()。

为什么需要它：
  START_TIMEOUT 同时管两件事（见 ota_host.py 那段注释）——【START 的重发节奏】要和
  boot 的 BOOT_WAIT_MS(5s) 上电窗口对齐，【等 START 的 ACK 的耐心】要盖过擦除下载区的
  1~4s。这两件事什么时候打架，只在"设备复位"这一个场景下才暴露，而那个场景要等 APP
  侧的 SPI 轮询做完才能上板子复现。本脚本把那个场景搬到 PC 上，扫一遍"复位延迟"，
  直接给出：**哪些复位延迟连得上、哪些连不上、会不会多擦一遍**。

★ 它跑的是真代码：run_ota / wait_ack / build_ota_start / FrameScanner 全部来自
  ota_host / ota_proto，只换掉两样 ——
    · ota_host.time  → 虚拟时钟（否则一次扫描要跑几十分钟真实时间）
    · 链路对象       → FakeLink，对端是下面的 DevModel
  所以这里量出来的时序就是上位机真实的时序。一旦 ota_host 改了重发逻辑，
  重跑本脚本立刻反映出来；它不会和真实实现漂移。

★ 打印一律 ASCII：Windows 下 stdout 走 GBK(cp936)，中文会变乱码、甚至
  UnicodeEncodeError 把脚本搞崩（ota_host.py 里为此也只用 ASCII 前缀）。
  注释可以中文 —— 文件是 UTF-8，Python 读源码不受 console 影响。
  ★ 唯一例外是 --verbose：它原样转发 ota_host.run_ota() 自己的中文输出，在 Git Bash
    （UTF-8 终端）里会花屏、在 cmd.exe 里正常。这是控制台编码问题，不是脚本问题。

用法：
    py tools/ota_timing_sim.py                          # 量 ota_host.py 里当前配的那个值
    py tools/ota_timing_sim.py --timeouts 3,5,8         # 对比候选值
    py tools/ota_timing_sim.py --retries 3,5            # 看 START_RETRY 有没有用
    py tools/ota_timing_sim.py --erase 4.0              # 最坏擦除时间，看会不会重擦
    py tools/ota_timing_sim.py --mode buffer            # APP 不吃 START 的对照
    py tools/ota_timing_sim.py --verbose 2.0            # 打印"复位延迟 2s"那次的真实升级日志
"""
import argparse
import contextlib
import io
import os
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ota_proto as P
import ota_host

# 现实量级（可被命令行覆盖）：
#   ERASE_S      实测擦除下载区 1.03~1.20s（ST 规格最坏 4s）
#   WINDOW_S     = boot 侧 Bootloader.c 的 BOOT_WAIT_MS / 1000
#   OPEN_DELAY   从 PC 发出第一帧，到 boot 窗口开启的总延迟（APP 轮询→复位→启动初始化）
DEFAULT_ERASE_S = 1.2
DEFAULT_WINDOW_S = 5.0


# ==================== 虚拟时钟 ====================
class VClock:
    """把 time.time()/time.sleep() 换成"只推进、不真的等"。

    ★ 这是本脚本唯一伸手进 ota_host 内部的地方：run_ota() 里的 time.time() 和
      wait_ack() 里的 time.sleep(0.005) 读的都是 ota_host 模块全局的 `time`，
      所以替换这一个名字，两个函数就全走虚拟时钟了 —— 不用改一行真代码。
    ★ 反过来看它也是本脚本最大的假设：**只有当 ota_host 里的等待全部经由
      time.sleep/time.time 表达时它才成立**。哪天上位机改成 select() 超时或
      事件回调，这里会立刻失真（表现为"扫描结果全是失败"），别当成真结论。"""

    def __init__(self):
        self.t = 0.0

    def time(self):
        return self.t

    def sleep(self, sec):
        self.t += sec


# ==================== 假 boot ====================
def _ack(code, rsv=0):
    return P.build_frame(P.CMD_RSP_ACK, bytes([code]), rsv=rsv)


class DevModel:
    """按真实时序反应的假 boot。

    时间轴上只有三段，都对应真机上的一件事：
      · t < open_delay          → 设备不在窗口里。mode="app" 时**吃掉**帧（APP 在轮询
        SPI，每笔事务都从 C3 的环里取走最多 256B，见 esp32/wifi/main/spi_bridge.c:160）；
        mode="buffer" 时留着（C3 那 4096B 的环会缓冲它）。
      · open_delay ≤ t ≤ close  → boot 窗口。窗口里收到 START **先擦除再回 ACK**，
        擦除期间 Flash 总线 stall、CPU 冻结 → 不收不处理，帧躺在环里。
      · t > close 且无会话       → 跳回 APP，回到第一段。若 APP 每看到东西就复位
        （re_reset），窗口会以同样的延迟重新打开 —— 这就是"复位后永远连不上"那个
        死循环的成因，也是本脚本要量出来的东西。

    ★ 不建模（有意为之，避免给出虚假的精确感）：
        · 擦除期间到达的帧，真机是在 erase 返回后才从环里逐帧取出，本模型照此"擦完再处理"
          —— 但不再细分协议内部的处理顺序；
        · CRC 坏帧、令牌校验、DATA 偏移不匹配的 NACK 细节只做最小近似（只比 off==rx_total）；
        · C3 环溢出（4096B）不计：单帧 1036B，只在疯狂连发时才会撞上。
    """

    def __init__(self, open_delay, erase_s, window_s, mode="app", re_reset=True):
        self.open_delay = open_delay      # 窗口开启时刻
        self.reset_latency = open_delay   # "再复位一次"所需的时间（与首次同一量级）
        self.window_s = window_s
        self.window_close = open_delay + window_s
        self.erase_s = erase_s
        self.mode = mode                  # "app": 帧被 APP 取走; "buffer": C3 的环留着
        self.re_reset = re_reset

        self.scanner = P.FrameScanner()
        self.out = []                     # [(due_time, bytes)] 待回给上位机
        self.held = []                    # [(t, frm)] 忙/没人在听时留在环里的帧
        self.busy = False                 # 正在擦除
        self.erase_until = 0.0
        self.session = False              # 会话已建立（boot 进了常驻下载循环）
        self.rx_total = 0
        self.erases = 0
        self.start_hits = []              # [(t, 上位机第几笔帧)] 被接受的 START
        self.data_in = 0                  # 到达 _serve 的 DATA 帧数（1 帧固件 → 多余的就是重发）

    # ---- 时序 ----
    def _phase(self, t):
        """t 时刻谁在听：'boot'（被取走）/ 'app'（被吃掉或留着）。"""
        if self.session or self.busy:
            return "boot"
        return "boot" if self.open_delay <= t <= self.window_close else "app"

    # ---- 收到上位机的字节 ----
    def on_send(self, t, seq, data):
        self.scanner.push(data)
        while True:
            frm = self.scanner.next_frame()
            if frm is None:
                return
            self._inbound(t, seq, frm)

    def _inbound(self, t, seq, frm):
        cmd, _data, _rsv = P.parse_frame(frm)
        if self._phase(t) == "app":
            if self.mode == "app":
                # APP 把这帧取走了。它只认"有东西来了"就复位（不需要解析帧），
                # 于是窗口在 reset_latency 之后重新打开 —— 死循环的每一圈都从这里开始。
                if cmd == P.CMD_OTA_START and self.re_reset:
                    self.open_delay = t + self.reset_latency
                    self.window_close = self.open_delay + self.window_s
                return
            self.held.append((t, frm))     # C3 的环留着，等窗口开了再取
            return
        if self.busy:
            self.held.append((t, frm))     # 擦除中，谁都处理不了
            return
        self._serve(t, seq, frm)

    # ---- 设备当前真的在处理一帧 ----
    def _serve(self, t, seq, frm):
        cmd, data, rsv = P.parse_frame(frm)
        if cmd == P.CMD_OTA_START:
            self.erases += 1
            self.start_hits.append((t, seq))
            self.busy = True
            self.erase_until = t + self.erase_s    # 擦完才回 ACK
        elif cmd == P.CMD_OTA_DATA:
            self.data_in += 1
            if rsv == self.rx_total:
                self.rx_total += len(data)
                self.out.append((t, _ack(P.RSP_OK, rsv=rsv)))
            else:
                self.out.append((t, _ack(P.RSP_ERR, rsv=rsv)))
        elif cmd == P.CMD_OTA_END:
            got = struct.unpack("<I", data[:4])[0] if len(data) >= 4 else -1
            self.out.append((t, _ack(P.RSP_OK if got == self.rx_total else P.RSP_ERR, rsv=0)))

    # ---- 时钟推进 ----
    def poll(self, now):
        self._advance(now)
        out, rest = b"", []
        for due, fr in self.out:
            if due <= now:
                out += fr
            else:
                rest.append((due, fr))
        self.out = rest
        return out

    def _advance(self, now):
        if self.busy and now >= self.erase_until:
            self.busy = False
            self.session = True
            self.rx_total = 0             # START 分支会重置接收计数（与 protocol.c 一致）
            self.out.append((self.erase_until, _ack(P.RSP_OK, rsv=0)))
        self._drain(now)

    def _drain(self, now):
        """把"环里攒着"的帧按到达顺序喂给设备。

        设备此刻在轮询 SPI 的两个时机：**窗口一开**（内存里的帧一直等在这儿，见
        mode="buffer"）和**擦除结束后**（擦除期间到达的帧）。擦除中不处理 ——
        Flash 总线 stall、CPU 冻结。某个帧若自己又开了一次擦除，剩下的留到那之后。"""
        if self.busy:
            return
        if not (self.session or self.open_delay <= now <= self.window_close):
            return                        # 不在窗口里也没会话 → 没人轮询，继续躺着
        while self.held and not self.busy:
            _t, frm = self.held.pop(0)
            self._serve(now, 0, frm)


class FakeLink:
    """run_ota() 只认 poll()/send()/close() 三个方法，这里接到 DevModel 上。"""

    def __init__(self, dev, clock):
        self.dev = dev
        self.clock = clock
        self.n = 0

    def send(self, data):
        self.n += 1
        self.dev.on_send(self.clock.t, self.n, data)

    def poll(self):
        return self.dev.poll(self.clock.t)

    def close(self):
        pass


# ==================== 跑一次 ====================
def run_one(timeout, retries, open_delay, erase_s, window_s, mode, re_reset):
    """跑一次完整升级，返回观测到的结果。"""
    clock = VClock()
    ota_host.time = clock                 # ★ 见 VClock 的注释
    ota_host.START_TIMEOUT = timeout
    ota_host.START_RETRY = retries

    dev = DevModel(open_delay, erase_s, window_s, mode, re_reset)
    link = FakeLink(dev, clock)

    # 1 帧固件：把 DATA 段压到最短，量的就是 START 那段时序。
    # 多余出来的 DATA 帧数（data_in > 1）= 被"擦除中"顶掉后重发的次数。
    fw = b"\x5A" * P.OTA_DATA_CHUNK
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        ok, _n, msg = ota_host.run_ota(link, fw, False, P.OTA_TOKEN)

    return {
        "ok": ok, "msg": msg, "dur": clock.t, "log": buf.getvalue(),
        "start_seq": dev.start_hits[0][1] if dev.start_hits else None,
        "erases": dev.erases, "data_in": dev.data_in,
        "start_sends": link.n,
    }


def _fmt_ranges(pairs, want):
    """把 (值, 成功?) 序列压成区间文本。只做展示。"""
    out, start, prev = [], None, None
    for v, ok in pairs:
        if ok == want:
            if start is None:
                start = v
            prev = v
        elif start is not None:
            out.append((start, prev))
            start = None
    if start is not None:
        out.append((start, prev))
    return out


def _fmt_iv(ivs):
    if not ivs:
        return "none"
    return "  ".join(("[%.2f, %.2f]" % (a, b)) if a != b else ("{%.2f}" % a) for a, b in ivs)


def sweep(timeout, retries, args):
    """扫一遍"复位延迟"，返回 (成功区间, 失败区间, 统计)。"""
    pairs, erases_max, dbl, data_retry_cnt = [], 0, [], 0
    d = 0.0
    while d <= args.rmax + 1e-9:
        r = run_one(timeout, retries, d, args.erase, args.window, args.mode,
                    re_reset=(not args.no_re_reset) and args.mode == "app")
        pairs.append((d, r["ok"]))
        erases_max = max(erases_max, r["erases"])
        if r["erases"] >= 2:
            dbl.append(d)
        if r["data_in"] > 1:
            data_retry_cnt += 1
        d = round(d + args.step, 6)
    return pairs, erases_max, dbl, data_retry_cnt


def main():
    ap = argparse.ArgumentParser(description="OTA_START 重发时序离线验证（不接板子）")
    ap.add_argument("--timeouts", default=None,
                    help="要对比的 START_TIMEOUT（秒，逗号分隔）。"
                         "★ 默认不是写死的候选值，而是**读 ota_host.py 里当前配的那个** "
                         "—— 不这样，改了常量再跑脚本就会量到别的值，白验一遍")
    ap.add_argument("--retries", default="3",
                    help="要对比的 START_RETRY（逗号分隔），默认 3")
    ap.add_argument("--erase", type=float, default=DEFAULT_ERASE_S,
                    help="擦除下载区耗时（秒）。实测 1.03~1.20；ST 规格最坏 4.0")
    ap.add_argument("--window", type=float, default=DEFAULT_WINDOW_S,
                    help="boot 上电窗口宽度（秒）= BOOT_WAIT_MS/1000")
    ap.add_argument("--rmax", type=float, default=20.0, help="复位延迟扫描上限（秒）")
    ap.add_argument("--step", type=float, default=0.1, help="扫描步长（秒）")
    ap.add_argument("--mode", choices=("app", "buffer"), default="app",
                    help="app=APP 吃掉 START 再复位（要做的设计）; buffer=C3 的环留住它")
    ap.add_argument("--no-re-reset", action="store_true",
                    help="APP 只复位一次，之后不再对 START 做反应")
    ap.add_argument("--verbose", type=float, default=None, metavar="R",
                    help="打印指定复位延迟那一次的真实升级日志")
    args = ap.parse_args()

    # ★ 不带 --timeouts 时量的就是 ota_host.py 当前配的值（而不是某个写死的候选），
    #   这样"改完常量重跑一遍"才有意义。
    tos = [ota_host.START_TIMEOUT] if args.timeouts is None \
        else [float(x) for x in args.timeouts.split(",")]
    rets = [int(x) for x in args.retries.split(",")]

    print("OTA START retry timing simulation (no board, virtual clock)")
    print("  erase=%.2fs  window=%.2fs  open-delay sweep=0.00..%.2f step=%.2f  mode=%s"
          % (args.erase, args.window, args.rmax, args.step, args.mode))
    print("  START_TIMEOUT under test: %s"
          % ", ".join("%.2fs%s" % (t, "  (from ota_host.py)" if args.timeouts is None else "")
                      for t in tos))
    if args.mode == "app" and not args.no_re_reset:
        print("  model: the APP eats every START it sees and resets again "
              "(the loop we are trying to break)")
    elif args.mode == "app":
        print("  model: the APP eats the first START and resets once, then stops reacting")
    else:
        print("  model: the C3 ring keeps the START; nothing consumes it "
              "(the reset must come from elsewhere)")
    print("")

    # ---- 自检：正常升级（设备本来就在窗口里）必须第一发就成功、只擦一次 ----
    print("=== sanity check: device already in window (normal upgrade) ===")
    bad = []
    for T in tos:
        r = run_one(T, 3, 0.0, args.erase, args.window, "app", re_reset=False)
        flag = "OK" if (r["ok"] and r["start_seq"] == 1 and r["erases"] == 1) else "FAIL"
        if flag == "FAIL":
            bad.append(T)
        print("  START_TIMEOUT=%.1fs -> %s  (first-send=%s, erases=%d, data_frames=%d)"
              % (T, flag, r["start_seq"], r["erases"], r["data_in"]))
    print("")

    # ---- 主扫描 ----
    for T in tos:
        for n in rets:
            pairs, emax, dbl, dretry = sweep(T, n, args)
            cov = _fmt_ranges(pairs, True)
            mis = _fmt_ranges(pairs, False)
            nok = sum(1 for _v, ok in pairs if ok)
            print("=== START_TIMEOUT=%.1fs  START_RETRY=%d ===" % (T, n))
            print("  connected for open-delay D in : %s   (%d/%d)" % (cov, nok, len(pairs)))
            print("  FAILED    for open-delay D in : %s" % mis)
            print("  max erases per run : %d%s" % (emax,
                  "   <-- !! double erase (retry landed while erasing)" if emax >= 2 else ""))
            if dbl:
                print("  double-erase at D   : %s" % _fmt_iv([(a, a) for a in dbl]))
            if dretry:
                print("  runs with DATA retry: %d  (erase ate the DATA frame; recovered)" % dretry)
            print("")

    # ---- 可选：打印某一次的真实日志 ----
    if args.verbose is not None:
        T = tos[0]
        n = rets[0]
        r = run_one(T, n, args.verbose, args.erase, args.window, args.mode,
                    re_reset=(not args.no_re_reset) and args.mode == "app")
        print("=== verbose run: D=%.2f  START_TIMEOUT=%.1f  START_RETRY=%d ===" % (args.verbose, T, n))
        print("  ok=%s  start_seq=%s  erases=%d  data_frames=%d  virtual_time=%.2fs"
              % (r["ok"], r["start_seq"], r["erases"], r["data_in"], r["dur"]))
        print("--- real run_ota() output ---")
        print(r["log"])

    print("Legend: 'open-delay D' = seconds from the PC's first frame to the moment the")
    print("        boot's BOOT_WAIT window opens (APP poll -> reset -> boot init).")
    print("        'connected for D in ...' = the reset delays that still complete an upgrade.")
    return 1 if bad else 0


if __name__ == "__main__":
    _orig_time = ota_host.time
    try:
        sys.exit(main())
    finally:
        ota_host.time = _orig_time
