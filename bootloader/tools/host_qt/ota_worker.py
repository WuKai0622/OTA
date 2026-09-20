# -*- coding: utf-8 -*-
"""
升级流程 Worker（QThread）：在后台线程执行 读bin->START->DATA...->END 并等待应答，
通过信号回报进度/日志/结果，避免卡死 UI。
"""
import os
import sys
import time

from PySide6.QtCore import QThread, Signal

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
import ota_proto as P

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
# 帧间停顿：原为 0.02s（防极端背靠背）。实测它吃掉以太网通道约 29% 的传输时间 ——
#   1024B/帧时，0.02s 单独就把上限锁死在约 410 kbps，而 TCP 自身已有 Nagle 与拥塞
#   控制兜底，故改为 0。调用处用 >0 判断，跳过 sleep 系统调用本身。
#   ★ 若在弱链路上出现丢帧/超时，可调回 0.002 ~ 0.02 之间。
INTER_FRAME_GAP = 0.0
END_TIMEOUT   = 3.0
END_RETRY     = 3


class OtaWorker(QThread):
    log = Signal(str)                 # 普通日志行
    err = Signal(str)                 # 错误/失败行
    progress = Signal(int, int, str)  # (已发帧, 总帧数, 状态文本)
    done = Signal(bool, str)          # (成功?, 摘要)

    def __init__(self, transport, bin_path: str, target: int = 0, parent=None):
        super().__init__(parent)
        self._transport = transport
        self._bin_path = bin_path
        self._target = target
        self._stop = False

    def stop(self):
        self._stop = True

    def _wait_ack(self, timeout: float, expect_rsv: "int | None" = None):
        """阻塞读一个 ACK 帧；expect_rsv 给定则只认 RSV 相等的应答（DATA 逐帧确认），
        其余偏移的旧 ACK 忽略继续等。支持 stop() 及时打断。超时返回 None。"""
        scanner = P.FrameScanner()
        deadline = time.time() + timeout
        while time.time() < deadline and not self._stop:
            data = self._transport.recv(256, 0.05)
            if data:
                scanner.push(data)
                frm = scanner.next_frame()
                while frm is not None:
                    cmd, payload, rsv = P.parse_frame(frm)
                    if cmd == P.CMD_RSP_ACK and len(payload) >= 1:
                        if expect_rsv is None or rsv == expect_rsv:
                            return P.Ack(ok=(payload[0] == P.RSP_OK),
                                         code=payload[0], cmd=cmd, raw=frm, rsv=rsv)
                    frm = scanner.next_frame()
        return None

    def run(self):
        try:
            self._transport.open()
        except Exception as e:
            self.err.emit("连接失败: %s" % e)
            self.done.emit(False, "连接失败")
            return
        self.log.emit("已连接，开始读取固件...")

        try:
            with open(self._bin_path, "rb") as f:
                fw = f.read()
        except Exception as e:
            self._transport.close()
            self.err.emit("读取固件失败: %s" % e)
            self.done.emit(False, "读取固件失败")
            return

        if len(fw) == 0:
            self._transport.close()
            self.err.emit("固件为空")
            self.done.emit(False, "固件为空")
            return
        if len(fw) > P.APP_MAX_SIZE:
            self._transport.close()
            self.err.emit("固件 %d B 超过上限 %d B" % (len(fw), P.APP_MAX_SIZE))
            self.done.emit(False, "固件超限")
            return

        chunks = P.split_firmware(fw)
        total = len(chunks)
        self.log.emit("固件 %d B，分包 %d 帧(每帧 %d B，逐帧确认)" % (len(fw), total, P.OTA_DATA_CHUNK))

        # 计时起点：发出 OTA_START 之前。区间覆盖到收到 END 的 ACK 为止，
        # 所以【包含 boot 擦除下载区】的 1~4s（擦除完成才会回 START 的 ACK）。
        t0 = time.time()

        # 1) START：等 ACK OK（应答丢失则整帧重发）
        self._transport.flush_rx()
        start_ok = False
        for attempt in range(1, START_RETRY + 1):
            if self._stop:
                self._transport.close(); self.done.emit(False, "用户中止")
                return
            self._transport.send(P.build_ota_start(len(fw), self._target))
            self.log.emit("已发送 OTA_START(总长=%d)，等待应答..." % len(fw))
            ack = self._wait_ack(timeout=START_TIMEOUT, expect_rsv=0)
            if ack is not None:
                if ack.ok:
                    start_ok = True
                    break
                self._transport.close(); self.err.emit("OTA_START 被拒绝(RSP_ERR)")
                self.done.emit(False, "START 拒绝")
                return
            self.err.emit("OTA_START 无应答，重试 %d/%d" % (attempt, START_RETRY))
        if not start_ok:
            self._transport.close(); self.err.emit("OTA_START 无应答(重发 %d 次超时)" % START_RETRY)
            self.done.emit(False, "START 超时")
            return
        # START 的 ACK 到手 = 下载区擦除完毕（另外含发送 START + 一个往返，毫秒级可忽略）
        erase_s = time.time() - t0
        self.log.emit("OTA_START ACK=OK")

        # 2) DATA：逐帧发，等该偏移 ACK OK；超时/NACK 重发本帧
        sent = 0
        for i, (off, chunk) in enumerate(chunks):
            if self._stop:
                self._transport.close(); self.done.emit(False, "用户中止")
                return
            accepted = False
            for attempt in range(1, DATA_RETRY + 1):
                if self._stop:
                    self._transport.close(); self.done.emit(False, "用户中止")
                    return
                self._transport.send(P.build_ota_data(off, chunk))
                ack = self._wait_ack(timeout=DATA_TIMEOUT, expect_rsv=off)
                if ack is not None and ack.ok:
                    accepted = True
                    break
                if ack is not None:
                    self.err.emit("帧%d 第%d次: NACK(code=%d) —— 单片机拒绝/错位" % (i + 1, attempt, ack.code))
                else:
                    self.err.emit("帧%d 第%d次: 超时无应答 —— 链路卡顿，静默退避中" % (i + 1, attempt))
                if not self._stop:
                    quiet = RETRY_QUIET[attempt - 1] if attempt - 1 < len(RETRY_QUIET) else RETRY_QUIET[-1]
                    time.sleep(quiet)
            if not accepted:
                self._transport.close()
                self.err.emit("第 %d 帧(off=%d) 重发 %d 次仍无确认" % (i + 1, off, DATA_RETRY))
                self.done.emit(False, "验收失败")
                return
            sent += len(chunk)
            if not self._stop and INTER_FRAME_GAP > 0:
                time.sleep(INTER_FRAME_GAP)
            self.progress.emit(i + 1, total, "已发 %d/%d 帧 (%d B)" % (i + 1, total, sent))

        self.log.emit("数据帧已全部确认 (%d B)" % sent)
        data_s = time.time() - t0 - erase_s

        # 3) END：等最终 ACK 验收（boot 应答后即搬运+跳转）
        end_ok = False
        for attempt in range(1, END_RETRY + 1):
            if self._stop:
                self._transport.close(); self.done.emit(False, "用户中止")
                return
            self._transport.send(P.build_ota_end(len(fw)))
            ack = self._wait_ack(timeout=END_TIMEOUT, expect_rsv=0)
            if ack is not None:
                if ack.ok:
                    end_ok = True
                else:
                    self._transport.close(); self.err.emit("OTA_END ACK=ERR（长度不符）")
                    self.done.emit(False, "验收失败")
                    return
                break
            self.err.emit("OTA_END 无应答，重试 %d/%d" % (attempt, END_RETRY))
        elapsed = time.time() - t0
        self._transport.close()
        if self._stop:
            self.done.emit(False, "用户中止")
            return
        if end_ok:
            # 吞吐按固件净字节算，不含帧头/ACK 开销，故低于链路波特率属正常。
            # ★ 擦除段是固定成本（F407 单 bank，最小擦除单位 128KB 扇区，与固件大小无关），
            #   所以"含擦除的总吞吐"会随固件变小而暴跌。要衡量链路能力看【传输段】那个数。
            kbps = len(fw) * 8.0 / elapsed / 1000.0 if elapsed > 0 else 0.0
            kbs = len(fw) / elapsed / 1024.0 if elapsed > 0 else 0.0
            dkbps = len(fw) * 8.0 / data_s / 1000.0 if data_s > 0 else 0.0
            ms_per = data_s * 1000.0 / total if total else 0.0
            self.log.emit("OTA_END ACK=OK，升级验收通过！")
            self.log.emit("分段: 擦除下载区 %.2f s | 传输 %.2f s -> 传输段 %.1f kbps (每帧 %.1f ms)"
                          % (erase_s, data_s, dkbps, ms_per))
            self.log.emit("升级耗时 %.2f 秒，含擦除总吞吐 %.1f kbps (%.2f KB/s)" % (elapsed, kbps, kbs))
            self.done.emit(True, "升级成功 —— 耗时 %.2f 秒（擦除 %.2f + 传输 %.2f），"
                                 "传输段 %.1f kbps" % (elapsed, erase_s, data_s, dkbps))
        else:
            self.err.emit("OTA_END 应答丢失（若已跳转运行 app 属正常，否则请整包重发）")
            self.done.emit(False, "END 超时")
