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


class OtaWorker(QThread):
    log = Signal(str)                 # 普通日志行
    err = Signal(str)                 # 错误/失败行
    progress = Signal(int, int, str)  # (已发帧, 总帧数, 状态文本)
    done = Signal(bool, str)          # (成功?, 摘要)

    def __init__(self, transport, bin_path: str, target: int = 0,
                 delay_per_frame: float = 0.1, parent=None):
        super().__init__(parent)
        self._transport = transport
        self._bin_path = bin_path
        self._target = target
        self._delay = delay_per_frame
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
            if not self._stop:
                time.sleep(INTER_FRAME_GAP)
            self.progress.emit(i + 1, total, "已发 %d/%d 帧 (%d B)" % (i + 1, total, sent))

        self.log.emit("数据帧已全部确认 (%d B)" % sent)

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
        self._transport.close()
        if self._stop:
            self.done.emit(False, "用户中止")
            return
        if end_ok:
            self.log.emit("OTA_END ACK=OK，升级验收通过！")
            self.done.emit(True, "升级成功")
        else:
            self.err.emit("OTA_END 应答丢失（若已跳转运行 app 属正常，否则请整包重发）")
            self.done.emit(False, "END 超时")
