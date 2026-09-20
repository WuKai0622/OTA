# -*- coding: utf-8 -*-
"""
OTA 上位机共享协议层 —— 与 Bootloader/protocol.c 严格同口径。

帧格式（考试自定义，参考 modbus）：
    | HEAD(2) | CMD(2) | LEN(2) | DATA N | RSV(4) | CRC16(2) |
      0xA5 0x5A   LE      LE=N      N<=1024    LE       LE
CRC16-MODBUS 覆盖 [CMD..RSV] 共 8+LEN 字节（不含包头、不含 CRC），init=0xFFFF。
帧总长 = 12 + N（含包头）。

本模块由上位机(GUI 与 CLI)共享；CRC 表与 C 端 const 数组逐值相同，
供 --selftest 交叉校验两端一致。
"""
import struct

PKT_HEAD = b"\xA5\x5A"
FRAME_DATA_MAX = 1024   # 单帧数据区上限（boot 侧协议硬上限，帧总长上限 12+1024）
OTA_DATA_CHUNK = 1024   # 实际每个 DATA 帧携带的固件字节数(=boot 侧 FRAME_DATA_MAX 上限)。
                        # 取满上限=帧数最少，帧固定开销摊到最薄（128KB 固件 128 帧）。
                        # ETH 通道下帧总长 12+1024=1036B，加 TCP/IP 头 ~40B 仍 < MTU 1500，
                        # 不会被 IP 分片 —— 这也是 boot 侧 lwipopts.h 要把 TCP_MSS
                        # 提到 1460 的原因（536 会把这个帧拆成 2 个 TCP 段）。

# 命令码（与 protocol.h 一致）
CMD_OTA_START = 0x0001   # PC->boot：开始，数据=固件总长 u32 LE，RSV=目标区(0=A)
CMD_OTA_DATA = 0x0002    # PC->boot：固件数据帧，RSV=本帧起始偏移 u32 LE
CMD_OTA_END = 0x0003     # PC->boot：结束，数据=固件总长 u32 LE
CMD_RSP_ACK = 0x0100     # boot->PC：应答，数据=状态码 1B(0=OK,1=ERR)

RSP_OK = 0
RSP_ERR = 1

APP_MAX_SIZE = 131072    # 固件上限 128KB —— 与 F407 工程 flash_if.h 的 APP_MAX_SIZE 一致。
                         # (F103 版本是 15360；本工程已移植到 F407ZGT6，boot 镜像本身就
                         #  有 77KB，沿用旧的 15KB 上限会让任何固件都在发包前被拒)


def _build_table():
    """生成 CRC16-MODBUS 查表（poly 0xA001 反射版）。"""
    tbl = []
    for i in range(256):
        c = i
        for _ in range(8):
            c = (c >> 1) ^ 0xA001 if (c & 1) else (c >> 1)
        tbl.append(c)
    return tbl


_CRC_TABLE = _build_table()


def crc16_modbus(data: bytes, crc: int = 0xFFFF) -> int:
    """CRC16-MODBUS：查表逐字节更新，返回 16 位结果。"""
    for b in data:
        crc = (crc >> 8) ^ _CRC_TABLE[(crc ^ b) & 0xFF]
    return crc & 0xFFFF


def crc16_modbus_c_array() -> str:
    """输出与 C 端一致的 const 表文本（每行 16 项）。"""
    lines = []
    for r in range(0, 256, 16):
        lines.append("   " + ", ".join("0x%04X" % v for v in _CRC_TABLE[r:r + 16]) + ",")
    return "\n".join(lines)


def build_frame(cmd: int, data: bytes = b"", rsv: int = 0) -> bytes:
    """组帧：cmd 小端、len 小端、rsv 小端 u32、CRC 覆盖 [CMD..RSV]。"""
    if len(data) > FRAME_DATA_MAX:
        raise ValueError("data too long: %d > %d" % (len(data), FRAME_DATA_MAX))
    hdr = PKT_HEAD + struct.pack("<HH", cmd, len(data))
    rsv_b = struct.pack("<I", rsv)
    crc = crc16_modbus(hdr[2:] + data + rsv_b)   # 从 CMD 起，不含包头
    return hdr + data + rsv_b + struct.pack("<H", crc)


def build_ota_start(total_len: int, target: int = 0) -> bytes:
    return build_frame(CMD_OTA_START, struct.pack("<I", total_len), rsv=target)


def build_ota_data(off: int, chunk: bytes) -> bytes:
    return build_frame(CMD_OTA_DATA, chunk, rsv=off)


def build_ota_end(total_len: int) -> bytes:
    return build_frame(CMD_OTA_END, struct.pack("<I", total_len), rsv=0)


def split_firmware(bin_: bytes) -> "list[tuple[int, bytes]]":
    """按 OTA_DATA_CHUNK(1024B)分包，返回 [(offset, chunk)]。offset 恒为
    1024 倍数（4 字节对齐，满足 boot 侧 Flash_ProgramBytes 的字编程对齐约束）。"""
    if len(bin_) > APP_MAX_SIZE:
        raise ValueError("firmware %d B > APP_MAX_SIZE %d" % (len(bin_), APP_MAX_SIZE))
    chunk = min(OTA_DATA_CHUNK, FRAME_DATA_MAX)
    return [(i, bin_[i:i + chunk])
            for i in range(0, len(bin_), chunk)]


class Ack:
    """应答帧解析结果。"""
    def __init__(self, ok: bool, code: int, cmd: int, raw: bytes, rsv: int = 0):
        self.ok = ok          # 状态码 == RSP_OK
        self.code = code      # 原始状态码
        self.cmd = cmd        # 应答的命令（CMD_RSP_ACK=0x0100 或未知）
        self.rsv = rsv        # 应答帧 RSV：DATA 帧应答 = 被确认的偏移；START/END = 0
        self.raw = raw

    def __repr__(self):
        return "Ack(ok=%s, code=%s, rsv=%s)" % (self.ok, self.code, self.rsv)


class FrameScanner:
    """从字节流中扫描出一帧：处理串口粘包/拆包/坏帧。
    用法：多次 push(bytes)，每次返回可能的一帧完整 bytes；无效数据被丢弃重扫。"""

    def __init__(self):
        self._buf = bytearray()

    def push(self, data: bytes):
        self._buf += data

    def next_frame(self) -> "bytes | None":
        """尝试从缓冲中解析一帧；不足则返回 None（等更多数据）。
        帧长 = 12 + LEN；LEN 在偏移 4..5（小端）。"""
        while True:
            # 1) 找包头
            i = self._buf.find(PKT_HEAD)
            if i < 0:
                self._buf.clear()
                return None
            if i > 0:                     # 丢包头前垃圾
                del self._buf[:i]
            if len(self._buf) < 4:        # 至少要读到 LEN
                return None
            n = self._buf[4] | (self._buf[5] << 8)
            if n > FRAME_DATA_MAX:        # 越界长度：坏包头，丢 1 字节重扫
                del self._buf[:1]
                continue
            need = 12 + n
            if len(self._buf) < need:
                return None
            frm = bytes(self._buf[:need])
            del self._buf[:need]
            # 校验 CRC（覆盖 CMD..RSV = frm[2:2+8+n]）
            calc = crc16_modbus(frm[2:2 + 8 + n])
            recv = frm[2 + 8 + n] | (frm[2 + 8 + n + 1] << 8)
            if calc != recv:
                continue                  # 坏帧：丢弃，继续扫下一帧
            return frm

    def clear(self):
        self._buf.clear()


def parse_frame(frm: bytes):
    """解析一帧，返回 (cmd, data, rsv)。调用方需已通过 CRC 校验（next_frame 已做）。"""
    cmd = frm[2] | (frm[3] << 8)
    n = frm[4] | (frm[5] << 8)
    data = frm[6:6 + n]
    rsv = struct.unpack("<I", frm[6 + n:10 + n])[0]
    return cmd, data, rsv
