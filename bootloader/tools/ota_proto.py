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
CMD_OTA_START = 0x0001   # PC->boot：开始，数据=固件总长 u32 LE + 令牌 8B，RSV=目标区(0=A)
CMD_OTA_DATA = 0x0002    # PC->boot：固件数据帧，RSV=本帧起始偏移 u32 LE
CMD_OTA_END = 0x0003     # PC->boot：结束，数据=固件总长 u32 LE
CMD_RSP_ACK = 0x0100     # boot->PC：应答，数据=状态码 1B(0=OK,1=ERR)

RSP_OK = 0
RSP_ERR = 1

def token_hex(b: bytes) -> str:
    """令牌的显示形式：'A1 B2 C3 D4 E5 F6 07 18'（每字节两位大写 hex）。
    ★ 必须用 hex 而不是 decode：令牌是二进制，可能含 0x00，decode 出来是一串
      控制字符，终端上看着像乱码，会把"令牌不对"这种最简单的排查方向挡掉。"""
    return " ".join("%02X" % x for x in b)


# ---- OTA_START 的 8 字节令牌，与 protocol.h 的 OTA_TOKEN_W0/W1 逐字节对应 ----
#   boot 侧按**小端 u32 两段**比较：OTA_TOKEN_W0(0x06000002) 展开 = 02 00 00 06
#                                  OTA_TOKEN_W1(0x02020600) 展开 = 00 06 02 02
# ★ 令牌现在是**二进制字节**（见下面两个常量），不是可读串 —— 所以：
#     · 上位机不接受直接输入这 8 个字节，而是让操作者按 16 位 hex 输入
#       （见 parse_token，两种写法都收）；
#     · 日志一律用 token_hex() 显示，**不要** decode 成字符串（会印出控制字符）。
# ★ 令牌的具体值**只写在源码里**（本文件的两个常量 + boot 侧 protocol.h）。界面提示
#   和三份 README 都不给出取值，操作者必须自己去源码取 —— 否则"填令牌"这个动作
#   在文档里就被代劳了，等于没设。
# ★ 这里以那对 u32 常量为准（与 C 侧同源），再用断言校验它小端展开后**确实**是预期的
#   8 个字节 —— 手写 hex 一旦按"自然阅读顺序"（大端）写，或把 W0/W1 填反，与 C 侧的
#   小端比较就永远对不上，而症状只是"每条 START 都被拒"，与令牌设错完全无从区分
#   （boot 侧只会回一个 RSP_ERR）。断言让这种错在 import 阶段就炸出来，
#   而不是等到连板子才以"升级失败"的面目出现。
# ★ 若将来把令牌挪出仓库（Keil 的 -D 覆盖 protocol.h 的 #ifndef），改这里的
#   OTA_TOKEN 即可，两个 u32 常量随之同步更新（下方断言会强制它们一致）。
OTA_TOKEN_W0 = 0x06000002
OTA_TOKEN_W1 = 0x02020600
OTA_TOKEN = struct.pack("<II", OTA_TOKEN_W0, OTA_TOKEN_W1)
assert OTA_TOKEN == bytes.fromhex("0200000600060202"), (
    "OTA_TOKEN_W0/W1 与预期的 8 字节令牌不符：%s" % token_hex(OTA_TOKEN))

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


_HEX_DIGITS = set("0123456789abcdefABCDEF")


def parse_token(s: str) -> bytes:
    """把操作者填写的令牌**字符串**转成 8 字节，供 CLI / GUI 共用同一套校验。

    ★ 为什么要有这个函数而不是各自 encode：令牌长度是协议约束（boot 侧
      OTA_START_DATA_LEN=12 = 4 + 8），填错了 boot 只会回一个 RSP_ERR，
      现场看就是"升级失败"，与令牌本身不对完全无从区分。校验放在发第一个
      字节之前，报错就明确指向"令牌"这一项。
    ★ 接受的两种写法（当前令牌是**二进制**，只能用第一种；第二种留给
      "令牌恰好是可读 8 字符"的场合）：
        ① 16 位 hex（大小写不限，空格/'-'/':' 分隔符可有可无）
           "0123456789ABCDEF" / "01 23 45 67 89 AB CD EF" → 同 8 字节
        ② 8 个字符的原样串（UTF-8 后恰好 8 字节）
    两种写法不会冲突：hex 形式恒为 16 个 hex 字符（= 8 字节），而任何可读串
    要么不是 16 个 hex 字符，要么长度就不是 8 字节 —— 没有第二种解释。
    ★ 上面两个例子只是**格式示意**，不是本机当前令牌；当前值见本文件顶部的
      OTA_TOKEN_W0/W1（boot 侧同源常量在 protocol.h）。
    非法时抛 ValueError，消息直接给操作者看。"""
    if s is None:
        raise ValueError("令牌不能为空（必填 8 字节）")
    s = s.strip()
    if s == "":
        raise ValueError("令牌不能为空（必填 8 字节）")

    # ① hex 形式：去掉分隔符后正好 16 个 hex 字符
    compact = s.replace(" ", "").replace("-", "").replace(":", "")
    if len(compact) == 16 and all(c in _HEX_DIGITS for c in compact):
        return bytes.fromhex(compact)

    # ② 原样 8 字符
    b = s.encode("utf-8")
    if len(b) != 8:
        raise ValueError("令牌必须是 8 字节：请填 16 位 hex（如 0123456789ABCDEF）"
                         "或 8 个字符；当前 %d 字节：%r" % (len(b), s))
    return b


def build_ota_start(total_len: int, target: int = 0, token: bytes = None) -> bytes:
    """OTA_START：数据区 = 固件总长 u32 小端 + 8 字节令牌（共 12B，对应 boot 侧
    OTA_START_DATA_LEN）。令牌默认用模块级的 OTA_TOKEN；显式传入可用于构造
    "错令牌"的负向测试（boot 侧应回 RSP_ERR 且不擦除任何扇区）。"""
    if token is None:
        token = OTA_TOKEN
    if len(token) != 8:
        raise ValueError("令牌必须是 8 字节，当前 %d 字节" % len(token))
    return build_frame(CMD_OTA_START, struct.pack("<I", total_len) + token, rsv=target)


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
