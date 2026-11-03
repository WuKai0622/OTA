#include "protocol.h"
#include "flash_if.h"
#include "update.h"

/* ==================== CRC16-MODBUS 查表（poly 0xA001, init 0xFFFF） ====================
 * 本表与上位机 tools/ota_proto.py 逐值相同（由 Python 生成，保证两端一致）。
 */
static const uint16_t crc16_table[256] = {
   0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241, 0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
   0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40, 0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
   0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40, 0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
   0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641, 0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
   0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240, 0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
   0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41, 0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
   0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41, 0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
   0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640, 0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
   0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240, 0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
   0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41, 0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
   0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41, 0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
   0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640, 0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
   0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241, 0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
   0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40, 0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
   0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40, 0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
   0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641, 0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040,
};

uint16_t CRC16_Modbus(const uint8_t *p, uint32_t len)
{
    uint16_t crc = 0xFFFFu;
    uint32_t i;

    for (i = 0; i < len; i++)
    {
        crc = (uint16_t)((crc >> 8) ^ crc16_table[(crc ^ p[i]) & 0xFFu]);
    }
    return crc;
}

/* ==================== 帧状态机 ====================
 * 帧序：| HEAD(2) | CMD(2) | LEN(2) | DATA(N) | RSV(4) | CRC(2) |  总长 12+N
 * frm[0..1] HEAD, frm[2..3] CMD, frm[4..5] LEN,
 * frm[6..6+N) DATA, frm[6+N..10+N) RSV, frm[10+N..12+N) CRC
 * CRC 覆盖 frm[2 .. 10+N) 即 [CMD..RSV] 共 8+N 字节（不含包头/CRC）。
 *
 * 帧同步：只在 ST_WAIT_HEAD0/HEAD1 里搜索 0xA5 0x5A；一旦进入 DATA/RSV/CRC
 * 就按 LEN 盲收固定字节数，数据区里的 0xA5 0x5A 不会被误判为帧头。
 */
typedef enum {
    ST_WAIT_HEAD0,     /* 等 0xA5 */
    ST_WAIT_HEAD1,     /* 等 0x5A */
    ST_WAIT_HDR,       /* 收 CMD+LEN 共 4B -> frm[2..5] */
    ST_WAIT_DATA,      /* 收 N 字节数据（N>0） */
    ST_WAIT_RSV,       /* 收 4B 保留 -> frm[6+N..10+N) */
    ST_WAIT_CRC        /* 收 2B CRC -> frm[10+N..12+N) */
} ProtoState_t;

static ProtoState_t  g_st;                 /* 状态机状态 */
static uint8_t       g_frm[FRAME_BUF_SIZE]; /* 整帧暂存（含包头） */
static uint16_t      g_frm_len;            /* 已收字节数（绝对写入下标） */
static uint16_t      g_frm_data_len;       /* 当前帧 LEN（数据区字节数） */
static uint32_t      g_ota_len_expected;   /* ★ F407 固件上限 128KB —— 必须 uint32_t，
                                            *   用 uint16_t 会在大固件时溢出，导致
                                            *   OTA_END 的长度核对永远不相等 */
static uint32_t      g_ota_rx_total;       /* 已写下载区累计字节 */
static uint8_t       g_ota_active;         /* 1=已进入下载会话(收到合法 OTA_START) */
static Proto_Tx_t    g_tx = 0;             /* 发送出口（由 Proto_SetTx 注入） */

/* ---- 会话绑定（见 protocol.h 顶部那段说明）----
 * g_cur_ch  : 最后一次收到数据的通道。没有锁存时的兜底选择。
 * g_sess_chan: 本次升级会话**锁存**的通道，收到 OTA_START 时置位，之后不再变。
 *             应答一律回它 —— 与"发送时刻哪条通道连着"无关。
 * 两者都是 OTA_CH_NONE 时（从未收到过任何字节）出口回落串口，
 * 与加 SPI 通道之前的行为一致（那时是"TCP 没连接就走串口"）。 */
static OtaChan_t     g_cur_ch = OTA_CH_NONE;
static OtaChan_t     g_sess_chan = OTA_CH_NONE;

/* 本次会话该往哪条通道发。
 * ★ 只此一处做通道选择 —— 出口（BootTxEx）、三条通道驱动都不再判断"谁连着"。 */
static OtaChan_t TxChan(void)
{
    return (g_sess_chan != OTA_CH_NONE) ? g_sess_chan : g_cur_ch;
}

/* 已写下载区字节数（供 LCD 显示，Bootloader.c 同量） */
extern uint32_t g_dl_off;

static void Proto_HandleFrame(void);
static void SendAckEx(uint8_t code, uint32_t rsv);
static void SendAck(uint8_t code);

/* ★ 本函数是**整个协议层的复位**（上电时调用；也可在运行期重入，用于
 *   "放弃当前会话、重开一个"—— 常驻下载循环的空闲超时就靠它拆会话，
 *   见 Bootloader.c 的 BOOT_DL_IDLE_MS）。因此除了帧状态机与 g_ota_active，
 *   会话的**全部**状态都必须在这里清掉，绝不能只清一部分 —— 见下方第二段。 */
void Proto_Init(void)
{
    g_st = ST_WAIT_HEAD0;
    g_frm_len = 0;
    g_frm_data_len = 0;
    g_ota_active = 0u;   /* 每次上电复位会话标志：等待窗口据它判断是否进入下载 */

    /* ★ 会话计数与进度也必须清。漏掉这三个会让"拆会话"名不副实：
     *   g_ota_len_expected 残留上一次的长度、g_ota_rx_total 残留 0，于是一条
     *   off=0 的 DATA 帧就能穿过 CMD_OTA_DATA 分支里的
     *   `off == g_ota_rx_total` 和 `(off+n) <= g_ota_len_expected` 两道判断，
     *   在**没有任何会话**的情况下直接 Flash_ProgramBytes() 写下载区 ——
     *   没有 START、没有擦除，是最危险的一种写。
     *   （CMD_OTA_DATA 分支另有一道显式的 g_ota_active 门禁做兜底，
     *     但根因在这里：不清干净，门禁之外的路径照样漏。） */
    g_ota_len_expected = 0u;
    g_ota_rx_total     = 0u;
    g_dl_off           = 0u;   /* LCD 显示量：一并归零，屏上不会残留上次的进度 */

    /* ★ 通道也必须一起复位。上电时 g_sess_chan 靠静态零初始化本来就是 NONE，
     *   但这里显式写一遍：将来若有人在运行期调用 Proto_Init()（比如"放弃本次
     *   升级、重开一个会话"），漏掉这一行就会把上一次锁存的通道沿用下去 ——
     *   表现是"新会话的 ACK 发去了上一条通道"，与本次要修的 bug 一模一样。 */
    g_cur_ch    = OTA_CH_NONE;
    g_sess_chan = OTA_CH_NONE;
}

void Proto_SetTx(Proto_Tx_t tx)
{
    g_tx = tx;
}

/* 收到合法完整帧后按命令分发 */
static void Proto_HandleFrame(void)
{
    uint16_t cmd   = (uint16_t)(g_frm[2] | ((uint16_t)g_frm[3] << 8));
    uint16_t n     = g_frm_data_len;
    uint8_t *data  = &g_frm[6];
    uint32_t rsv   = (uint32_t)g_frm[6 + n]
                   | ((uint32_t)g_frm[7 + n] << 8)
                   | ((uint32_t)g_frm[8 + n] << 16)
                   | ((uint32_t)g_frm[9 + n] << 24);
    uint32_t len32;
    uint32_t tk_lo;
    uint32_t tk_hi;

    if (cmd == CMD_OTA_START)
    {
        /* ★★ 会话绑定就在这里锁存，**在合法性校验之前**，但**只在没有活动会话时**。
         *
         * 为什么要在校验之前锁存：START 被拒时也要回 ACK_ERR，而那条 ERR 同样
         *   必须回**发起方所在的那条通道**。放在校验之后的话，一个"被拒的 START"
         *   会用上一次会话锁存的旧通道回 ERR，PC 在本次通道上白等。
         *   （没有活动会话时 TxChan() 本就会回落到 g_cur_ch，而 g_cur_ch 是
         *     Proto_Feed 刚刚按本帧来源设过的，所以这条 ERR 照样回到发起方。）
         * 为什么必须加 g_ota_active 这个前提：不加的话，一个**被拒的 START**
         *   （令牌不对/长度非法/擦除失败）会把**正在进行的**会话的应答通道改掉 ——
         *   正是本文件反复强调要避免的"ACK 发去了别的通道"。加了令牌之后 START
         *   多了一种被拒理由，这条路径才变得容易被外部触发。
         * 安全性不靠"只在合法时锁存"来保证 —— 能走到这里的帧 CRC 已经过了，
         *   而 CRC 是在 ST_WAIT_CRC 里校验通过才调用本函数的。 */
        if (g_ota_active == 0u)
        {
            g_sess_chan = g_cur_ch;
        }

        /* ---- ① 数据区必须够长：12B = 固件总长 u32(4B) + 令牌(8B) ----
         * 老版本上位机只发 4B，走的就是这一条 —— **fail-closed**（拒绝），
         * 这是正确方向：绝不能因为"少读了 8 字节"就把校验整个跳过。 */
        if (n < OTA_START_DATA_LEN)
        {
            SendAck(RSP_ERR);
            return;
        }

        /* ---- ② 令牌校验：放在长度校验和擦除**之前** ----
         * 令牌不对就一个扇区都不动 —— 这是令牌存在的全部意义。
         * 按小端 u32 两段比较，与 ota_proto.py 的 struct.pack("<II", ...) 对应。 */
        tk_lo = (uint32_t)data[4] | ((uint32_t)data[5] << 8)
              | ((uint32_t)data[6] << 16) | ((uint32_t)data[7] << 24);
        tk_hi = (uint32_t)data[8] | ((uint32_t)data[9] << 8)
              | ((uint32_t)data[10] << 16) | ((uint32_t)data[11] << 24);
        if (tk_lo != OTA_TOKEN_W0 || tk_hi != OTA_TOKEN_W1)
        {
            SendAck(RSP_ERR);              /* 令牌不符：拒绝，不建会话、不擦除 */
            return;
        }

        /* ---- ③ 固件总长 L ---- */
        len32 = (uint32_t)data[0] | ((uint32_t)data[1] << 8)
              | ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);

        if (len32 == 0u || len32 > APP_MAX_SIZE)
        {
            SendAck(RSP_ERR);              /* 越界/非法长度：拒绝 */
            return;
        }
        /* 擦除下载区中固件实际覆盖的全部扇区。
         * ★ F4 的擦除粒度是扇区且最小 128KB —— 即使只有 13KB 固件也要擦一整个
         *   S7（典型 1s、最坏 4s），且擦除期间 Flash 总线 stall、CPU 冻结、lwIP 停摆。
         *   这段耗时发生在 ACK 之前，上位机第一次等待(当前 2.0s)必须 ≥ 实测擦除 1~1.2s，
         *   否则会重发并再擦一遍；第 2/3 次等待 4.5s 覆盖 ST 规格最坏值。见 tools/README.md。 */
        if (Flash_EraseRegion(APP_DL_ADDR, len32) != 0u)
        {
            SendAck(RSP_ERR);              /* 擦除失败：拒绝本次升级 */
            return;
        }
        g_ota_len_expected = len32;
        g_ota_rx_total = 0u;
        g_dl_off = 0u;
        g_ota_active = 1u;                 /* 进入下载会话：窗口据此中断等待 */
        SendAck(RSP_OK);                   /* 擦除在 ACK 前完成，PC 等 ACK 后才发数据 */
    }
    else if (cmd == CMD_OTA_DATA)
    {
        /* RSV = 本帧起始偏移 off。逐帧应答（配合上位机 stop-and-wait 重发）：
         *   写成功或重复帧(ACK 丢失被重发, 已提交)   -> ACK_OK(RSV=off)，不重复写
         *   乱序/越界/写失败                        -> ACK_ERR(RSV=off)
         *   空帧或半字错对齐                         -> 不答，上位机超时重发 */
        uint32_t off = rsv;
        uint8_t  ack_code;

        if (n == 0u || (off & 1u) != 0u)
        {
            return;
        }

        /* ★ 显式门禁：没有活动会话时绝不写 Flash。
         *   Proto_Init() 已经把 g_ota_len_expected/g_ota_rx_total 清 0，
         *   所以这一条在当前代码下走不到 —— 它是一道**兜底**护栏，把
         *   "写 Flash 必须先有会话"这个不变量写在写操作旁边，
         *   而不是依赖两个计数器的初值恰好让下面两判断不成立。
         *   ★ 必须放在上面那两行对齐检查**之后**：一是 off 要先赋值，
         *     二是现在空帧/半字错对齐是**静默丢弃**（靠上位机超时重发），
         *     门禁提到前面会把这种"无法判定"变成一次 ERR 应答。 */
        if (g_ota_active == 0u)
        {
            SendAckEx(RSP_ERR, off);
            return;
        }

        if (off == g_ota_rx_total)
        {
            if ((off + n) <= g_ota_len_expected)
            {
                if (Flash_ProgramBytes(APP_DL_ADDR + off, data, n) == 0u)
                {
                    g_ota_rx_total += n;
                    g_dl_off = g_ota_rx_total;
                    ack_code = RSP_OK;
                }
                else
                {
                    ack_code = RSP_ERR;    /* 写失败：NACK，上位机重发本帧 */
                }
            }
            else
            {
                ack_code = RSP_ERR;        /* 越界：NACK */
            }
        }
        else if ((off + n) <= g_ota_rx_total)
        {
            ack_code = RSP_OK;             /* 重复帧：已提交过，幂等确认，不重写 */
        }
        else
        {
            ack_code = RSP_ERR;            /* off > 已写：乱序缺帧，NACK */
        }
        SendAckEx(ack_code, off);
    }
    else if (cmd == CMD_OTA_END)
    {
        len32 = (uint32_t)data[0] | ((uint32_t)data[1] << 8)
              | ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);

        /* ★ 与 DATA 分支同一条门禁：没有活动会话时不做任何会话动作。
         *   不能省 —— Proto_Init() 把两个计数清 0 后，一条 len32=0 的野 END
         *   会满足 `len32 == g_ota_len_expected(0) && g_ota_rx_total(0) == 0`，
         *   拿到一个 RSP_OK（Upd_OnDownloadComplete 内部虽有 total_len==0 的
         *   兜底、不会真写 Flash，但对一个无意义帧回"验收通过"本身就是错的），
         *   且与 DATA 分支"无会话就 ERR"的行为自相矛盾。 */
        if (g_ota_active == 0u)
        {
            SendAck(RSP_ERR);
            return;
        }

        if (n >= 4u && len32 == g_ota_len_expected && g_ota_rx_total == g_ota_len_expected)
        {
            SendAck(RSP_OK);               /* 本次升级写入验收通过 */
            Upd_OnDownloadComplete(g_ota_len_expected);  /* 候选入 run + 跳转；失败则停留 */
        }
        else
        {
            SendAck(RSP_ERR);
        }
    }
    /* 其它命令忽略（状态机已复位重扫） */
}

/* 发送应答帧：HEAD + CMD_ACK + LEN=1 + code + RSV(4B 小端) + CRC（覆盖 CMD..RSV）
 * RSV 携带被确认的偏移：DATA 帧应答用它告诉上位机"本偏移已写入/需重发"，
 * START/END 应答 RSV=0。上位机靠 RSV 做逐帧确认/幂等重发。
 *
 * ★ 通道由 TxChan() 统一算出（会话已锁存则用锁存值），出口只负责照着发。
 *   "选哪条通道"这个决策必须只有一处 —— 散到出口函数里就会变成三处，
 *   而三处里任意一处判错都是"ACK 发去了别的通道"这种偶发且难查的故障。 */
static void SendAckEx(uint8_t code, uint32_t rsv)
{
    uint8_t buf[13];
    uint16_t crc;

    buf[0] = PKT_HEAD0;
    buf[1] = PKT_HEAD1;
    buf[2] = (uint8_t)(CMD_RSP_ACK & 0xFFu);
    buf[3] = (uint8_t)((CMD_RSP_ACK >> 8) & 0xFFu);
    buf[4] = 1u;                          /* LEN = 1 */
    buf[5] = 0u;
    buf[6] = code;                        /* DATA */
    buf[7] = (uint8_t)(rsv & 0xFFu);      /* RSV：确认偏移（数据帧）或 0（START/END） */
    buf[8] = (uint8_t)((rsv >> 8) & 0xFFu);
    buf[9] = (uint8_t)((rsv >> 16) & 0xFFu);
    buf[10] = (uint8_t)((rsv >> 24) & 0xFFu);
    crc = CRC16_Modbus(&buf[2], 9u);      /* CMD+LEN+DATA+RSV = 2+2+1+4 = 9B */
    buf[11] = (uint8_t)(crc & 0xFFu);
    buf[12] = (uint8_t)((crc >> 8) & 0xFFu);

    if (g_tx != 0)
    {
        (void)g_tx(TxChan(), buf, 13u);   /* 失败不重发：由上位机超时重发兜底 */
    }
}

static void SendAck(uint8_t code)
{
    SendAckEx(code, 0u);
}

/* 查询是否已进入下载会话（main 窗口编排用：真则永久等待，不执行启动决策） */
uint8_t Proto_SessionStarted(void)
{
    return g_ota_active;
}

/* 喂入字节流，驱动状态机；收满一帧后校验并分发。
 * ch = 这段字节来自哪条通道（由 producer 决定，见 protocol.h）。 */
void Proto_Feed(const uint8_t *p, uint16_t len, OtaChan_t ch)
{
    uint16_t i;

    /* ★ 在循环**之前**记下来源，而不是在 Proto_HandleFrame 里现取：
     *   Proto_HandleFrame 是由**收满整帧那一次**调用触发的，而那一次不一定就是
     *   第一段 —— 一帧可能被拆在多次 Proto_Feed 里到达（SPI 每笔事务只有 256B，
     *   一个 1036B 的 DATA 帧要 5 笔才凑齐）。在循环前记下，当下这一段的来源就
     *   是确定的。
     *
     * ★ 注意由此得到的语义是"**以收满整帧的那一段为准**"，不是"以第一段为准"。
     *   同一帧的各段来自同一条通道时两者等价（正常情况都是这样），
     *   真的混了通道才会显出差别 —— 那已经是 boot 侧 g_ring_src 那个
     *   单通道前提被破坏的信号了，见 Bootloader.c 的"已知假设"。 */
    g_cur_ch = ch;

    for (i = 0; i < len; i++)
    {
        uint8_t b = p[i];

        switch (g_st)
        {
        case ST_WAIT_HEAD0:
            if (b == PKT_HEAD0)
            {
                g_frm[0] = b;
                g_st = ST_WAIT_HEAD1;
            }
            break;

        case ST_WAIT_HEAD1:
            if (b == PKT_HEAD1)
            {
                g_frm[1] = b;
                g_st = ST_WAIT_HDR;
                g_frm_len = 2;
            }
            else if (b != PKT_HEAD0)      /* 非包头续字节：回到等头0 */
            {
                g_st = ST_WAIT_HEAD0;
            }
            break;

        case ST_WAIT_HDR:                  /* 收 CMD+LEN 共 4B -> frm[2..5] */
            g_frm[g_frm_len++] = b;
            if (g_frm_len == 6u)          /* 2 包头 + 4 头字段 = 6 */
            {
                g_frm_data_len = (uint16_t)(g_frm[4] | ((uint16_t)g_frm[5] << 8));
                if (g_frm_data_len > FRAME_DATA_MAX)  /* LEN 越界：坏帧丢弃重扫 */
                {
                    g_st = ST_WAIT_HEAD0;
                    break;
                }
                g_st = (g_frm_data_len > 0u) ? ST_WAIT_DATA : ST_WAIT_RSV;
            }
            break;

        case ST_WAIT_DATA:                 /* 收 N 字节 -> frm[6..6+N) */
            g_frm[g_frm_len++] = b;
            if (g_frm_len == 6u + g_frm_data_len)
            {
                g_st = ST_WAIT_RSV;
            }
            break;

        case ST_WAIT_RSV:                  /* 收 4B 保留 -> frm[6+N..10+N) */
            g_frm[g_frm_len++] = b;
            if (g_frm_len == 10u + g_frm_data_len)
            {
                g_st = ST_WAIT_CRC;
            }
            break;

        case ST_WAIT_CRC:                  /* 收 2B CRC -> frm[10+N..12+N) */
            g_frm[g_frm_len++] = b;
            if (g_frm_len == 12u + g_frm_data_len)
            {
                uint16_t calc = CRC16_Modbus(&g_frm[2], 8u + g_frm_data_len);
                uint16_t recv = (uint16_t)(g_frm[10u + g_frm_data_len]
                              | ((uint16_t)g_frm[11u + g_frm_data_len] << 8));

                g_st = ST_WAIT_HEAD0;      /* 无论好坏，处理完一帧即复位重扫 */
                if (calc == recv)
                {
                    Proto_HandleFrame();
                }
            }
            break;

        default:
            g_st = ST_WAIT_HEAD0;
            break;
        }
    }
}
