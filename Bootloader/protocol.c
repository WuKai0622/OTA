#include "protocol.h"
#include "flash_if.h"
#include "usart.h"
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
static uint16_t      g_ota_len_expected;   /* 本次升级固件总长 */
static uint32_t      g_ota_rx_total;       /* 已写下载区累计字节 */
static uint8_t       g_ota_active;         /* 1=已进入下载会话(收到合法 OTA_START) */

/* 已写下载区字节数（供 OLED 显示，Bootloader.c 同量） */
extern uint32_t g_dl_off;

static void Proto_HandleFrame(void);
static void SendAckEx(uint8_t code, uint32_t rsv);
static void SendAck(uint8_t code);

void Proto_Init(void)
{
    g_st = ST_WAIT_HEAD0;
    g_frm_len = 0;
    g_frm_data_len = 0;
    g_ota_active = 0u;   /* 每次上电复位会话标志：5s 窗口据它判断是否进入下载 */
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

    if (cmd == CMD_OTA_START)
    {
        /* 数据区 4B = 固件总长 L */
        len32 = (uint32_t)data[0] | ((uint32_t)data[1] << 8)
              | ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);

        if (n < 4u || len32 == 0u || len32 > APP_MAX_SIZE)
        {
            SendAck(RSP_ERR);              /* 越界/非法长度：拒绝 */
            return;
        }
        /* 整区擦除下载区（按固件长度折算页数，上限为 A 区 15 页） */
        {
            uint32_t pages = (len32 + FLASH_PAGE - 1u) / FLASH_PAGE;
            if (pages > APP_MAX_SIZE / FLASH_PAGE)
            {
                pages = APP_MAX_SIZE / FLASH_PAGE;
            }
            Flash_ErasePages(APP_DL_ADDR, pages);
        }
        g_ota_len_expected = (uint16_t)len32;
        g_ota_rx_total = 0;
        g_dl_off = 0;
        g_ota_active = 1u;                 /* 进入下载会话：窗口据此中断等待 */
        SendAck(RSP_OK);                   /* 全擦(~450ms)在 ACK 前完成，PC 等 ACK 后才发数据 */
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
        if (n >= 4u && len32 == g_ota_len_expected && g_ota_rx_total == g_ota_len_expected)
        {
            SendAck(RSP_OK);               /* 本次升级写入验收通过 */
            Upd_OnDownloadComplete((uint32_t)g_ota_len_expected);  /* A→运行区搬运+跳转;失败则停留 */
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
 * START/END 应答 RSV=0。上位机靠 RSV 做逐帧确认/幂等重发。 */
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

    HAL_UART_Transmit(&huart1, buf, 13u, 100u);
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

/* 喂入字节流，驱动状态机；收满一帧后校验并分发 */
void Proto_Feed(const uint8_t *p, uint16_t len)
{
    uint16_t i;

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
