#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#include <stdint.h>

/* ==================== 帧格式（考试自定义，参考 modbus） ====================
 * | HEAD(2) | CMD(2) | LEN(2) | DATA N | RSV(4) | CRC16(2) |
 *   0xA5 0x5A   LE      LE=N      N<=1024    LE       LE
 * CRC16-MODBUS 覆盖 [CMD..RSV] 共 8+LEN 字节（不含包头、不含 CRC），init=0xFFFF。
 * 帧总长 = 12 + N（含包头）。
 *
 * ★ 本层与物理通道完全解耦：
 *     收 —— 上层把字节流喂进 Proto_Feed()，不关心它来自 UART 还是 TCP；
 *     发 —— 上层用 Proto_SetTx() 注入发送函数，本层不直接碰任何外设。
 *   于是 UART 与 ETH 两个通道共用同一个状态机、同一套命令处理。
 */
#define PKT_HEAD0          0xA5u
#define PKT_HEAD1          0x5Au
#define FRAME_DATA_MAX     1024u       /* 数据区上限 */
#define FRAME_BUF_SIZE     (12u + FRAME_DATA_MAX)

/* ==================== 命令码 ==================== */
#define CMD_OTA_START      0x0001u     /* PC->boot：数据=固件总长u32, RSV=目标区 0=A */
#define CMD_OTA_DATA       0x0002u     /* PC->boot：数据=固件块,     RSV=本帧起始偏移u32 */
#define CMD_OTA_END        0x0003u     /* PC->boot：数据=固件总长u32 */
#define CMD_RSP_ACK        0x0100u     /* boot->PC：数据=状态码1B */

#define RSP_OK             0u
#define RSP_ERR            1u

/* 发送回调：返回 0=成功, 1=失败。
 * TCP 上"失败"是真实可能的（无连接 / 发送窗口不足），失败时协议层不重发 ——
 * 重发由上位机的 stop-and-wait 超时机制负责，两端职责不重叠。 */
typedef uint8_t (*Proto_Tx_t)(const uint8_t *buf, uint16_t len);

/* ==================== 接口 ==================== */
uint16_t CRC16_Modbus(const uint8_t *p, uint32_t len);   /* poly 0xA001, init 0xFFFF, 查表 */
void     Proto_Init(void);                               /* 复位状态机（新升级会话前调用） */
void     Proto_SetTx(Proto_Tx_t tx);                     /* 注入发送出口（启动时调用一次） */
void     Proto_Feed(const uint8_t *p, uint16_t len);     /* 喂入字节流，内部收整帧->分发 */
uint8_t  Proto_SessionStarted(void);                     /* 1=已收到合法 OTA_START，进入下载会话 */

#endif /* __PROTOCOL_H */
