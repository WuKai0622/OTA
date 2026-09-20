#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#include <stdint.h>

#include "ota_chan.h"    /* OtaChan_t —— 本层要知道"这一帧从哪条通道来"，见下方会话绑定 */

/* ==================== 帧格式（考试自定义，参考 modbus） ====================
 * | HEAD(2) | CMD(2) | LEN(2) | DATA N | RSV(4) | CRC16(2) |
 *   0xA5 0x5A   LE      LE=N      N<=1024    LE       LE
 * CRC16-MODBUS 覆盖 [CMD..RSV] 共 8+LEN 字节（不含包头、不含 CRC），init=0xFFFF。
 * 帧总长 = 12 + N（含包头）。
 *
 * ★ 本层与物理通道的解耦：
 *     收 —— 上层把字节流喂进 Proto_Feed()，同时告诉它**这段字节来自哪条通道**；
 *     发 —— 上层用 Proto_SetTx() 注入发送函数，该函数按传入的通道号选路，本层不碰外设。
 *   于是三条通道（UART/ETH/SPI）共用同一个状态机、同一套命令处理。
 *
 * ★★ 会话绑定（步骤④ 新增，也是本层唯一的架构性改动）：
 *   在加 SPI 通道之前，发送出口是"**发送时刻**看 TCP 通不通，通则走网口、否则走串口"。
 *   三条通道都通之后这个判据就错了：PC 完全可能挂着一条**闲置的 ETH 连接**，
 *   而升级走 WiFi(SPI) —— 于是 F407 把 ACK 发去了 ETH，PC 在 SPI 那一侧永远等不到，
 *   超时重发、状态机错乱。故障是偶发的（取决于 PC 有没有留着旧连接），极难定位。
 *   改法：**在收到合法 OTA_START 的那一刻把通道锁存下来**，本次会话的所有应答
 *   都回这条通道，与"发送时刻哪条通道连着"彻底无关。
 *   锁存点选在 START 而不是 DATA，是因为 START 那一帧的 ACK 也必须回对通道 ——
 *   否则 PC 第一步就收不到应答，会话根本建立不起来。
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

/* 发送回调：**按通道选路**，返回 0=成功, 1=失败。
 * ch 由本层算好（会话已锁存就用锁存值，否则用最后一次收到数据的通道），
 * 出口只管照着发 —— 通道选择这个决策只应该有一处，放在出口里就会变成三处。
 *
 * 失败时协议层不重发：
 *   TCP 上"失败"是真实可能的（无连接 / 发送窗口不足）；
 *   SPI 上"失败"= 发送缓冲已满，同样不重发。
 * 重发一律由上位机的 stop-and-wait 超时机制负责，两端职责不重叠。 */
typedef uint8_t (*Proto_Tx_t)(OtaChan_t ch, const uint8_t *buf, uint16_t len);

/* ==================== 接口 ==================== */
uint16_t CRC16_Modbus(const uint8_t *p, uint32_t len);   /* poly 0xA001, init 0xFFFF, 查表 */
void     Proto_Init(void);                               /* 复位状态机（新升级会话前调用） */
void     Proto_SetTx(Proto_Tx_t tx);                     /* 注入发送出口（启动时调用一次） */

/* 喂入字节流，内部收整帧->分发。ch = 这段字节来自哪条通道。
 * ★ ch 由**生产者**决定并一路带上来（Bootloader_RingWrite 的首参），
 *   不是本层猜的 —— 猜出来的通道一旦猜错，ACK 就发去了别的通道。 */
void     Proto_Feed(const uint8_t *p, uint16_t len, OtaChan_t ch);
uint8_t  Proto_SessionStarted(void);                     /* 1=已收到合法 OTA_START，进入下载会话 */

#endif /* __PROTOCOL_H */
