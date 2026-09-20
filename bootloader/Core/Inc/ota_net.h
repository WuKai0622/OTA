#ifndef __OTA_NET_H
#define __OTA_NET_H

#include <stdint.h>

#include "ota_chan.h"    /* OTA_RxSink_t —— 三条通道共用的接收汇点类型，见那里 */

/* ==================== OTA 以太网通道（TCP 服务器） ====================
 * 角色：设备做 TCP 服务器，由上位机主动连接。
 *       端口见 OTA_TCP_PORT，本机 IP 在 lwip.c 的 MX_LWIP_Init() 里配置。
 *
 * 职责只有两件事：
 *   ① 接收 —— 把网口收到的字节经"接收汇点"交给上层（boot 侧即写入 8KB 环，
 *              `Bootloader.c` 的 `RING_SIZE = 8192u`）
 *   ② 发送 —— 提供 OTA_Net_Send()，通过 Proto_SetTx() 注册给协议层
 *
 * 本文件不依赖 lwIP 类型，故可脱离 boot 单独编译验证。
 */

/* OTA 服务端口（上位机 GUI 里填这个） */
#define OTA_TCP_PORT        5001

/* 接收汇点类型 OTA_RxSink_t 现在住在 ota_chan.h（三条通道共用一份定义）。
 *
 * 返回值的意义（重要）：只把"真正被接收"的字节回报给 TCP 做 tcp_recved，
 * 环满时窗口自然不放大 → 对端自动降速。这是 TCP 通道的背压机制，
 * 也是它不能像串口那样"静默丢弃"的原因（TCP 认为已送达，不会重发）。
 *
 * boot 侧的实现是 Bootloader.c 里的 EthSink（补上 OTA_CH_ETH 后写进环形缓冲）。 */

/* 初始化并起监听。必须在 MX_LWIP_Init() 之后调用（裸机下即 main 里）。
 * 之后再周期调用 MX_LWIP_Process() 才会真正收包。 */
void    OTA_Net_Init(OTA_RxSink_t sink);

/* 发送出口：注册给协议层（Proto_SetTx(OTA_Net_Send)）。
 * 返回 0 = 已交给 lwIP 发送队列；1 = 无连接/窗口不足/入队失败。 */
uint8_t OTA_Net_Send(const uint8_t *buf, uint16_t len);

/* 是否有上位机连接（LCD 显示 / 通道选择用） */
uint8_t OTA_Net_Connected(void);

/* 关闭监听与当前连接，并停掉 ETH 的 MAC/DMA。跳转 app 之前必须调用。
 *
 * ★ 为什么必须停 DMA：ETH 的 DMA 是总线主设备，只要描述符环还在，
 *   它就会继续把收到的帧写进 boot 的 RAM 缓冲。而 app 会重新初始化 ETH、
 *   重设自己的描述符环 —— 两套环指向同一片内存，故障是随机的、极难定位。
 *   在 __disable_irq() 之前调用（它内部要动 lwIP 和 ETH 寄存器）。 */
void    OTA_Net_DeInit(void);

#endif /* __OTA_NET_H */
