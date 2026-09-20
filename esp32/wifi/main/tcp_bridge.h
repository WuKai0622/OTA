#ifndef __TCP_BRIDGE_H
#define __TCP_BRIDGE_H

#include <stdint.h>

/* ==================== 步骤③：SPI 从机 ↔ TCP 桥 ====================
 * 本模块管 TCP 侧：做**服务器**（端口 5001）、单连接模型、两个方向的搬运。
 * SPI 侧在 spi_bridge.c。
 *
 * ★ 为什么做服务器而不是客户端：上位机零改动，且与 ETH 通道**同构**
 *   （boot 侧的 ota_net.c 也是做服务器）。GUI 的 wifi 选项卡、ota_host.py
 *   的 --tcp 直接就能用，不必为 WiFi 通道另写一套上位机。
 */

#define TCP_BRIDGE_PORT   5001

/* 起 TCP 服务器任务。必须在 Spi_Bridge_Start() 之后调用（要用它建的环）。 */
void    Tcp_Bridge_Start(void);

/* 当前是否有 PC 连着。SPI 侧把它作为 STATUS 的 bit1 上报给 F407 ——
 * 这样 LCD 上就能直接区分"WiFi 断了"还是"PC 没连"。
 * 会从 SPI 任务（另一个任务）里调用，所以内部用 volatile。 */
uint8_t Tcp_Bridge_HasClient(void);

#endif /* __TCP_BRIDGE_H */
