#ifndef __WIFI_LINK_H
#define __WIFI_LINK_H

#include <stdint.h>

/* ==================== WiFi 连接（AP / STA 二选一） ====================
 * 本模块只负责"把网连上"并对外提供拿到的 IP，不涉及 SPI、不涉及 OTA。
 *
 * ★ 为什么两种模式都做：考试现场不一定有可用路由器。
 *   AP 模式下 ESP 自己开热点、IP 固定 192.168.4.1，PC 连上即可演示，
 *   完全不依赖外部网络；有路由器时把 WIFI_USE_AP 改成 0 走 STA。
 *
 * 改 WIFI_USE_AP 后必须重新编译烧录（另一个分支的代码不会进固件）。
 */

#define WIFI_USE_AP   1              /* 1 = ESP 开热点（默认，考场最稳）; 0 = 连现有路由器 */

#if WIFI_USE_AP
  #define WIFI_AP_SSID      "OTA_ESP32"
  #define WIFI_AP_PASS      "12345678"   /* 至少 8 位走 WPA2；不足 8 位按开放网络处理 */
  #define WIFI_AP_CHANNEL   1
  #define WIFI_AP_MAX_CONN  2            /* 够用即可：PC 一个，留一个余量 */
#else
  #define WIFI_STA_SSID     "YOUR_ROUTER_SSID"      /* ★ 改成现场可用的 WiFi */
  #define WIFI_STA_PASS     "YOUR_ROUTER_PASSWORD"
#endif

/* 初始化并启动 WiFi。必须在 app_main 里最先调用（内部会初始化 esp_netif，
 * 之后才能建 socket）。不阻塞：连上/拿到 IP 由事件回调异步完成。 */
void Wifi_Link_Init(void);

/* 是否已就绪：AP 模式 = 热点已启动；STA 模式 = 已拿到 IP。 */
uint8_t Wifi_Link_Ready(void);

/* 当前 IP 字符串，未就绪时为 "0.0.0.0"。用于打印提示连哪个地址。 */
const char *Wifi_Link_IpStr(void);

#endif /* __WIFI_LINK_H */
