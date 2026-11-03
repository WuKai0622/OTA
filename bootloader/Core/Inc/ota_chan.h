#ifndef __OTA_CHAN_H
#define __OTA_CHAN_H

#include <stdint.h>

/* ==================== 通道身份 + 接收汇点（三条通道共用的最小词汇） ====================
 * 这个头文件存在的唯一理由：**打破一个循环依赖**。
 *   帧协议层（protocol.h）需要知道"这一帧是从哪条通道来的"—— 会话绑定要靠它；
 *   三条通道驱动（uart/eth/spi）需要同一个"接收汇点"类型 —— 它们的注入签名必须一样。
 *   把这两样放在这里，protocol.h / ota_net.h / ota_spi.h / Bootloader.h 就都能用，
 *   谁也不用去包含别的驱动的头文件（否则 ota_spi.h 为了拿一个 typedef 得去包含
 *   ota_net.h，看着就荒唐，而且会诱导出真正的循环包含）。
 */

/* 通道身份。
 * ★ OTA_CH_NONE 必须显式 = 0：它是"没锁存"的哨兵值，靠 memset/静态零初始化成立。
 *   顺序（UART/ETH/SPI）本身无意义，别依赖它做比较或排序。 */
typedef enum
{
    OTA_CH_NONE = 0,        /* 未知/未绑定：会话尚未开始 */
    OTA_CH_UART,            /* USART3（USB-TTL 串口） */
    OTA_CH_ETH,             /* TCP 服务器（lwIP） */
    OTA_CH_SPI              /* SPI 从机 ←→ ESP32-C3 ←→ WiFi/TCP */
} OtaChan_t;

/* 接收汇点：通道驱动收到的每段数据交给它处理。**三条通道同型**。
 *   参数 data/len —— 本段数据
 *   返回          —— 实际被接收的字节数（≤ len）
 *
 * ★ 返回值的意义（重要）：只有 TCP 那条通道用得上 —— 它把"真正被接收"的字节数
 *   回报给 lwIP 做 tcp_recved，环满时窗口自然不放大，对端自动降速。
 *   UART 与 SPI 没有背压手段（字节已经被时钟/中断打进来了，无法拒收），
 *   它们的 sink 忽略返回值即可，丢掉的靠上位机 stop-and-wait 超时重发兜底。
 *
 * 实现见 Bootloader.c 的三个 static wrapper（每一条通道一个），
 * 它们的作用只是把 OTA_CH_xxx 这个常量补上去 —— 生产者**不可能忘**。 */
typedef uint16_t (*OTA_RxSink_t)(const uint8_t *data, uint16_t len);

#endif /* __OTA_CHAN_H */
