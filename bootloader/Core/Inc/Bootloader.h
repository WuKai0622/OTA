#ifndef __BOOTLOADER_H
#define __BOOTLOADER_H

#include <stdint.h>

#include "ota_chan.h"    /* OtaChan_t —— 环的写入必须声明来源通道 */

#define BOOTLOADER_UART_REC_BUFFER_SIZE 1536u   /* 1.5KB 串口接收缓冲 */
#define BOOT_WAIT_MS                    5000u   /* 上电等待窗口：窗口内收 OTA_START 进下载，否则启动决策 */

/* 下载会话空闲上限(ms)：常驻下载循环里超过这么久**没有任何字节到达**，
 * 判定上位机已经放弃（拔线 / 关窗口 / 上位机死机 / 从蓝牙换到 WiFi），
 * 拆除会话并尝试回到运行区 APP —— 否则设备会永远停在 "DL:" 等一个不会来的
 * 数据帧，只能靠复位救回来（与长期无人值守的目标直接冲突）。
 *
 * ★ 取值依据：正常会话里最长的**合法**静默是 stop-and-wait 的"重发前静默"，
 *   上位机侧 DATA_TIMEOUT(3.0s) + RETRY_QUIET(3.0s) ≈ 6s，而每次重发都会往环里
 *   补字节、自动续上计时。START 的擦除(最坏 ~4s)与下载完成的搬运都发生在
 *   **同一次** Bootloader_Task() 调用内，字节早已到齐。30s 有约 5 倍余量。
 *   下限不要低于 ~10s，否则链路上的正常抖动会被误判成掉线。 */
#define BOOT_DL_IDLE_MS                 30000u

/* 已写入下载区 A 的字节数（由协议分发更新，供 LCD 显示） */
extern uint32_t g_dl_off;

void Bootloader_Init(void);

/* 主循环消费：环形缓冲喂帧协议状态机 */
void Bootloader_Task(void);

/* 主流程：5s 窗口 -> 启动决策/常驻，不返回(跳转或死循环) */
void Bootloader_Run(void);

/* 环形缓冲写入（生产者入口）。
 * 供 UART 空闲中断、ETH 接收汇点、SPI 事务循环三处共用。
 *
 * ch 是**必填的首参**，不是可选项 —— 这是刻意的：通道身份要一路带到协议层去做
 * 会话绑定（见 protocol.h），而"生产者忘了声明来源"是那种编译期抓不到、
 * 运行期表现为"ACK 发去了别的通道"的偶发故障。放在第一个位置上，忘不掉。
 *
 * 返回实际写入字节数 —— TCP 侧据此调用 tcp_recved() 做窗口背压；
 * 串口与 SPI 侧忽略返回值（这两条通道没有背压手段，丢帧靠 stop-and-wait 重发）。 */
uint16_t Bootloader_RingWrite(OtaChan_t ch, const uint8_t *p, uint16_t len);

#endif /* __BOOTLOADER_H */
