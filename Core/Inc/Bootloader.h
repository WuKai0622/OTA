#ifndef __BOOTLOADER_H
#define __BOOTLOADER_H

#include <stdint.h>

#define BOOTLOADER_UART_REC_BUFFER_SIZE 1536u   /* 1.5KB 串口接收缓冲 */
#define BOOT_WAIT_MS                    5000u   /* 上电等待窗口：窗口内收 OTA_START 进下载，否则启动决策 */

/* 已写入下载区 A 的字节数（由协议分发更新，供 LCD 显示） */
extern uint32_t g_dl_off;

void Bootloader_Init(void);

/* 主循环消费：环形缓冲喂帧协议状态机 */
void Bootloader_Task(void);

/* 主流程：5s 窗口 -> 启动决策/常驻，不返回(跳转或死循环) */
void Bootloader_Run(void);

/* 环形缓冲写入（生产者入口）。
 * 供 UART 空闲中断 与 ETH 接收汇点 共用；返回实际写入字节数 ——
 * TCP 侧据此调用 tcp_recved() 做窗口背压，串口侧忽略返回值（丢帧靠 stop-and-wait 重发）。 */
uint16_t Bootloader_RingWrite(const uint8_t *p, uint16_t len);

#endif /* __BOOTLOADER_H */
