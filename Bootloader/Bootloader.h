#ifndef __BOOTLOADER_H
#define __BOOTLOADER_H

#include "usart.h"
#include "string.h"
#include "flash_if.h"
#include "protocol.h"


#define BOOTLOADER_UART_REC_BUFFER_SIZE 1536   /* 1.5KB 串口接收缓冲 */
#define BOOT_WAIT_MS                5000u   /* 上电等待窗口：窗口内收 OTA_START 进下载，否则启动决策 */

/* 已写入下载区 A 的字节数（由协议分发更新，供 OLED 显示） */
extern uint32_t g_dl_off;

void Bootloader_Init(void);
void Bootloader_Task(void);                                  /* 主循环消费：环形缓冲喂帧协议状态机 */
void Bootloader_Run(void);                                   /* 主流程：5s 窗口->启动决策/常驻，不返回(跳转或死循环) */

#endif
