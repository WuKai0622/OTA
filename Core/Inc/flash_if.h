#ifndef __FLASH_IF_H
#define __FLASH_IF_H

#include <stdint.h>

/* ==================== Flash 5 分区（F407ZGT6, 1MB 单 bank） ====================
 *
 * ★ 与 F103 的关键差异：F4 擦除单位是"扇区"，且各扇区大小不等 ——
 *     S0–S3 = 16KB | S4 = 64KB | S5–S11 = 128KB
 *   F103 的"均匀 1KB 页"概念在这里不存在，所有"按页算数"的旧代码都要改。
 *
 * ★★ boot 必须独占 S0–S4 共 128KB，不能只占 64KB —— 这是实测踩出来的：
 *   本工程 lwIP 裸机 + LCD 字库下，boot 镜像实测 77.18KB，代码铺到 0x08013493。
 *   若按 64KB 划分、把标志区放在 S4(0x08010000)，则第一次 Flags_Save() 擦 S4 时
 *   会连带把 boot 自己在 0x08010000–0x08013493 的 13.2KB 代码一并擦成 0xFF
 *   （被波及的有 lcd.o 7220B、protocol.o 512B、flash_if.o 及 tcp/tcp_in/tcp_out
 *     /etharp/timeouts 若干）。
 *   症状：上位机报"升级成功"，随后卡在 LCD 的 DL:（HardFault，画面冻结在最后一帧），
 *         断电重启黑屏（main 一进去就调 LCD_Init，而它已被擦掉）。
 *   之所以上位机会看到"成功"：OTA_END 的 ACK 在 Upd_OnDownloadComplete() 之前发出，
 *   自毁发生在其内部的 Flags_Save() 里，此时应答早已送达。
 *   ⇒ 链接脚本必须同步限制 IROM1 Size = 0x20000，否则覆盖检查形同虚设。
 *
 *   0x08000000 ┌──────────┐
 *              │  boot    │ 128KB  S0–S4
 *   0x08020000 ├──────────┤
 *              │  标志区   │ 128KB  S5   （状态记录 + 健康字，同扇区）
 *   0x08040000 ├──────────┤
 *              │  run     │ 128KB  S6
 *   0x08060000 ├──────────┤
 *              │  A 下载  │ 128KB  S7
 *   0x08080000 ├──────────┤
 *              │  B 备份  │ 128KB  S8
 *   0x080A0000 ├──────────┤
 *              │  预留    │ 384KB  S9–S11
 *   0x08100000 └──────────┘
 *
 * ★ 每个分区各占整数个扇区，且互不重叠 —— 这是"擦一个区不会波及邻居"的前提。
 * ★ 改任何一个分区地址，都必须同步 app 工程的 IROM1 起始地址（见 APP_RUN_ADDR）。
 */
#define BL_BASE         0x08000000u  /* Bootloader 区  128KB, S0-S4 */
#define PARAM_ADDR      0x08020000u  /* 标志区         128KB, S5    */
#define APP_RUN_ADDR    0x08040000u  /* APP 运行区     128KB, S6    */
#define APP_DL_ADDR     0x08060000u  /* 下载区 a       128KB, S7    */
#define APP_BK_ADDR     0x08080000u  /* 备份区 b       128KB, S8    */

#define APP_MAX_SIZE    (128u * 1024u)   /* 固件上限 128KB（= A/B/run 区大小） */

/* 自检用扇区：S11（预留区末尾），可安全擦写 */
#define FLASH_TEST_ADDR 0x080E0000u
#define FLASH_TEST_SIZE 0x20000u         /* 128KB */

/* Flash 操作接口（返回 0=成功, 1=失败） */
uint8_t Flash_EraseRegion(uint32_t addr, uint32_t len);                     /* 擦除覆盖 [addr,addr+len) 的扇区 */
uint8_t Flash_ProgramBytes(uint32_t addr, const uint8_t *buf, uint32_t len); /* 字编程，len 可为任意长度 */
void    Flash_ReadBuffer(uint32_t addr, uint8_t *buf, uint32_t len);         /* 读取，供校验 */

/* 扇区信息（供上层估算擦除范围 / 耗时） */
uint32_t Flash_SectorSize(uint8_t sector);

/* 自检：擦 S11 → 写 10 字节(非 4 倍数) → 读回比对。返回 0=通过，非 0=失败步骤号 */
uint8_t Flash_SelfTest(void);

#endif /* __FLASH_IF_H */
