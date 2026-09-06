#ifndef __FLASH_IF_H
#define __FLASH_IF_H

#include <stdint.h>

/* ==================== Flash 5 分区（1KB 页对齐） ==================== */
#define BL_BASE         0x08000000u  /* Bootloader 区  16KB, 页0-15   */
#define APP_RUN_ADDR    0x08004000u  /* APP 运行区     15KB, 页16-30  */
#define APP_DL_ADDR     0x08007C00u  /* 下载区 a       15KB, 页31-45  */
#define APP_BK_ADDR     0x0800B800u  /* 备份区 b       15KB, 页46-60  */
#define PARAM_ADDR      0x0800F400u  /* 参数区          3KB, 页61-63  */

#define APP_MAX_SIZE    15360u       /* 固件上限 15KB */
#define FLASH_PAGE      1024u        /* 页大小 1KB */

/* Flash 操作接口（返回 0=成功, 1=失败） */
uint8_t Flash_ErasePages(uint32_t addr, uint32_t nb);            /* 页擦除 */
uint8_t Flash_ProgramBytes(uint32_t addr, const uint8_t *buf, uint32_t len); /* 半字编程，len 可为奇数 */
void    Flash_ReadBuffer(uint32_t addr, uint8_t *buf, uint32_t len);          /* 读取，供验证 */

#endif
