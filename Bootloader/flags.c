#include "flags.h"
#include "flash_if.h"
#include <string.h>

/* CRC16-MODBUS 复用 protocol.c 的同名实现（同一 boot 工程链接） */
extern uint16_t CRC16_Modbus(const uint8_t *p, uint32_t len);

#define FLAG_REC_SIZE  24u   /* CRC 覆盖的前 6 个字段（magic..run_len） */

/* 读无效时使用的"全新"默认记录：无 B、无 run、seq 从 0 起 */
static void Flags_Default(s_flags *f)
{
    memset(f, 0, sizeof(*f));
    f->magic   = FLAGS_MAGIC;
    f->b_ver   = SEQ_NO_VER;
    f->run_ver = SEQ_NO_VER;
}

uint8_t Flags_Load(s_flags *f)
{
    uint32_t raw[7];
    uint32_t i;

    for (i = 0u; i < 7u; i++)
    {
        raw[i] = *(volatile uint32_t *)(FLAG_REC_ADDR + (i << 2));
    }
    if (raw[0] != FLAGS_MAGIC)
    {
        Flags_Default(f);
        return 0;                    /* 全 FF / 未知 → 未初始化 */
    }
    f->magic   = raw[0];
    f->seq     = raw[1];
    f->b_ver   = raw[2];
    f->b_len   = raw[3];
    f->run_ver = raw[4];
    f->run_len = raw[5];
    f->crc     = raw[6];
    if (f->crc != (uint32_t)CRC16_Modbus((const uint8_t *)f, FLAG_REC_SIZE))
    {
        Flags_Default(f);
        return 0;                    /* 撕裂写（断电半途）→ 按全无效 */
    }
    return 1;
}

uint8_t Flags_Save(const s_flags *f)
{
    s_flags out;
    memcpy(&out, f, sizeof(out));
    out.crc = (uint32_t)CRC16_Modbus((const uint8_t *)&out, FLAG_REC_SIZE);

    if (Flash_ErasePages(FLAG_REC_ADDR, 1u) != 0u)
    {
        return 1;
    }
    /* s_flags 28B，偶数长且地址对齐，满足半字编程约束 */
    if (Flash_ProgramBytes(FLAG_REC_ADDR, (const uint8_t *)&out, sizeof(out)) != 0u)
    {
        return 1;
    }
    return 0;
}

uint8_t Health_Read(void)
{
    return (*(volatile uint32_t *)FLAG_HEALTH_ADDR == HEALTH_OK) ? 1u : 0u;
}

uint8_t Health_Clear(void)
{
    return Flash_ErasePages(FLAG_HEALTH_ADDR, 1u);   /* 0=成功 */
}
