#include "flags.h"
#include "flash_if.h"
#include <string.h>

/* CRC16-MODBUS 复用 protocol.c 的同名实现（同一 boot 工程链接） */
extern uint16_t CRC16_Modbus(const uint8_t *p, uint32_t len);

/* 读无效时使用的"全新"默认记录：无 B、无 run、seq 从 0 起 */
static void Flags_Default(s_flags *f)
{
    memset(f, 0, sizeof(*f));
    f->magic   = FLAGS_MAGIC;
    f->b_ver   = SEQ_NO_VER;
    f->run_ver = SEQ_NO_VER;
}

uint8_t Health_Read(void)
{
    return (*(volatile uint32_t *)FLAG_HEALTH_ADDR == HEALTH_OK) ? 1u : 0u;
}

uint8_t Flags_Load(s_flags *f)
{
    uint32_t raw[8];
    uint32_t h;
    uint32_t i;

    for (i = 0u; i < 8u; i++)
    {
        raw[i] = *(volatile uint32_t *)(FLAG_REC_ADDR + (i << 2));
    }
    h = raw[6];                        /* health 独立于记录有效性，先留住 */

    f->magic   = raw[0];
    f->seq     = raw[1];
    f->b_ver   = raw[2];
    f->b_len   = raw[3];
    f->run_ver = raw[4];
    f->run_len = raw[5];
    f->health  = h;
    f->crc     = raw[7];

    if (raw[0] != FLAGS_MAGIC)
    {
        Flags_Default(f);              /* 全 FF / 未知 → 未初始化 */
        f->health = h;
        return 0u;
    }
    /* CRC 覆盖 magic..run_len（偏移 0..23 共 24B，不含 health）。结构体无填充，可整块入 CRC。 */
    if (f->crc != (uint32_t)CRC16_Modbus((const uint8_t *)f, FLAG_REC_SIZE))
    {
        Flags_Default(f);              /* 撕裂写（断电半途）→ 按全无效 */
        f->health = h;
        return 0u;
    }
    return 1u;
}

uint8_t Flags_Save(const s_flags *f)
{
    s_flags out;

    memcpy(&out, f, sizeof(out));
    out.health = HEALTH_NONE;   /* ★ 记录一变，旧的健康确认即失效（核心不变量） */
    out.crc = (uint32_t)CRC16_Modbus((const uint8_t *)&out, FLAG_REC_SIZE);

    /* 擦整个 S5 扇区（128KB）。传入长度只是圈定范围，F4 的实际擦除粒度是整扇区。
     * 擦除期间 Flash 总线 stall、CPU 冻结，典型 1s 量级 —— 仅在上电决策/下载完成时发生，
     * 常态快速启动路径不写记录，因此不影响日常启动速度。
     * ★ 该扇区必须完全属于标志区：boot 代码实测铺到 0x08013493，若标志区还在
     *   S4(0x08010000)，这次擦除会把 boot 自己 13.2KB 的代码一并擦掉。 */
    if (Flash_EraseRegion(FLAG_REC_ADDR, (uint32_t)sizeof(out)) != 0u)
    {
        return 1u;
    }
    if (Flash_ProgramBytes(FLAG_REC_ADDR, (const uint8_t *)&out, (uint32_t)sizeof(out)) != 0u)
    {
        return 1u;
    }
    return 0u;
}
