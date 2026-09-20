#include "flash_if.h"
#include "stm32f4xx_hal.h"

/* ==================== 扇区映射（F407ZGT6, 1MB 单 bank） ====================
 *   S0–S3  : 0x08000000 + n*16KB    （16KB each）
 *   S4     : 0x08010000             （64KB）
 *   S5–S11 : 0x08020000 + n*128KB   （128KB each）
 */

/* 地址 → 扇区号（内部使用） */
static uint8_t Flash_AddrToSector(uint32_t addr)
{
    if (addr < 0x08010000u)                                /* S0-S3: 16KB */
    {
        return (uint8_t)((addr - 0x08000000u) >> 14);      /* /16KB */
    }
    if (addr < 0x08020000u)                                /* S4: 64KB */
    {
        return 4u;
    }
    return (uint8_t)(5u + ((addr - 0x08020000u) >> 17));   /* S5-S11: 128KB */
}

/* 扇区号 → 扇区大小（对外公开，供上层估算耗时） */
uint32_t Flash_SectorSize(uint8_t sector)
{
    if (sector <= 3u)
    {
        return 0x4000u;                                    /* 16KB */
    }
    if (sector == 4u)
    {
        return 0x10000u;                                   /* 64KB */
    }
    return 0x20000u;                                       /* 128KB */
}

/**
 * @brief 擦除覆盖 [addr, addr+len) 的全部扇区
 * @note  起始地址不在扇区边界时，会从所在扇区的起始一并擦除（本工程各分区均按扇区对齐）
 * @retval 0=成功, 1=失败
 */
uint8_t Flash_EraseRegion(uint32_t addr, uint32_t len)
{
    FLASH_EraseInitTypeDef erase;
    uint32_t sector_error = 0u;
    HAL_StatusTypeDef st;
    uint8_t first;
    uint8_t last;

    if (len == 0u)
    {
        return 1u;
    }

    first = Flash_AddrToSector(addr);
    last  = Flash_AddrToSector(addr + len - 1u);   /* 末尾字节所在扇区 */

    erase.TypeErase    = FLASH_TYPEERASE_SECTORS;
    erase.Banks        = FLASH_BANK_1;             /* F407 单 bank；仅整片擦除时真正生效 */
    erase.Sector       = first;
    erase.NbSectors    = (uint32_t)(last - first + 1u);
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;    /* 2.7~3.6V：支持字/半字/字节/双字编程 */

    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        return 1u;
    }
    st = HAL_FLASHEx_Erase(&erase, &sector_error);
    HAL_FLASH_Lock();

    return (st == HAL_OK) ? 0u : 1u;
}

/**
 * @brief 写入数据到 Flash（字编程，一次 Unlock/Lock 批量写入）
 * @param addr 目标地址（必须 4 字节对齐）
 * @param buf  数据源
 * @param len  字节数，可为任意值（不足 4 字节的尾部补 0xFF 凑整字，
 *             邻字节本为擦除态 0xFF，无害）
 * @retval 0=成功, 1=失败
 *
 * @note F4 支持字/双字编程，比 F103 只能半字快一倍以上；
 *       实际 Caller（OTA 帧、标志记录）长度都是 4 的倍数，尾部补位仅作兜底。
 */
uint8_t Flash_ProgramBytes(uint32_t addr, const uint8_t *buf, uint32_t len)
{
    HAL_StatusTypeDef st = HAL_OK;
    uint32_t i;
    uint32_t word;
    uint32_t rem;
    uint8_t  tail[4];

    if (len == 0u)
    {
        return 0u;
    }
    if ((addr & 3u) != 0u)          /* 必须 4 字节对齐 */
    {
        return 1u;
    }

    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        return 1u;
    }

    /* ---- 主体：按 32 位字写入 ---- */
    for (i = 0u; i + 4u <= len; i += 4u)
    {
        word = (uint32_t)buf[i]
             | ((uint32_t)buf[i + 1u] << 8)
             | ((uint32_t)buf[i + 2u] << 16)
             | ((uint32_t)buf[i + 3u] << 24);

        st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + i, (uint64_t)word);
        if (st != HAL_OK)
        {
            break;
        }
    }

    /* ---- 尾部：不足 4 字节，补 0xFF 凑成整字 ---- */
    if ((st == HAL_OK) && ((len & 3u) != 0u))
    {
        rem = len & 3u;
        for (i = 0u; i < 4u; i++)
        {
            tail[i] = (i < rem) ? buf[len - rem + i] : 0xFFu;
        }
        word = (uint32_t)tail[0]
             | ((uint32_t)tail[1] << 8)
             | ((uint32_t)tail[2] << 16)
             | ((uint32_t)tail[3] << 24);

        st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + (len & ~3u), (uint64_t)word);
    }

    HAL_FLASH_Lock();

    return (st == HAL_OK) ? 0u : 1u;
}

/**
 * @brief 从 Flash 读取数据
 */
void Flash_ReadBuffer(uint32_t addr, uint8_t *buf, uint32_t len)
{
    const uint8_t *p = (const uint8_t *)addr;
    uint32_t i;

    for (i = 0u; i < len; i++)
    {
        buf[i] = p[i];
    }
}

/**
 * @brief Flash 层自检（擦 S11 → 擦后校验 0xFF → 写 10 字节 → 读回比对）
 * @note  会擦掉 FLASH_TEST_ADDR 所在扇区（预留区 S11，安全）
 * @retval 0=通过；2/3/4/5 = 失败的步骤号；1 = 擦除失败
 */
uint8_t Flash_SelfTest(void)
{
    static const uint8_t pat[10] = {
        0xA5u, 0x5Au, 0x01u, 0x02u, 0xFEu, 0xFFu, 0x00u, 0x80u, 0x7Fu, 0x11u
    };
    uint8_t  rd[16];
    uint32_t i;

    /* ① 擦除 */
    if (Flash_EraseRegion(FLASH_TEST_ADDR, (uint32_t)sizeof(pat)) != 0u)
    {
        return 1u;
    }

    /* ② 擦除后应为全 0xFF */
    Flash_ReadBuffer(FLASH_TEST_ADDR, rd, 16u);
    for (i = 0u; i < 16u; i++)
    {
        if (rd[i] != 0xFFu)
        {
            return 2u;
        }
    }

    /* ③ 写 10 字节（非 4 的倍数，验证尾部补位） */
    if (Flash_ProgramBytes(FLASH_TEST_ADDR, pat, (uint32_t)sizeof(pat)) != 0u)
    {
        return 3u;
    }

    /* ④ 读回比对主体 */
    Flash_ReadBuffer(FLASH_TEST_ADDR, rd, 16u);
    for (i = 0u; i < (uint32_t)sizeof(pat); i++)
    {
        if (rd[i] != pat[i])
        {
            return 4u;
        }
    }

    /* ⑤ 尾部补位字节应仍为 0xFF */
    for (i = (uint32_t)sizeof(pat); i < 12u; i++)
    {
        if (rd[i] != 0xFFu)
        {
            return 5u;
        }
    }

    return 0u;
}
