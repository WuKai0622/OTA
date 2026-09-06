#include "flash_if.h"
#include "stm32f1xx_hal.h"

/**
 * @brief 页擦除
 * @param addr 目标页起始地址（须页对齐）
 * @param nb   要擦除的页数
 * @retval 0=成功, 1=失败
 */
uint8_t Flash_ErasePages(uint32_t addr, uint32_t nb)
{
    FLASH_EraseInitTypeDef erase;
    uint32_t page_error = 0;
    HAL_StatusTypeDef st;

    erase.TypeErase   = FLASH_TYPEERASE_PAGES;
    erase.Banks       = FLASH_BANK_1;
    erase.PageAddress = addr;
    erase.NbPages     = nb;

    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        return 1;
    }
    st = HAL_FLASHEx_Erase(&erase, &page_error);
    HAL_FLASH_Lock();

    if(st == HAL_OK)
    {
        return 0;
    }
    else
    {
        return 1;
    }
}

/**
 * @brief 写入数据到 Flash（半字编程，一次 Unlock/Lock 批量写入）
 * @param addr 目标地址（必须 2 字节对齐）
 * @param buf  数据源
 * @param len  字节数，可为奇数（末尾字节补 0xFF 半字，邻字节本为擦除态无害）
 * @retval 0=成功, 1=失败
 */
uint8_t Flash_ProgramBytes(uint32_t addr, const uint8_t *buf, uint32_t len)
{
    uint32_t i;
    HAL_StatusTypeDef st = HAL_OK;

    /* 目标地址必须半字（2 字节）对齐 */
    if ((addr & 1u) != 0u)
    {
        return 1;
    }

    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        return 1;
    }

    for (i = 0; i + 1 < len; i += 2)
    {
        uint16_t hw = (uint16_t)(buf[i] | ((uint16_t)buf[i + 1] << 8));
        st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr + i, hw);
        if (st != HAL_OK)
        {
            break;
        }
    }

    /* 奇数长度：最后 1 字节与补位 0xFF 组成半字写入 */
    if ((len & 1u) != 0u)
    {
        uint16_t hw = (uint16_t)(buf[len - 1] | 0xFF00u);
        st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr + len - 1, hw);
    }

    HAL_FLASH_Lock();
    return (st == HAL_OK) ? 0 : 1;
}

/**
 * @brief 从 Flash 读取数据
 * @param addr 源地址
 * @param buf  目标缓冲区
 * @param len  字节数
 */
void Flash_ReadBuffer(uint32_t addr, uint8_t *buf, uint32_t len)
{
    const uint8_t *p = (const uint8_t *)addr;
    uint32_t i;

    for (i = 0; i < len; i++)
    {
        buf[i] = p[i];
    }
}
