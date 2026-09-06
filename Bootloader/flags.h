#ifndef __FLAGS_H
#define __FLAGS_H

#include <stdint.h>

/* ==================== 标志区布局（参数区 3KB = 页 61~63，boot 独写） ====================
 *   页61 0x0800F400  状态记录 s_flags（整页擦 + 整写，28B）
 *   页62 0x0800F800  健康标志单字：0xA5A5A5A5=候选已被确认健康（app 写，boot 整页擦清除）
 */
#define FLAG_REC_ADDR       0x0800F400u
#define FLAG_HEALTH_ADDR    0x0800F800u
#define HEALTH_OK           0xA5A5A5A5u

#define FLAGS_MAGIC         0x5AA5F00Du    /* 记录有效标记（否则视为全 FF 未初始化） */
#define SEQ_NO_VER          0xFFFFFFFFu    /* 无有效固件（擦除态） */
#define SEQ_B_WIP           0xFFFFFFFEu    /* B 正在被改写，断电后不可信 */

typedef struct {
    uint32_t magic;    /* FLAGS_MAGIC = 记录有效 */
    uint32_t seq;      /* 单调版本序：每放一个候选 +1 */
    uint32_t b_ver;    /* B 内容对应 seq；NO_VER=无有效B，B_WIP=B写中 */
    uint32_t b_len;    /* B 固件长度（供拷贝） */
    uint32_t run_ver;  /* run 内容对应 seq；NO_VER=无 */
    uint32_t run_len;  /* run 固件长度 */
    uint32_t crc;      /* CRC16-MODBUS 覆盖前 6 字段(24B)，兜底撕裂写 */
} s_flags;             /* 28B，无填充 */

/* 读状态记录到 *f。记录损坏（magic/CRC 不符）按"全无效"处理：
 * 填默认 {seq=0, b_ver=NO_VER, b_len=0, run_ver=NO_VER, run_len=0}，返回 0。
 * 返回 1 = 读到一份可信记录。 */
uint8_t Flags_Load(s_flags *f);

/* 写状态记录：按 *f 内容算 CRC 后擦页61 + 整写。返回 0=成功,1=失败。 */
uint8_t Flags_Save(const s_flags *f);

/* 健康标志。Health_Read：1=已写 HEALTH_OK。Health_Clear：整页擦页62。 */
uint8_t Health_Read(void);
uint8_t Health_Clear(void);

#endif /* __FLAGS_H */
