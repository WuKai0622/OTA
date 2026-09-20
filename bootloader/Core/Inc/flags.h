#ifndef __FLAGS_H
#define __FLAGS_H

#include <stdint.h>
#include <stddef.h>

/* ==================== 标志区（S5 = 0x08020000，128KB，独占扇区） ====================
 *
 * ★ 与 F103 版本的关键差异：健康字不再是独立页（F103 是页61记录 + 页62健康字），
 *   而是并进同一条记录，成为第 7 个字段（偏移 24）。
 *   原因：F4 最小擦除单位是扇区，这里一个扇区就是 128KB —— 为写一个健康字单独
 *         占一个扇区代价太大；合并后 app 只需往固定偏移编程一个字，无需擦除。
 *
 * ★ 为什么标志区不在 0x08010000：boot 镜像实测 77.18KB，代码一直铺到 0x08013493，
 *   0x08010000 起的那 13.2KB 装的全是 boot 自己的代码（lcd.o / protocol.o /
 *   flash_if.o 及 lwip 若干）。标志区放在那里，第一次 Flags_Save() 擦除 S4 时
 *   就会把 boot 自己擦掉 —— 症状是"上位机报升级成功，随后卡在 DL:，重启黑屏"。
 *   S4 必须留给 boot，标志区挪到 S5。
 *
 *   0x08020000 ┌──────────────────────────┐
 *              │  s_flags (32B)           │  ← 整扇区擦 + 整写
 *   0x08020018 │    ...health 字段        │  ← app 只编程这一个字（1→0，不需擦）
 *   0x0802001C ├──────────────────────────┤
 *              │  （扇区剩余 131040B 未用）│
 *   0x08040000 └──────────────────────────┘
 *
 * ★ 不变量：任何一次 Flags_Save() 都会把 health 强制清回"未确认"。
 *   于是"记录一变 ⇒ 旧的健康确认自动失效"由机制保证，不依赖调用方记得去清 ——
 *   F103 版本里散落在各跳转分支前的 Health_Clear() 调用因此全部删除。
 */
#define FLAG_REC_ADDR       0x08020000u   /* = PARAM_ADDR，S5 首址 */

#define FLAGS_MAGIC         0x5AA5F00Du   /* 记录有效标记（否则视为全 FF 未初始化） */
#define SEQ_NO_VER          0xFFFFFFFFu   /* 无有效固件（擦除态） */
#define SEQ_B_WIP           0xFFFFFFFEu   /* B 正在被改写，断电后不可信 */

#define HEALTH_OK           0xA5A5A5A5u   /* run 中的固件已被 app 确认"跑起来了" */
#define HEALTH_NONE         0xFFFFFFFFu   /* 未确认（= 擦除态） */

typedef struct {
    uint32_t magic;    /*  0: FLAGS_MAGIC = 记录有效 */
    uint32_t seq;      /*  4: 单调版本序：每放一个候选 +1 */
    uint32_t b_ver;    /*  8: B 内容对应 seq；NO_VER=无有效B，WIP=B写中 */
    uint32_t b_len;    /* 12: B 固件长度（供拷贝） */
    uint32_t run_ver;  /* 16: run 内容对应 seq；NO_VER=无 */
    uint32_t run_len;  /* 20: run 固件长度 */
    uint32_t health;   /* 24: ★ HEALTH_OK=run 中的固件已被确认健康；不在 CRC 覆盖内 */
    uint32_t crc;      /* 28: CRC16-MODBUS 覆盖偏移 0..23（magic..run_len） */
} s_flags;             /* 32B，无填充 */

#define FLAG_REC_SIZE     offsetof(s_flags, health)                   /* CRC 覆盖长度 = 24 */
#define FLAG_HEALTH_ADDR  (FLAG_REC_ADDR + offsetof(s_flags, health)) /* = 0x08020018 */

/* 编译期护栏：字段偏移锁定。
 * app 工程按硬编码地址 0x08020018 写 health —— 布局一旦漂移，
 * 症状是"每次升级后莫名回退"，不会有任何编译期或运行期提示，
 * 所以这里用负长度数组把错误挡在编译阶段（ARMCC 只会在条件为假时才报错）。 */
typedef char Flags_HealthOffset_Check[(offsetof(s_flags, health) == 24u) ? 1 : -1];
typedef char Flags_CrcOffset_Check   [(offsetof(s_flags, crc) == 28u) ? 1 : -1];
typedef char Flags_StructSize_Check  [(sizeof(s_flags) == 32u) ? 1 : -1];

/* 读状态记录到 *f。记录损坏（magic/CRC 不符）按"全无效"处理：
 * 填默认 {seq=0, b_ver=NO_VER, run_ver=NO_VER} 并返回 0。
 * 返回 1 = 读到一份可信记录。
 * 无论记录有效与否，f->health 始终填的是 Flash 里的真实字节。 */
uint8_t Flags_Load(s_flags *f);

/* 写状态记录：按 *f 内容算 CRC 后擦扇区 + 整写。
 * ★ health 被强制置为 HEALTH_NONE（见文件头不变量说明）。
 * 返回 0=成功, 1=失败。 */
uint8_t Flags_Save(const s_flags *f);

/* 健康标志：1 = run 中的固件已被 app 确认健康。
 * 只有精确等于 HEALTH_OK 才算确认，其它任何值（含撕裂产生的野值）按未确认处理。 */
uint8_t Health_Read(void);

#endif /* __FLAGS_H */
