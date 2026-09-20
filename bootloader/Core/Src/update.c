#include "update.h"
#include "flash_if.h"
#include "flags.h"
#include "ota_net.h"
#include "ota_spi.h"
#include "stm32f4xx_hal.h"

/* 分块搬运缓冲（RAM）：一次搬一块再写，避免整区大数组占用 RAM。
 * F407 有 128KB SRAM，比 F103 宽裕，块取大一点减少循环次数。 */
#define UPD_COPY_CHUNK   1024u
static uint8_t g_cpy[UPD_COPY_CHUNK];

/**
 * @brief 把 Flash 源区拷贝到目标区（读 Flash→擦目标扇区→字写）
 * @param src  源地址
 * @param dst  目标地址（须扇区对齐）
 * @param len  拷贝字节数（≤ APP_MAX_SIZE）
 * @retval 0=成功, 1=失败
 */
static uint8_t Flash_CopyRegion(uint32_t src, uint32_t dst, uint32_t len)
{
    uint32_t off = 0u;

    if (len == 0u || len > APP_MAX_SIZE)
    {
        return 1u;
    }

    /* 目标区按需整擦（各分区均扇区对齐；擦除不会波及 boot 所在扇区） */
    if (Flash_EraseRegion(dst, len) != 0u)
    {
        return 1u;
    }

    /* 分块：读源区到 RAM → 写目标区 */
    while (off < len)
    {
        uint32_t n = len - off;
        if (n > UPD_COPY_CHUNK)
        {
            n = UPD_COPY_CHUNK;
        }
        Flash_ReadBuffer(src + off, g_cpy, n);
        if (Flash_ProgramBytes(dst + off, g_cpy, n) != 0u)
        {
            return 1u;
        }
        off += n;
    }
    return 0u;
}

/**
 * @brief 运行区首址的向量表合理性检查（跳转前兜底）
 * word0 = 初始 MSP（须落在 SRAM/CCM 内），word1 = Reset_Handler（须落在运行区内）
 * @retval 1=通过, 0=不通过
 */
static uint8_t Upd_VectorOk(uint32_t addr)
{
    uint32_t msp = *(volatile uint32_t *)addr;
    uint32_t rst = *(volatile uint32_t *)(addr + 4u);
    uint8_t  msp_ok;

    /* F407ZGT6：SRAM1+SRAM2 = 128KB @0x20000000；CCM = 64KB @0x10000000。
     * 初始 MSP 必须落在这两块之一，否则向量表无效。 */
    msp_ok = (((msp >= 0x20000000u) && (msp <= 0x20020000u))
           || ((msp >= 0x10000000u) && (msp <= 0x10010000u))) ? 1u : 0u;
    if (msp_ok == 0u)
    {
        return 0u;
    }
    if (rst == 0xFFFFFFFFu)
    {
        return 0u;
    }
    if (rst < APP_RUN_ADDR || rst >= (APP_RUN_ADDR + APP_MAX_SIZE))
    {
        return 0u;
    }
    return 1u;
}

typedef void (*pFunction)(void);

/**
 * @brief 跳转执行运行区 app（不返回）
 * 先停 ETH（DMA 是总线主设备，不停会在 app 运行期继续写 boot 的描述符环），
 * 再关中断、清外设中断使能/挂起、停 SysTick、搬向量表、切栈后跳 Reset_Handler。
 */
static void Upd_JumpToApp(uint32_t addr)
{
    uint32_t msp = *(volatile uint32_t *)addr;
    pFunction app_entry = (pFunction)(*(volatile uint32_t *)(addr + 4u));
    uint32_t i;

    OTA_Net_DeInit();                      /* ★ 必须在关中断之前：内部要动 lwIP 和 ETH DMA */

    /* ★ 拆 SPI 之前把发送队清空 —— 这是队列被销毁前的最后一道。
     *   （正常路径上队在 Upd_OnDownloadComplete 开头就已经排空了，所以这里是
     *     零开销的兜底；留着是因为"跳转"是本队列唯一的销毁点，
     *     将来任何在跳转前入队的应答都得靠它。）
     *   ★ 这里的预算同样用 OTA_SPI_FLUSH_MS：这是 ACK 的**最后一次机会** ——
     *     下面一句就把队列销毁了，而跳转之后上位机重发的 END 再也没人应答。 */
    OTA_Spi_FlushTx(OTA_SPI_FLUSH_MS);

    OTA_Spi_DeInit();                      /* ★ 同理：不能让 SPI2 带着已使能状态交给 app。
                                            *   app 会重新 MX_SPI2_Init()，两套配置打架的结果
                                            *   是"偶发、且看起来跟 OTA 毫无关系"的故障。 */
    __disable_irq();                       /* 关全局中断 */
    for (i = 0u; i < 8u; i++)
    {
        NVIC->ICER[i] = 0xFFFFFFFFu;       /* 清中断使能（F407 只实现 ICER[0..2]，其余写无效） */
        NVIC->ICPR[i] = 0xFFFFFFFFu;       /* 清中断挂起，防残留 IRQ 误入 app */
    }
    SysTick->CTRL = 0u;                    /* 停滴答，由 app 重新配置 */
    SysTick->VAL  = 0u;
    SCB->VTOR = addr;                      /* 向量表搬至运行区（各分区 128KB 对齐，满足 VTOR 对齐要求） */
    __set_MSP(msp);                        /* 切换到 app 初始栈指针 */
    __enable_irq();                        /* 恢复复位态(PRIMASK=0)，供 app 正常使用中断 */
    app_entry();                           /* 进入 Reset_Handler，不再返回 */
}

/* ---------------- 以下为 A/B v2 运行时回滚相关 ---------------- */

/** 长度兜底：0 / 越界(撕裂)按整区处理，保证拷贝范围安全 */
static uint32_t Upd_NormLen(uint32_t len)
{
    return (len > 0u && len <= APP_MAX_SIZE) ? len : APP_MAX_SIZE;
}

/** 下一版本序，避免撞上 NO_VER/WIP 哨兵值 */
static uint32_t Upd_NextSeq(uint32_t s)
{
    if (s == SEQ_NO_VER || s == SEQ_B_WIP)
    {
        s = 0u;
    }
    s += 1u;
    if (s == SEQ_NO_VER || s == SEQ_B_WIP)
    {
        s = 1u;
    }
    return s;
}

/**
 * @brief 把当前运行区固件存为 B（种子化/首次保护用）。不擦 run，断电安全。
 * run 无版本(旧版直接下载)时先给它一个新 seq。成功后调用方负责 Flags_Save。
 * @retval 0=成功(已拷 B, f->b_ver/b_len 待落盘), 1=失败(run 无有效固件或拷失败)
 */
static uint8_t Upd_SeedBFromRun(s_flags *f)
{
    uint32_t len;

    if (Upd_VectorOk(APP_RUN_ADDR) == 0u)
    {
        return 1u;                      /* run 空/坏，无法做种子 */
    }
    len = Upd_NormLen(f->run_len);
    if (f->run_ver == SEQ_NO_VER)       /* 旧工程直接下载留下的 app：补一个版本序 */
    {
        f->seq     = Upd_NextSeq(f->seq);
        f->run_ver = f->seq;
        f->run_len = len;
    }
    if (Flash_CopyRegion(APP_RUN_ADDR, APP_BK_ADDR, len) != 0u)
    {
        return 1u;
    }
    f->b_ver = f->run_ver;
    f->b_len = len;
    return 0u;
}

/**
 * @brief 用 B 恢复运行区（回退）。成功后调用方负责 Flags_Save；返回后停留，由下次复位跳 run。
 */
static void Upd_RestoreRunFromB(s_flags *f)
{
    uint32_t len;

    if (f->b_ver == SEQ_NO_VER || f->b_ver == SEQ_B_WIP)
    {
        return;                         /* 无可用 B → 停留，等重下 */
    }
    len = Upd_NormLen(f->b_len);
    if (Flash_CopyRegion(APP_BK_ADDR, APP_RUN_ADDR, len) != 0u)
    {
        return;                         /* 拷失败：停留，下次复位再试 */
    }
    f->run_ver = f->b_ver;
    f->run_len = len;
    Flags_Save(f);
}

/* ------------------------------------------------------------------ */

void Upd_StartupLaunch(void)
{
    s_flags f;
    uint32_t len;

    Flags_Load(&f);

    if (f.b_ver == SEQ_NO_VER || f.b_ver == SEQ_B_WIP)
    {
        /* ---- 无可用 B ---- */
        if (Upd_VectorOk(APP_RUN_ADDR) != 0u)
        {
            /* 仅有的固件在 run → 种子化 B（旧版直接下载的 app 也自动纳入保护） */
            if (Upd_SeedBFromRun(&f) == 0u)
            {
                Flags_Save(&f);                     /* 同时清健康 */
                Upd_JumpToApp(APP_RUN_ADDR);        /* 不返回 */
            }
        }
        /* 无固件可跳 → 返回，boot 常驻等待下载/SWD */
        return;
    }

    /* ---- 有可用 B ---- */
    if (Upd_VectorOk(APP_RUN_ADDR) != 0u && f.run_ver == f.b_ver)
    {
        /* 常态：run 就是已确认的好版本 → 快速启动，不拷贝也不写记录（记录没变） */
        Upd_JumpToApp(APP_RUN_ADDR);                /* 不返回 */
    }
    else if (Upd_VectorOk(APP_RUN_ADDR) != 0u && f.run_ver != SEQ_NO_VER
             && f.run_ver > f.b_ver && Health_Read() != 0u)
    {
        /* 候选已跑起来并写过健康标志 → 固化进 B，成为新的好版本 */
        len = Upd_NormLen(f.run_len);
        f.b_ver = SEQ_B_WIP;                        /* 先作废 B：固化拷贝中断则 B 不可信 */
        Flags_Save(&f);
        if (Flash_CopyRegion(APP_RUN_ADDR, APP_BK_ADDR, len) == 0u)
        {
            f.b_ver = f.run_ver;
            f.b_len = len;
            Flags_Save(&f);
            Upd_JumpToApp(APP_RUN_ADDR);            /* 不返回 */
        }
        /* 固化中断/失败：b_ver 留在 B_WIP，本次停留；下次上电按"无B+run健康"重种子恢复 */
        return;
    }
    else
    {
        /* 候选未确认健康 / run 坏 / run_ver 倒退 → 回退 B→run */
        len = Upd_NormLen(f.b_len);
        if (Flash_CopyRegion(APP_BK_ADDR, APP_RUN_ADDR, len) == 0u)
        {
            f.run_ver = f.b_ver;
            f.run_len = len;
            Flags_Save(&f);
            if (Upd_VectorOk(APP_RUN_ADDR) != 0u)
            {
                Upd_JumpToApp(APP_RUN_ADDR);        /* 不返回 */
            }
            else
            {
                /* B 内容本身已坏(理论极罕见)：丢弃 B，避免每次复位反复空拷 */
                f.b_ver = SEQ_NO_VER;
                Flags_Save(&f);
                return;
            }
        }
        return;                                     /* 拷失败：停留 */
    }
}

void Upd_OnDownloadComplete(uint32_t total_len)
{
    s_flags f;

    /* ★★ 第一件事：把 OTA_END 的应答**推上总线**，然后再开始擦写。
     *
     *   调用方（protocol.c 的 CMD_OTA_END 分支）刚刚才把 ACK 交给发送层，而
     *   SPI 的发送层是个**队列**——入队 ≠ 发出去，要等主循环再跑一次事务才会
     *   真的上总线。而本函数下面做的是擦扇区 + 拷 128KB，是**秒级**的阻塞操作，
     *   主循环全程回不来。上位机那头 END_TIMEOUT = 3.0 秒，等不到就重发、
     *   重试耗尽后报"升级失败"——可固件其实已经写好了。
     *
     *   ETH 通道没有这个问题（lwIP 的发送由 ETH DMA 独立完成，与我们卡不卡在
     *   擦除里无关）；UART 是阻塞发送、发完才返回。只有队列式的 SPI 会把
     *   "入队成功"当成"已发出"。
     *
     *   队列空时本调用零开销，所以这里无条件执行，不做通道判断——
     *   在 update.c 里区分通道是错的分层，它只该知道"发完了再干活"。
     *
     *   ★ 预算给到 OTA_SPI_FLUSH_MS（2000ms）而不是几十毫秒：一次 HAL 超时就
     *     要 100ms，预算比它小的话**一次抖动就能让这根 ACK 永远发不出去**。
     *     详见 ota_spi.h 里 OTA_SPI_FLUSH_MS 那段的算式。 */
    OTA_Spi_FlushTx(OTA_SPI_FLUSH_MS);

    if (total_len == 0u || total_len > APP_MAX_SIZE)
    {
        return;
    }
    Flags_Load(&f);

    /* ① 无 B 时，把当前 run 的旧固件先收进 B 作保护（首次升级/旧版迁移安全网） */
    if ((f.b_ver == SEQ_NO_VER || f.b_ver == SEQ_B_WIP)
        && Upd_SeedBFromRun(&f) == 0u)
    {
        Flags_Save(&f);                             /* 保留旧版：坏候选有处可退 */
    }

    /* ② 声明候选（先记录后落内容）：断电安全靠"候选未确认→上电回退 B"兜底。
     *    Flags_Save 内已强制清健康，无需单独调用 —— 旧健康标志不可能被误认到新候选上。 */
    f.seq     = Upd_NextSeq(f.seq);
    f.run_ver = f.seq;
    f.run_len = total_len;
    Flags_Save(&f);

    /* ③ A → 运行区：候选试运行 */
    if (Flash_CopyRegion(APP_DL_ADDR, APP_RUN_ADDR, total_len) != 0u)
    {
        Flags_Load(&f);                             /* 重载，含①可能刚存的 B */
        Upd_RestoreRunFromB(&f);                    /* 有 B 就地恢复；无 B 停留 */
        return;
    }
    if (Upd_VectorOk(APP_RUN_ADDR) != 0u)
    {
        Upd_JumpToApp(APP_RUN_ADDR);                /* 不返回：进候选试运行 */
    }
    /* ④ 候选向量非法（垃圾固件）：从 B 恢复后停留，可整包重发 */
    Flags_Load(&f);
    Upd_RestoreRunFromB(&f);
}
