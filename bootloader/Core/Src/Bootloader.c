#include "Bootloader.h"
#include "protocol.h"
#include "update.h"
#include "flash_if.h"
#include "ota_net.h"
#include "ota_spi.h"
#include "lwip.h"
#include "usart.h"
#include "lcd.h"
#include "stm32f4xx_hal.h"
#include <string.h>

/* 接收缓冲区 */
uint8_t  rx_buffer[BOOTLOADER_UART_REC_BUFFER_SIZE] = {0};
uint16_t rx_size = 0;

/* 已写入下载区A的字节数（由协议 OTA_DATA 分发更新，供 LCD 显示） */
uint32_t g_dl_off = 0u;

/* ==================== 环形缓冲（三条通道的唯一汇合点） ====================
 * UART / ETH / SPI 三个通道收到的字节都进这一个环，由 Bootloader_Task() 统一消费
 * 喂给 Proto_Feed()。因此帧状态机、命令处理、Flash 写入路径全部只有一份，
 * 通道差异到此为止 —— 三个驱动唯一的区别就是写环时声明的那个通道号。
 *
 * ★★ 已知假设（本设计里**唯一**一处启发式，必须清楚它的边界）：
 *   环里只存字节，不存"每个字节来自哪条通道"；来源用 g_ring_src 这个
 *   "最后写入者"的标记来近似。它正确的**前提是同一时刻只有一条通道在投递字节**：
 *   PC 一次只在一个通道上升级，其余通道即使连着也是**空闲**的（不产生任何字节）。
 *   这个前提在本项目里成立 —— 上位机一次只用一个 transport，而 TCP 连接空闲时
 *   lwIP 不会凭空投递字节（没开 keepalive 数据、没有心跳包）。
 *   反过来说：**如果将来出现了"两条通道同时有字节流进来"，这里会张冠李戴**，
 *   表现为"ACK 发去了另一条通道"。真到了那一步，正确的修法是给环加逐段来源标注
 *   （每个 chunk 带一个通道号），而不是在这里继续打补丁。 */
#define RING_SIZE 8192u                    /* 8KB：容得下一个 1KB 帧被拆散时的多段到达 */
static uint8_t  g_ring[RING_SIZE];
static volatile uint16_t g_ring_head = 0u;  /* 生产指针（ISR 与主循环都写） */
static uint16_t g_ring_tail = 0u;           /* 消费指针（仅主循环写） */

/* 最后一次**真正写进环**的字节来自哪条通道。
 * ★ 只在"确实写入了 ≥1 字节"时更新：写 0 字节（环满）却把标记改掉，
 *   会让下一批属于原通道的字节被标成新通道。 */
static volatile OtaChan_t g_ring_src = OTA_CH_NONE;

/**
 * @brief 写入环形缓冲，返回实际写入字节数
 *
 * ★ 为什么必须进临界区：生产者有两个 —— UART 空闲中断 和 主循环里的 ETH 接收回调。
 *   两者会互相打断（ISR 抢占主循环）。若无保护会出现这样的竞态：
 *     主循环读 head=100 → 被 UART ISR 抢占 → ISR 也读 head=100，写入并推进到 105
 *     → 回到主循环，它仍按 head=100 写 → 覆盖 ISR 刚写的数据，且把 head 回拨。
 *   临界区最长就是 len 字节的拷贝（8KB 全拷也仅数十微秒 @168MHz），
 *   对 UART 中断延迟的影响可忽略。
 *
 * ★ __set_PRIMASK 恢复调用前的中断状态，因此本函数在 ISR 内部调用同样安全
 *   （不会在中断里提前开中断）。
 */
uint16_t Bootloader_RingWrite(OtaChan_t ch, const uint8_t *p, uint16_t len)
{
    uint16_t i;
    uint16_t head;
    uint16_t tail;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();

    head = g_ring_head;
    tail = g_ring_tail;

    for (i = 0u; i < len; i++)
    {
        if ((uint16_t)((head + 1u) % RING_SIZE) == tail)   /* 留一格用于区分空/满 */
        {
            break;                                          /* 环满：停止写入 */
        }
        g_ring[head] = p[i];
        head = (uint16_t)((head + 1u) % RING_SIZE);
    }
    g_ring_head = head;

    /* 只有真的写进去了才更新来源标记 —— 见 g_ring_src 的说明。
     * 与 head 一起放在临界区内：这样**生产者侧**看到的永远是自洽的一对
     * （"这批字节写进去了，标记是它的来源"）。
     * ★ 注意这只是生产者侧的自洽，**不是对消费者的保证**：消费者在临界区外
     *   读这个标记（见 Bootloader_Task），它不取这把锁，所以下面这种时序照样
     *   可能发生 —— 环里已有 256B 来自 SPI，消费者读到标记 = SPI，此刻 USART3
     *   空闲中断抢入写进若干字节并把标记改成 UART，消费者随后 RingRead 的快照
     *   已包含那批字节，于是这批**混了通道**的数据被整体贴上 SPI 的标签。
     *   单通道前提下这条路走不到（见文件头的"已知假设"），
     *   真破了前提就只有"逐段带通道号"能解决。 */
    if (i > 0u)
    {
        g_ring_src = ch;
    }

    __set_PRIMASK(primask);

    return i;                          /* ★ 实际写入量：TCP 背压的依据 */
}

/* 环中待消费字节数 */
static uint16_t RingCount(void)
{
    return (uint16_t)((g_ring_head - g_ring_tail) % RING_SIZE);
}

/* 消费者（主循环独占）：从环取最多 max 字节到 out，返回实际取到的字节数 */
static uint16_t RingRead(uint8_t *out, uint16_t max)
{
    uint16_t n = RingCount();
    uint16_t i;
    uint16_t tail = g_ring_tail;

    if (n > max)
    {
        n = max;
    }
    for (i = 0u; i < n; i++)
    {
        out[i] = g_ring[tail];
        tail = (uint16_t)((tail + 1u) % RING_SIZE);
    }
    g_ring_tail = tail;
    return n;
}

/* ==================== 协议层发送出口（三通道择路） ====================
 * ★ 这里**只负责照着 ch 发**，不再判断"谁连着"。
 *   为什么：加 SPI 通道之前这里是"发送时刻看 TCP 通不通，通则走网口、否则串口"。
 *   三条通道都通之后这个判据就错了 —— PC 完全可能挂着一条**闲置的 ETH 连接**
 *   而升级走 WiFi(SPI)，于是 ACK 被发去 ETH，PC 在 SPI 侧永远等不到应答。
 *   现在通道由协议层锁存（见 protocol.h 的会话绑定），本函数退化成一张纯查表。 */
static uint8_t BootTxEx(OtaChan_t ch, const uint8_t *buf, uint16_t len)
{
    switch (ch)
    {
    case OTA_CH_ETH:
        return OTA_Net_Send(buf, len);

    case OTA_CH_SPI:
        return OTA_Spi_Send(buf, len);

    case OTA_CH_UART:
    default:
        /* 兜底也走串口：ch 为 NONE 时（上电后一条通道都还没收到过字节）
         * 与加 SPI 之前"TCP 没连接就走串口"的行为一致。 */
        return (HAL_UART_Transmit(&huart3, (uint8_t *)buf, len, 100u) == HAL_OK) ? 0u : 1u;
    }
}

/* ==================== 三条通道的接收汇点 ====================
 * 三个驱动都只认 OTA_RxSink_t（同型），通道号在这里定型后补上。
 * 写成 3 行的 static wrapper 而不是给每个驱动传一个"通道号参数"，
 * 是为了让驱动完全不需要知道 OTA_CH_xxx 这个枚举的存在 ——
 * 它们只负责"收到字节就交给上层"，通道身份是 boot 侧的组织方式。 */
static uint16_t EthSink(const uint8_t *p, uint16_t len)
{
    return Bootloader_RingWrite(OTA_CH_ETH, p, len);
}

static uint16_t SpiSink(const uint8_t *p, uint16_t len)
{
    return Bootloader_RingWrite(OTA_CH_SPI, p, len);
}

/* ==================== 初始化 ==================== */
void Bootloader_Init(void)
{
    /* 清除标志位 */
    __HAL_UART_CLEAR_IDLEFLAG(&huart3);
    __HAL_UART_CLEAR_OREFLAG(&huart3);

    Proto_Init();                          /* 帧状态机复位 + 通道绑定清零 */
    Proto_SetTx(BootTxEx);                 /* 注入发送出口：协议层不再直接碰外设 */

    OTA_Net_Init(EthSink);                 /* 起 TCP 监听，网口字节流汇入同一个环 */
    OTA_Spi_Init(SpiSink);                 /* SPI 通道（**F407 是主机**，对端 C3 做从机）。
                                            * ★ 必须在 MX_SPI2_Init() 之后 */

    /* 启动串口接收中断 */
    HAL_UARTEx_ReceiveToIdle_IT(&huart3, rx_buffer, BOOTLOADER_UART_REC_BUFFER_SIZE);
}

/* 主循环消费：环形缓冲数据喂入帧协议状态机
 * （收满整帧->CRC校验->按命令分发->写Flash）
 * 已无攒页/空闲flush：下载区在 OTA_START 时整区擦除，数据帧按偏移顺序写 */
void Bootloader_Task(void)
{
    static uint8_t tmp[256];
    uint16_t n;
    OtaChan_t src;

    while (RingCount() > 0u)
    {
        /* ★ 顺序：先取来源、再取数据。
         *   两种顺序**都不是保证** —— 消费者在临界区外读 g_ring_src、不取锁，
         *   所以"旧来源 + 新数据"这种错法照样可能（见 g_ring_src 的说明）。
         *   选这个顺序只有一个站得住的理由：标记因此**永远不新于**读到的数据，
         *   不会把一批字节归给一条"此刻还没往环里写过字节"的通道。
         *   真破了单通道前提，正确的修法是逐段带通道号，而不是在这里调顺序。 */
        src = g_ring_src;
        n = RingRead(tmp, (uint16_t)sizeof(tmp));
        Proto_Feed(tmp, n, src);   /* 内部收整帧后由协议分发 */
    }
}

/* 主流程：上电 5s 等待窗口 -> 无下载则按标志启动决策 -> 常驻等待下载/跳转，不返回 */
void Bootloader_Run(void)
{
    uint32_t t0 = HAL_GetTick();
    uint32_t left;
    uint8_t  session = 0u;

    /* ① BOOT_WAIT_MS 窗口：喂协议 + 喂网口 + 喂 SPI + LCD 倒计时；
     *    收到合法 OTA_START 转永久下载 */
    while ((HAL_GetTick() - t0) < BOOT_WAIT_MS)
    {
        /* ★★ 窗口期必须跑 SPI 事务，否则 WiFi 通道连 START 都收不到：
         *   SPI 的时钟是**主机（F407）发的**，我们不主动去问，C3 那边收到的字节
         *   就永远上不来 —— 它不是中断推给我们的。少这一行，表现是
         *   "PC 通过 WiFi 发起升级，5 秒窗口走完、boot 去启动 app 了"。
         *   节流在 OTA_Spi_Poll 内部（自己比时刻），这里放心每轮都调。 */
        OTA_Spi_Poll();

        /* ★ 窗口期必须持续喂 lwIP：TCP 三次握手、连接建立全靠它回包。
         *   不喂的话 PC 在 5s 窗口内根本连不上，ETH OTA 只能靠复位后抢时间。 */
        MX_LWIP_Process();
        Bootloader_Task();

        left = (BOOT_WAIT_MS - (HAL_GetTick() - t0) + 999u) / 1000u;  /* 剩余秒 5..1 */
        LCD_DisplayString(10, 10, 16, (u8 *)"Wait OTA:");
        LCD_DisplayNum(90, 10, left, 1, 16, 0);

        /* SPI 通道状态放第二行：这一步之后的三条通道里，只有它没有任何
         * 现成的输出通道（串口的日志走 USART3 数据线、ETH 有上位机侧可见），
         * 所以屏上必须能直接看出"SPI 那边到底通没通"。 */
        OTA_Spi_ShowDiag(10u, 40u);

        if (Proto_SessionStarted() != 0u)
        {
            session = 1u;
            break;
        }
    }

    /* ② 窗口内没来下载：启动决策（跳运行区/回退/固化/停留；跳转则本函数到此为止） */
    if (session == 0u)
    {
        Upd_StartupLaunch();
    }

    /* ③ 常驻（下载会话已开始 或 窗口超时后无有效固件可跳）：
     *    一直收帧，LCD 实时显示已写入下载区A的字节数，直至 OTA_END 跳转 */
    LCD_Clear(BLACK);
    LCD_DisplayString_color(10, 10, 16, (u8 *)"DL:", 65535, 0);
    for (;;)
    {
        /* ★ 常驻下载循环同样要跑 SPI 事务，理由与上面窗口期那一处完全相同。
         *   这是**整条 WiFi 升级链路的数据通路** —— 少了它，走 WiFi 的升级会在
         *   窗口期过后彻底静默（PC 看到的就是"START 之后没下文了"）。 */
        OTA_Spi_Poll();

        MX_LWIP_Process();     /* 下载全程靠它收发：擦写 Flash 的间隙也要回到这里 */
        Bootloader_Task();
        LCD_DisplayNum_color(50, 10, g_dl_off, 6, 16, 0, 65535, 0);
        OTA_Spi_ShowDiag(10u, 40u);
    }
}

/* 串口空闲中断回调：仅把数据搬进环，帧解析/Flash 写入由主循环 Bootloader_Task 完成 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART3)
    {
        rx_size = Size;

        /* 串口没有背压手段：环满就丢，丢掉的帧靠上位机 stop-and-wait 超时重发 */
        (void)Bootloader_RingWrite(OTA_CH_UART, rx_buffer, rx_size);

        memset(rx_buffer, 0, BOOTLOADER_UART_REC_BUFFER_SIZE);   /* 清空后重启接收 */
        HAL_UARTEx_ReceiveToIdle_IT(&huart3, rx_buffer, BOOTLOADER_UART_REC_BUFFER_SIZE);
    }
}
