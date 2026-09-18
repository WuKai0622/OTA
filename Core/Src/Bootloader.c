#include "Bootloader.h"
#include "protocol.h"
#include "update.h"
#include "flash_if.h"
#include "ota_net.h"
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

/* ==================== 环形缓冲（UART 与 ETH 两个通道的唯一汇合点） ====================
 * 两个通道收到的字节都进这一个环，由 Bootloader_Task() 统一消费喂给 Proto_Feed()。
 * 因此帧状态机、命令处理、Flash 写入路径全部只有一份，通道差异到此为止。 */
#define RING_SIZE 8192u                    /* 8KB：容得下一个 1KB 帧被拆散时的多段到达 */
static uint8_t  g_ring[RING_SIZE];
static volatile uint16_t g_ring_head = 0u;  /* 生产指针（ISR 与主循环都写） */
static uint16_t g_ring_tail = 0u;           /* 消费指针（仅主循环写） */

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
uint16_t Bootloader_RingWrite(const uint8_t *p, uint16_t len)
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

/* ==================== 协议层发送出口（双通道择一） ====================
 * TCP 有连接时优先走网口：有窗口背压、非阻塞；
 * 否则回落串口（阻塞发送，100ms 超时）。
 * 说明：这是"通道选择"而非"双发"——一次升级只在一个通道上进行，
 *       上位机不会同时从两个通道发帧，所以回答走最近的活跃通道即可。 */
static uint8_t BootTx(const uint8_t *buf, uint16_t len)
{
    if (OTA_Net_Connected() != 0u)
    {
        return OTA_Net_Send(buf, len);
    }
    return (HAL_UART_Transmit(&huart3, (uint8_t *)buf, len, 100u) == HAL_OK) ? 0u : 1u;
}

/* ==================== 初始化 ==================== */
void Bootloader_Init(void)
{
    /* 清除标志位 */
    __HAL_UART_CLEAR_IDLEFLAG(&huart3);
    __HAL_UART_CLEAR_OREFLAG(&huart3);

    Proto_Init();                          /* 帧状态机复位 */
    Proto_SetTx(BootTx);                   /* 注入发送出口：协议层不再直接碰外设 */
    OTA_Net_Init(Bootloader_RingWrite);    /* 起 TCP 监听，网口字节流汇入同一个环 */

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

    while (RingCount() > 0u)
    {
        n = RingRead(tmp, (uint16_t)sizeof(tmp));
        Proto_Feed(tmp, n);   /* 内部收整帧后由协议分发 */
    }
}

/* 主流程：上电 5s 等待窗口 -> 无下载则按标志启动决策 -> 常驻等待下载/跳转，不返回 */
void Bootloader_Run(void)
{
    uint32_t t0 = HAL_GetTick();
    uint32_t left;
    uint8_t  session = 0u;

    /* ① BOOT_WAIT_MS 窗口：喂协议 + 喂网口 + LCD 倒计时；收到合法 OTA_START 转永久下载 */
    while ((HAL_GetTick() - t0) < BOOT_WAIT_MS)
    {
        /* ★ 窗口期必须持续喂 lwIP：TCP 三次握手、连接建立全靠它回包。
         *   不喂的话 PC 在 5s 窗口内根本连不上，ETH OTA 只能靠复位后抢时间。 */
        MX_LWIP_Process();
        Bootloader_Task();

        left = (BOOT_WAIT_MS - (HAL_GetTick() - t0) + 999u) / 1000u;  /* 剩余秒 5..1 */
        LCD_DisplayString(10, 10, 16, (u8 *)"Wait OTA:");
        LCD_DisplayNum(90, 10, left, 1, 16, 0);

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
    LCD_DisplayString_color(10, 10, 16, (u8 *)"DL:", 655350, 0);
    for (;;)
    {
        MX_LWIP_Process();     /* 下载全程靠它收发：擦写 Flash 的间隙也要回到这里 */
        Bootloader_Task();
        LCD_DisplayNum_color(50, 10, g_dl_off, 6, 16, 0, 65535, 0);
    }
}

/* 串口空闲中断回调：仅把数据搬进环，帧解析/Flash 写入由主循环 Bootloader_Task 完成 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART3)
    {
        rx_size = Size;

        /* 串口没有背压手段：环满就丢，丢掉的帧靠上位机 stop-and-wait 超时重发 */
        (void)Bootloader_RingWrite(rx_buffer, rx_size);

        memset(rx_buffer, 0, BOOTLOADER_UART_REC_BUFFER_SIZE);   /* 清空后重启接收 */
        HAL_UARTEx_ReceiveToIdle_IT(&huart3, rx_buffer, BOOTLOADER_UART_REC_BUFFER_SIZE);
    }
}
