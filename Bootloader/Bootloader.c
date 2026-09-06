#include "Bootloader.h"
#include "protocol.h"
#include "update.h"
#include "OLED.h"

//接收缓冲区
uint8_t rx_buffer[BOOTLOADER_UART_REC_BUFFER_SIZE] = {0};
//接收数据长度
uint16_t rx_size = 0;

//已写入下载区A的字节数（由协议 OTA_DATA 分发更新，供 OLED 显示）
uint32_t g_dl_off = 0;

/* ==================== 环形缓冲 ==================== */
#define RING_SIZE 4096u                 /* 缓冲 4KB */
static uint8_t  g_ring[RING_SIZE];      /* 数据缓冲 */
static volatile uint16_t g_ring_head = 0;  /* 生产指针（ISR 写） */
static uint16_t g_ring_tail = 0;        /* 消费指针（主循环写） */

//初始化Bootloader
void Bootloader_Init(void)
{
    //清除标志位
    __HAL_UART_CLEAR_IDLEFLAG(&huart1); //清除IDLE标志位
    __HAL_UART_CLEAR_OREFLAG(&huart1); //清除ORE标志位

    Proto_Init();                       //帧状态机复位

    //启动串口接收中断
    HAL_UARTEx_ReceiveToIdle_IT(&huart1, rx_buffer, BOOTLOADER_UART_REC_BUFFER_SIZE);

}

/* ==================== 环形缓冲操作 ==================== */

//环中待消费字节数
static uint16_t RingCount(void)
{
    return (uint16_t)((g_ring_head - g_ring_tail) % RING_SIZE);
}

//生产者（ISR）：拷入环形缓冲
static void RingWrite(const uint8_t *p, uint16_t len)
{
    uint16_t i;
    uint16_t head = g_ring_head;

    for (i = 0; i < len; i++)
    {
        g_ring[head] = p[i];
        head = (head + 1) % RING_SIZE;
        if (head == g_ring_tail)  //环满
        {
            break;
        }
    }
    g_ring_head = head;
}

//消费者（主循环）：从环取最多 max 字节到 out，返回实际取到的字节数
static uint16_t RingRead(uint8_t *out, uint16_t max)
{
    uint16_t n = RingCount();
    uint16_t i, tail = g_ring_tail;

    if (n > max)
    {
        n = max;
    }
    for (i = 0; i < n; i++)
    {
        out[i] = g_ring[tail];
        tail = (tail + 1) % RING_SIZE;
    }
    g_ring_tail = tail;
    return n;
}

//主循环消费：环形缓冲数据喂入帧协议状态机（收满整帧->CRC校验->按命令分发->写Flash）
//已无攒页/空闲flush：下载区在 OTA_START 时整区擦除，数据帧按偏移顺序写
void Bootloader_Task(void)
{
    static uint8_t tmp[256];
    uint16_t n;

    while (RingCount() > 0)
    {
        n = RingRead(tmp, sizeof(tmp));
        Proto_Feed(tmp, n);   //内部收整帧后由协议分发
    }
}

//主流程：上电 5s 等待窗口 -> 无下载则按标志启动决策 -> 常驻等待下载/跳转，不返回
void Bootloader_Run(void)
{
    uint32_t t0 = HAL_GetTick();
    uint8_t  session = 0u;

    /* ① BOOT_WAIT_MS 窗口：喂协议 + OLED 倒计时；收到合法 OTA_START 转永久下载 */
    while ((HAL_GetTick() - t0) < BOOT_WAIT_MS)
    {
        Bootloader_Task();
        OLED_ShowNum(1, 1, (BOOT_WAIT_MS - (HAL_GetTick() - t0) + 999u) / 1000u, 1); /* 剩余秒 5..1 */
        OLED_ShowString(2, 1, "Wait OTA 5s");
        if (Proto_SessionStarted())
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
     *    一直收帧，OLED 第1行实时显示已下载(写入下载区A)的字节数，直至 OTA_END 跳转 */
    OLED_Clear();
    for (;;)
    {
        Bootloader_Task(); //从环形缓冲取数据（ISR只收不写）
        OLED_ShowString(1, 1, "DL:");
        OLED_ShowNum(1, 4, g_dl_off, 5); //已下载字节数(0→总长)
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1)
    {
        rx_size = Size; //记录接收到的数据长度

        //中断中仅将数据拷入环形缓冲，帧解析/Flash写入由主循环Bootloader_Task完成
        RingWrite(rx_buffer, rx_size);

        //处理完数据后，清空接收缓冲区,重新启动接收中断
        memset(rx_buffer, 0, BOOTLOADER_UART_REC_BUFFER_SIZE);
        HAL_UARTEx_ReceiveToIdle_IT(&huart1, rx_buffer, BOOTLOADER_UART_REC_BUFFER_SIZE);
    }
}
