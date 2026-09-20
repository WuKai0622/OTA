#include "ota_spi.h"

#include <string.h>

#include "spi.h"       /* hspi2 */
#include "lcd.h"       /* 诊断行直接画在屏上 —— boot 阶段没有别的输出通道
                        * （USART3 是 OTA 数据通道，不能拿来打日志） */

/* ==================== 物理层 ==================== */

uint8_t OTA_Spi_Transfer(const uint8_t *tx, uint8_t *rx, uint16_t len)
{
    HAL_StatusTypeDef st;

    /* 长度为 0 直接当成功返回；空指针是调用方的 bug，返回失败而不是让它
     * 一路走到 HAL 里面去（HAL 拿到 NULL 不会报错，只会读写出无意义的地址）。 */
    if ((len == 0u) || (tx == NULL) || (rx == NULL))
    {
        return (len == 0u) ? 0u : 1u;
    }

    OTA_SPI_CS_LOW();

    /* ★ 必须用 TransmitReceive，不能用 Transmit + Receive 分两次：
     *   全双工是**同一段时钟里**收发。从机要在本次事务中回送的头
     *   （RX_LEN/STATUS/HB）只在这段时钟里出现在 MISO 上，分开调用就等于
     *   让它白发一趟，而且第二次调用还会白送一段时间钟给从机。
     *
     * ★ 这是**阻塞式**的：264B @1.3125MHz ≈ 1.61ms 全程忙等（轮询标志位）。
     *   所以"多久开一笔"直接等于"抢走多少 CPU"，节流策略见 OTA_SPI_ACTIVE_MS。
     *
     * tx 转掉 const：HAL 的签名是 uint8_t*，但它只读不写。 */
    st = HAL_SPI_TransmitReceive(&hspi2, (uint8_t *)tx, rx, len, OTA_SPI_TIMEOUT_MS);

    OTA_SPI_CS_HIGH();

    if (st != HAL_OK)
    {
        /* ★★ 失败之后必须把 SPI 软复位一次，否则这一笔能把整条通道**永久打死**。
         *
         *   HAL 的超时分支只做 State=READY + 解锁就返回（见 stm32f4xx_hal_spi.c
         *   的超时处理），**既不禁用外设、也不读走 DR**。于是超时那一刻若恰好有
         *   一个字节已经进了接收寄存器，它就**留在 DR 里**；下一笔事务的头一次读
         *   会把这个陈旧字节当成 s_miso[0]，其后所有字节**整体错位 1 字节** ——
         *   而且错位是自持的（每笔都多留一个），不会自愈。
         *   后果：s_miso[6..7] 的结构校验从此**每一笔都失败**，s_bad_cnt 一路涨、
         *   s_rx_total 再也不动。屏上表现是"RX 冻住、E 狂涨"，而 TCP 连接还在
         *   （C3 好好的）—— 于是排查方向会被带到电源/接线上，白白耗掉一天。
         *
         *   禁用外设会复位它的内部状态机，读掉 DR/SR 清掉 OVR 与残留数据。
         *   此刻 CS 已经拉高（从机已释放），所以这个软复位不会在总线上产生毛刺。
         *   SPE 之外的 CR1 位（CPOL/CPHA/BR/MSTR/DFF...）不受影响，配置不会被改掉。 */
        __HAL_SPI_DISABLE(&hspi2);
        (void)hspi2.Instance->DR;
        (void)hspi2.Instance->SR;
        __HAL_SPI_ENABLE(&hspi2);
        return 1u;
    }

    return 0u;
}

/* ==================== 通道层状态 ==================== */

/* 接收汇点，由 OTA_Spi_Init() 注入 */
static OTA_RxSink_t s_sink = 0;

/* 一次事务的两个缓冲。都是**静态**的：264B × 2 放栈上会吃掉主循环的栈余量，
 * 而且它们本来就该是常驻的（每笔事务都要用，重新 memset 也是浪费）。 */
static uint8_t s_mosi[OTA_SPI_XFER_LEN];
static uint8_t s_miso[OTA_SPI_XFER_LEN];

/* ---- 发送缓冲：协议层"发"出来的字节先入队，由事务搬上总线 ----
 * ★ 为什么必须隔一层队列：协议层的 SendAck 是在 Bootloader_Task 里调用的
 *   （解析完一帧就地应答），而字节真正上总线是在 OTA_Spi_Poll 里 ——
 *   两者不是同一个调用点，中间必须有地方存。
 * ★ 单生产者单消费者，且**两者都在主循环上下文**（协议层的调用链是
 *   Bootloader_Task → Proto_Feed → ... → OTA_Spi_Send），所以这里
 *   **不需要临界区**。将来谁要是从 ISR 里调 OTA_Spi_Send，必须补上。 */
static uint8_t  s_txb[OTA_SPI_TX_BUF];
static uint16_t s_txb_head = 0u;    /* 写指针（生产者） */
static uint16_t s_txb_tail = 0u;    /* 读指针（消费者：事务） */
#define TXB_MASK ((uint16_t)(OTA_SPI_TX_BUF - 1u))

/* ---- 事务层状态 ---- */
static uint16_t s_sq       = 0u;        /* MOSI 帧序号，每笔自增（含重发那一笔） */
static uint16_t s_status   = 0u;        /* 最近一笔 MISO 的 STATUS */
static uint16_t s_hb       = 0xFFFFu;   /* 最近一笔 MISO 的心跳（哨兵值见 Init） */
static uint32_t s_hb_tick  = 0u;        /* 心跳最后一次变化的时刻 */
static uint32_t s_last_poll = 0u;       /* 上一笔事务的**起点**时刻（节流用） */
static uint32_t s_last_act = 0u;        /* 最后一次有收发的时刻（档位切换用） */

/* ---- 统计（供 LCD 与排查用）----
 * 口径都是**从上电起累计**：boot 一次上电只服务有限几次升级，
 * 累计值足够定位问题，不需要像 C3 那样按会话清零。 */
static uint32_t s_rx_total = 0u;   /* 从 C3 收到并投递给 sink 的有效字节数 */
static uint32_t s_tx_total = 0u;   /* 交给 C3 的有效字节数（已上总线的） */
static uint32_t s_bad_cnt  = 0u;   /* MISO 头结构校验失败（帧错位/噪声） */
static uint32_t s_oob_cnt  = 0u;   /* MISO 的 RX_LEN 越界 */
static uint32_t s_err_cnt  = 0u;   /* HAL 超时（SCK 没敲起来 / 从机没挂上） */

/* 小端读写。定死字节序，别依赖两边机器的巧合。 */
static uint16_t RdU16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static void WrU16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
}

static uint16_t TxCount(void)
{
    return (uint16_t)((s_txb_head - s_txb_tail) & TXB_MASK);
}

/* 取发送缓冲里的字节拷进 dst，**不动读写指针** —— 真正的"已发出"要等事务成功
 * 回来才由 TxDrop 确认。见 SpiOneTransaction 里那处说明。 */
static uint16_t TxPeek(uint8_t *dst, uint16_t max)
{
    uint16_t n = TxCount();
    uint16_t i;

    if (n > max)
    {
        n = max;
    }
    for (i = 0u; i < n; i++)
    {
        dst[i] = s_txb[(uint16_t)((s_txb_tail + i) & TXB_MASK)];
    }
    return n;
}

static void TxDrop(uint16_t n)
{
    s_txb_tail = (uint16_t)((s_txb_tail + n) & TXB_MASK);
}

/* ==================== 通道层：一笔事务 ==================== */

/* 跑一笔完整事务：装 MOSI → 收发 → 解 MISO → 投递。
 * 不做节流判断（那是 OTA_Spi_Poll 的事），也不做"发送缓冲空就不发"的优化 ——
 * 空帧也**必须**照发：STATUS 和心跳就是靠这些空帧传回来的，不发的话屏上
 * 永远停在最后一次的结论上，而那个结论可能是几秒前还对、现在早就断了。 */
static void SpiOneTransaction(void)
{
    uint32_t now = HAL_GetTick();
    uint16_t tx_len;
    uint16_t rx_len;
    uint16_t hb;

    /* ---- MOSI 头：[TX_LEN(2)][SQ(2)][RSV(4)] ----
     * ★ RSV 那 4 个字节**全程保持 0**，靠的是"从来没人写过它"（Init 里
     *   memset 过一次），而不是"每笔都清一遍"。C3 拿这 4 字节做结构校验：
     *   非 0 就说明这一笔捕获到的是错位/混合内容（旧帧尾 + 新帧头）。
     *   所以**往这里写任何东西都会破坏 C3 的判据** —— 要加字段先看 C3 那边。 */
    tx_len = TxPeek(&s_mosi[OTA_SPI_HDR_LEN], OTA_SPI_PAYLOAD_LEN);
    WrU16(&s_mosi[0], tx_len);
    s_sq++;
    WrU16(&s_mosi[2], s_sq);

    /* ---- 全双工收发 ---- */
    if (OTA_Spi_Transfer(s_mosi, s_miso, OTA_SPI_XFER_LEN) != 0u)
    {
        /* ★ 这一笔没成：**不动发送缓冲**，下一笔原样重发（SQ 会变新，
         *   C3 因此不会把它当成陈旧帧丢掉 —— 这就是"重发那一笔也要自增序号"
         *   的理由）。这里也不更新 s_last_act：实在发不出去时应当降档，
         *   而不是一直按 80% 占空空转。 */
        s_err_cnt++;
        return;
    }
    TxDrop(tx_len);
    if (tx_len > 0u)
    {
        s_tx_total += tx_len;
        s_last_act = now;
    }

    /* ---- MISO 头：[RX_LEN(2)][STATUS(2)][HB(2)][RSV(2)] ----
     * ★ 先做结构校验，过了才认后面三个字段 —— 见下面的说明。 */
    if ((s_miso[6] | s_miso[7]) != 0u)
    {
        /* ★★ MISO 的 RSV[6..7] 恒为 0（C3 在 BuildNextTx 里整块 memset 之后
         *   只写 [0..5]）。非 0 ⇒ 这一笔收到的是**错位/噪声**，不是一帧完好的头，
         *   那么 RX_LEN 同样不可信，绝不能拿它去喂 sink（那会把垃圾字节塞进
         *   帧状态机，而状态机只认 0xA5 0x5A，可能被一个假包头卡在半帧上，
         *   把后面真正的帧一起吞掉）。
         *   ★ 判据取两个字节全 0：随机数据误判概率 2^-16。比 4 字节弱一些，
         *     但 MISO 头只剩这 2 字节是"结构上必须为 0"的 —— MOSI 那边能用
         *     4 字节，是因为它的 RSV 有 4 字节而 SQ 只占了 2 个。
         *   ★ 这一条**对 C3 的固件有依赖**：C3 若往 s_tx[6..7] 里写了东西，
         *     这里会把每一笔都判成坏帧、全线失败。改 C3 时务必同步。 */
        s_bad_cnt++;
        return;
    }

    s_status = RdU16(&s_miso[2]);
    hb       = RdU16(&s_miso[4]);
    if (hb != s_hb)
    {
        s_hb      = hb;
        s_hb_tick = now;
    }
    /* ★ 心跳判活用的是**"变没变"**而不是"是多少"：它不仅会回绕，
     *   含义也由 C3 自己定义。只有"在涨"这件事是有意义的。 */

    rx_len = RdU16(&s_miso[0]);
    if (rx_len > OTA_SPI_PAYLOAD_LEN)
    {
        s_oob_cnt++;
        return;
    }
    if (rx_len > 0u)
    {
        s_rx_total += rx_len;
        (void)s_sink(&s_miso[OTA_SPI_HDR_LEN], rx_len);
        s_last_act = now;
    }
}

/* ==================== 通道层：对外接口 ==================== */

void OTA_Spi_Init(OTA_RxSink_t sink)
{
    s_sink = sink;

    /* 片选先立到空闲高，再谈别的 —— 否则从机可能被吊在"选中"状态上，
     * 上电瞬间就开始数时钟。 */
    OTA_SPI_CS_HIGH();

    memset(s_mosi, 0, sizeof(s_mosi));
    memset(s_miso, 0, sizeof(s_miso));

    s_txb_head = 0u;
    s_txb_tail = 0u;

    s_sq     = 0u;
    s_status = 0u;

    /* ★ 心跳初值取 0xFFFF 这个**哨兵**：C3 的心跳从 0 开始，第一笔事务就会
     *   得到 0，与 0xFFFF 不等 ⇒ 判定为"变了"、刷新 s_hb_tick。
     *   若初值也取 0，则第一笔事务不算"变化"，屏上会白挂 500ms 的
     *   "C3 NOT RESPONDING" —— 一上电就报假故障，正是最该避免的那类现象。 */
    s_hb      = 0xFFFFu;
    s_hb_tick = HAL_GetTick();

    /* 都取 0：档位判断按"还没活动过"落到**空闲档** —— 别一上电就抢 80% 的 CPU。
     * （`now - 0` 要 ≥ 8ms 才会开第一笔；工程里走到这里时 HAL_GetTick() 早就
     *   远超 8 —— Bootloader_Init 之前已经跑过 MX_LWIP_Init 的 PHY 复位延时和
     *   LCD_Init。所以"上电第一笔会晚 8ms"只是理论上的，不影响什么。） */
    s_last_poll = 0u;
    s_last_act  = 0u;

    s_rx_total = 0u;
    s_tx_total = 0u;
    s_bad_cnt  = 0u;
    s_oob_cnt  = 0u;
    s_err_cnt  = 0u;
}

void OTA_Spi_DeInit(void)
{
    /* 先放掉片选再拆 SPI：让从机在时钟停止之前就退出选中状态。 */
    OTA_SPI_CS_HIGH();

    /* ★ 汇点一并摘掉：拆完之后若还有人调 OTA_Spi_Poll()，
     *   它会在第一行就返回，而不是拿着已失效的 hspi2 去开事务。 */
    s_sink = 0;

    /* ★ 理由与 OTA_Net_DeInit() 里停 ETH DMA 相同：不能让外设带着已使能的
     *   状态交给 app。app 会重新 MX_SPI2_Init()，两套寄存器配置打架的结果
     *   是"偶发、且看起来跟 OTA 毫无关系"的故障。
     *   HAL_SPI_DeInit 内部会关 SPI2 时钟并把 PB13/PC2/PC3 复位成默认态；
     *   PB12 是我们的普通 GPIO，不在它的管辖范围内，保持输出高即可。 */
    (void)HAL_SPI_DeInit(&hspi2);
}

uint8_t OTA_Spi_Send(const uint8_t *buf, uint16_t len)
{
    uint16_t i;

    if (len == 0u)
    {
        return 0u;
    }
    if (buf == 0)
    {
        return 1u;
    }

    /* 留一格区分空/满 —— 与 Bootloader 那个环同一个约定，别改成"装满为止"：
     * 满了之后 TxCount() 与"环空"无法区分，会让"还有没有没发出去的字节"
     * 这个判断永远为假。 */
    if (((uint32_t)TxCount() + (uint32_t)len) > (uint32_t)(OTA_SPI_TX_BUF - 1u))
    {
        return 1u;                     /* 缓冲满：协议层不重发，由上位机超时重发兜底 */
    }

    for (i = 0u; i < len; i++)
    {
        s_txb[s_txb_head] = buf[i];
        s_txb_head = (uint16_t)((s_txb_head + 1u) & TXB_MASK);
    }
    return 0u;
}

void OTA_Spi_Poll(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t interval;

    if (s_sink == 0)
    {
        return;                        /* 未初始化 / 已拆除 */
    }

    interval = ((uint32_t)(now - s_last_act) < (uint32_t)OTA_SPI_ACTIVE_HOLD_MS)
             ? (uint32_t)OTA_SPI_ACTIVE_MS
             : (uint32_t)OTA_SPI_IDLE_MS;

    if ((uint32_t)(now - s_last_poll) < interval)
    {
        return;                        /* 还没到点：立刻返回，不阻塞调用方 */
    }

    /* ★ 记账放在事务**之前**，口径是"两笔的**起点**间隔"。
     *   放到事务之后的话，实际周期会变成 interval + 1.61ms（事务本身的耗时），
     *   相当于悄悄把吞吐砍掉一半 —— 而 OTA_SPI_ACTIVE_MS 那个 2ms 的余量是按
     *   "起点到起点"算的（见 ota_spi.h 里 20ms 那段推导）。 */
    s_last_poll = now;

    SpiOneTransaction();
}

void OTA_Spi_FlushTx(uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();
    uint8_t  fails = 0u;

    if (s_sink == 0)
    {
        return;
    }

    /* 发送缓冲空就直接返回 —— 绝大多数跳转（上电路径）都是这种情况，零开销。 */
    while (TxCount() > 0u)
    {
        uint32_t errs = s_err_cnt;

        s_last_poll = HAL_GetTick();
        SpiOneTransaction();

        if (s_err_cnt != errs)
        {
            /* ★★ 失败的这一笔要吃满 OTA_SPI_TIMEOUT_MS = 100ms 的 HAL 超时，
             *   比原来那个 50ms 的预算**还长** —— 于是"超时在每笔之前查"这个
             *   写法会让**单独一次**抖动就把整个预算耗光、当场放弃，
             *   而去重试的那一笔根本没机会开出来。
             *   这就是"升级成功、上位机却收不到最后应答"的成因之一：
             *   ACK 留在队列里被 OTA_Spi_DeInit() 连队列一起销毁，此后 F407
             *   已经在 app 里，上位机那 3 次 END 重发全是石沉大海。
             *   改成按**连续失败笔数**放弃：单次抖动只是重试一次，只有连续
             *   OTA_SPI_FLUSH_FAIL_MAX 笔都失败才认定链路坏了、不再空耗。 */
            fails++;
            if (fails >= OTA_SPI_FLUSH_FAIL_MAX)
            {
                break;
            }
        }
        else
        {
            fails = 0u;                 /* 只要有一笔成了就重新数 —— "时好时坏"要继续重试 */
        }

        /* 墙钟上限放在最后查：它的作用是给"链路一直在失败但没连续失败"这种
         * 病态情形兜底，而不是给每笔事务设闸。 */
        if ((uint32_t)(HAL_GetTick() - t0) >= timeout_ms)
        {
            break;
        }
    }
}

/* ==================== 屏上诊断 ==================== */
/* （原先这里有个 OTA_Spi_Connected()，返回值就是 MISO 的 STATUS 里 TCP 那一位。
 *   已删除：它没有任何调用点 —— 唯一的候选使用者 OTA_Spi_ShowDiag 需要的是
 *   **整个** s_status（WIFI 和 TCP 两位都要，才能区分"WiFi 断了"和"PC 没连"），
 *   只给一位反而不好用。留一个自称"供上层判断用"却没人用的接口，
 *   比没有更容易把人带偏；将来真要用，三行就能补回来。） */

/* 心跳多久没变就认为 C3 没在响应（ms）。
 * ★ 用**墙钟毫秒**而不是"事务笔数"：一笔记事务正常只要 1.61ms，但失败的
 *   那一笔要吃满 OTA_SPI_TIMEOUT_MS = 100ms 的 HAL 超时 —— 按笔数算的话
 *   最坏要几十笔才报警，屏上会一直挂着上一次（可能还是 LINK OK）的结论，
 *   正好在最需要它准确的时候骗人。按时刻算就没这问题。 */
#define SPI_DIAG_HB_STALL_MS   500u

/* 重画间隔（ms）。刷屏要写几十个字符，比一笔事务慢得多，每轮都刷会把
 * 轮询节奏彻底打乱 —— 而且主循环两个都是**没有延时**的紧循环。 */
#define SPI_DIAG_PERIOD_MS     100u

/* ★★ 四条结论**必须等宽**（都补空格到 17 个字符）。
 *   LCD_DisplayString_color 只写 strlen 个字符，末尾之外的格子一个都不动
 *   （字符**格子内部**前景背景都画，所以写空格就是擦除，但它不会帮你擦后面的）。
 *   四个串长度不同，由长变短时必然留残字。而
 *   "上电 → C3 还没挂上 SPI → C3 NOT RESPONDING → C3 起来 → WAIT PC →
 *    PC 连上 → LINK OK" 是最普通的开机路径，两次跳变都会踩到，
 *   屏上会读成 "LINK OKRESPONDING" 这种东西 —— 屏是本通道唯一的诊断通道，
 *   结论行读错等于这个诊断白做。 */
static uint32_t s_diag_tick = 0u;
static uint8_t  s_diag_first = 1u;

void OTA_Spi_ShowDiag(uint16_t x, uint16_t y)
{
    uint32_t now = HAL_GetTick();
    const char *verdict;
    uint16_t brush;
    uint32_t bad;

    /* 首帧强制画一次（屏刚被 LCD_Clear 过，缓存式跳过会留下一片空白） */
    if ((s_diag_first == 0u) &&
        ((uint32_t)(now - s_diag_tick) < (uint32_t)SPI_DIAG_PERIOD_MS))
    {
        return;
    }
    s_diag_first = 0u;
    s_diag_tick  = now;

    if ((uint32_t)(now - s_hb_tick) >= (uint32_t)SPI_DIAG_HB_STALL_MS)
    {
        verdict = "C3 NOT RESPONDING";              /* 连心跳都不涨：C3 没跑 / SPI 收不到它 */
        brush   = RED;
    }
    else if ((s_status & OTA_SPI_ST_WIFI) == 0u)
    {
        verdict = "WIFI DOWN        ";              /* SPI 通、C3 活着，是它自己没连上 WiFi */
        brush   = RED;
    }
    else if ((s_status & OTA_SPI_ST_TCP) == 0u)
    {
        verdict = "WAIT PC          ";              /* C3 就绪，只等上位机连上来 */
        brush   = WHITE;
    }
    else
    {
        verdict = "LINK OK          ";              /* 三段全通 */
        brush   = GREEN;
    }

    /* 第一行：通道名 + 结论。通道名每帧重画是刻意的 —— 它便宜，
     * 而"只画一次"会引入"屏被清过之后前缀不见了"的额外状态。 */
    LCD_DisplayString_color((u16)x, (u16)y, 16u, (u8 *)"SPI", brush, BLACK);
    LCD_DisplayString_color((u16)(x + 32u), (u16)y, 16u, (u8 *)verdict, brush, BLACK);

    /* 第二行：两个计数。RX = 从 C3 收到多少字节（下载进度），
     * E = 坏帧笔数（badhdr + 越界 + HAL 超时三类合起来）。
     * ★ 宽度固定、mode=0（左侧补空格），所以数字变长变短都不会留残字。
     * ★ RX 用 7 位够：F407 的 app 上限 128KB，正常一次升级远小于 9999999，
     *   不需要像步骤③ 那样做 KB 换算（那时是无限回显，几十秒就写满 7 位）。 */
    bad = s_bad_cnt + s_oob_cnt + s_err_cnt;

    /* ★ 第二行必须用带 _color 的版本（第一行也是）。不用的话走的是 lcd.c 的
     *   全局 BRUSH_COLOR/BACK_COLOR，而它俩的初值是**黑字白底**，LCD_Clear()
     *   也不会改它们 —— 结果就是在整屏黑底上横着一条白底黑字的色带，
     *   跟第一行格格不入，看上去像屏画坏了。 */
    LCD_DisplayString_color((u16)x, (u16)(y + 30u), 16u, (u8 *)"RX", WHITE, BLACK);
    LCD_DisplayNum_color((u16)(x + 32u), (u16)(y + 30u), s_rx_total, 7u, 16u, 0u, WHITE, BLACK);
    LCD_DisplayString_color((u16)(x + 96u), (u16)(y + 30u), 16u, (u8 *)"E", WHITE, BLACK);
    LCD_DisplayNum_color((u16)(x + 112u), (u16)(y + 30u), bad, 4u, 16u, 0u, WHITE, BLACK);
}
