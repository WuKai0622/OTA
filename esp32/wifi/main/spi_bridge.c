/* SPI 从机 ↔ TCP 桥 —— SPI 侧。
 *
 * ★ 与 tcp_bridge.c 一样，本文件**与内容完全无关**：它把 MOSI 上的字节原样
 *   投进环、把环里的字节原样摆上 MISO。步骤③ 搬的是回显载荷，步骤④ 搬的是
 *   OTA 帧 —— 对它没有任何区别。（唯一"懂内容"的动作是下面那两道完整性校验，
 *   而它们只看事务头，不看载荷。）
 *
 * 事务循环做三件事：
 *   1. 从 ToStmRing 取一段 PC 发来的字节，装进 MISO 载荷
 *   2. 跑一笔 264B 全双工事务（同时把 STM32 发来的字节收进 rx 缓冲）
 *   3. 把收到的字节投进 FromStmRing，交给 TCP 任务送回 PC
 */
#include "spi_bridge.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "driver/spi_slave.h"
#include "esp_attr.h"
#include "esp_log.h"

#include "tcp_bridge.h"
#include "wifi_link.h"

static const char *TAG = "spi_bridge";

/* 接线（与 F407 侧 ota_spi.h 一一对应，两块板必须共地）：
 *   F407 PB13 ──→ GPIO6   SCK
 *   F407 PC3  ──→ GPIO7   MOSI
 *   F407 PC2  ←── GPIO5   MISO
 *   F407 PB12 ──→ GPIO10  CS
 *
 * ★ MISO 为什么不用 GPIO2：那才是 SPI2(FSPI) 的 IO_MUX 默认脚，走 IO_MUX 更快。
 *   但 GPIO2 是 C3 的 strapping 脚，上电时必须为高，挂上 SPI 从机的输出有风险。
 *   所以改用 GPIO5，由 GPIO 矩阵路由 —— 代价可忽略，换来启动可靠性。 */
#define PIN_SCK     6
#define PIN_MOSI    7
#define PIN_MISO    5
#define PIN_CS      10

/* 事务长度必须是 4 的倍数，否则 DMA 下主机会被直接丢弃写入（ESP-IDF 通用约束）。
 * 264 = 4 × 66 ✓ —— 钉死在编译期，谁改坏了直接报错，而不是留到板子上变成
 * "偶发丢几个字节、偶发校验错、重发能过"的诡异 bug。 */
_Static_assert((SPI_BRIDGE_XFER_LEN % 4u) == 0u,
               "事务长度必须是 4 的倍数，否则 DMA 下主机的写入会被丢弃");
_Static_assert(SPI_BRIDGE_XFER_LEN == 264u, "264B 是与 F407 的契约，改了两边都得改");

/* ★ DMA_ATTR 而不只是 WORD_ALIGNED_ATTR：前者语义正确。C3 没有 PSRAM，
 *   当前两者实际等价，但将来换带 PSRAM 的模组时只有 DMA_ATTR 仍然安全。 */
static DMA_ATTR uint8_t s_tx[SPI_BRIDGE_XFER_LEN];
static DMA_ATTR uint8_t s_rx[SPI_BRIDGE_XFER_LEN];

static RingbufHandle_t s_rb_to_stm;
static RingbufHandle_t s_rb_from_stm;

/* ---- 会话边界屏障（见 Spi_Bridge_FlushRings）----
 * s_flush_req  : TCP 任务置位，SPI 任务在事务里消费
 * s_flush_done : SPI 任务清干净后放行，TCP 任务在屏障上等它
 * 为什么要这么绕：要清掉的那一块**还没到 C3**，它此刻正躺在 F407 的 `pending`
 * 里，只能靠跑完一笔事务才能把它勾出来。所以清理动作必须在 **SPI 任务**里做。 */
static volatile uint8_t  s_flush_req = 0u;
static SemaphoreHandle_t s_flush_done;

static void FlushRing(RingbufHandle_t rb);

/* 短事务（被截断/零位）累计笔数，只用来给日志限流，见 spi_bridge_task 里那一支。 */
static uint32_t s_short_cnt = 0u;

/* ---- 帧新鲜度校验（见 spi_bridge_task 里那一支，以及 ota_spi.c 的 TX_SQ）----
 *
 * ★★ 为什么需要它：原先只靠驱动给的 `trans_len` 判断"这一笔有没有捕获到东西"，
 *   实测**不成立** —— 这一笔完全没有时钟进来时，trans_len 会保留上一笔的值，
 *   而 s_rx 也原封不动仍是上一帧。于是长度字段看着完全合法，同一块载荷被
 *   **又投递了一遍**，PC 收到双份数据而两端计数器都不报错。
 *   现在 F407 在 MOSI 头的 RSV 前 2 字节放一个每笔自增的序号，一比即可判定。 */
static uint16_t s_last_sq  = 0u;
static uint8_t  s_have_sq  = 0u;    /* 首笔之前没有"上一笔"可比，必须先放行一次 */
static uint32_t s_stale_cnt = 0u;   /* 序号未变 ⇒ 捕获失败（陈旧帧） */

/* ---- 对端固件不匹配的兜底（见下面 s_same_run 的判定）----
 * ★ 新 C3 配**旧 F407** 时，F407 的 RSV 恒为 0 ⇒ "序号没变"永远成立 ⇒ 所有帧
 *   都被当陈旧丢掉，表现是"全线失败"。故障现象和真正的 SPI 坏了长得一模一样，
 *   能把人引到接线上去查半天。
 *   判据：**连续** 64 笔序号完全相同 —— 真 F407 每笔自增，只有整条链路连丢 64 笔
 *   （≈128ms 全哑）才可能重现；那种情况下本来也没有东西可投递，关掉检查无损失。
 *   一旦命中就明确宣告是固件不匹配，而不是静默降级。 */
#define SPI_BRIDGE_SQ_MISMATCH_RUN  64u
static uint32_t s_same_run   = 0u;
static uint8_t  s_sq_check_on = 1u;

/* TX_LEN 越界次数（帧被错位/打坏），与短事务、陈旧帧一起在断连时汇总打印。 */
static uint32_t s_oob_cnt = 0u;

/* 头部结构校验不通过的笔数（见 spi_bridge_task 里那一支）。
 * ★ 与 s_stale_cnt 是**两种不同的坏法**，别混：
 *     stale   = 整帧没换（序号相同）—— 序列号能抓
 *     bad_hdr = 帧换了但**错位/混合**（旧帧尾 + 新帧头）—— 序号抓不住，只能靠结构
 *   实测（2026-09-19）就是后者在漏：`头=B2 B9 C0 C7 CE D5 DC E3`（全是差 7 的
 *   载荷字节）而 `trans_len=2112bit` 报的是**满长度**，长度检查完全被骗过去。 */
static uint32_t s_bad_hdr_cnt = 0u;

/* from_stm 环满丢弃的字节数。**接上真 OTA 之后丢掉的才有兜底**
 * （上位机逐帧 stop-and-wait，超时会重发那一帧），所以它不再是致命错误；
 * 但它仍然说明消费端（TCP）没跟上、速度上不去 —— 该查，而不是该忽略。 */
static uint32_t s_drop_cnt = 0u;

/* F407→C3 方向的**内容** dump。
 *
 * ★ 为什么需要它：PC 侧只看得见"收到的前 32B"，F407 屏上只看得见 RX 这个**数量**，
 *   中间没有任何地方能看见 F407 实际放上总线的字节。一旦"发了多少"和"收了多少"
 *   对不上，就只能靠猜 —— 而这一串 dump 正好卡在**F407 发出**与**PC 收到**之间，
 *   把"F407 回声逻辑错"和"C3 转发出错"这两件事一刀切开：
 *     这里对得上、PC 收到的却是别的 ⇒ 问题在 C3 的 TCP 发送路径；
 *     这里本身就不对               ⇒ 问题在 F407 的回声逻辑。
 *
 * ★ 限流：第 1~8 笔全打（诊断最关心开头），之后每 128 笔打一次，且每次都带累计值。
 *   不加限流的话，F407 持续回显时这一支会以几百行/秒刷屏，把 WiFi 任务一起拖慢
 *   （同一个理由见短事务那一支）。排查完把宏改成 0 即可彻底关掉。 */
#define SPI_BRIDGE_DUMP_FROM_STM    1

/* ★ 累计值是**记账**，不是日志 —— 断连时的汇总行（Spi_Bridge_GetStats）也要用它，
 *   所以放在宏外面。宏只决定"打不打日志"，不该决定"记不记账"。 */
static uint32_t s_rx_total = 0u;    /* 从 F407 收到的总有效字节数 */

#if SPI_BRIDGE_DUMP_FROM_STM
static uint32_t s_rx_cnt   = 0u;    /* rx_len > 0 的笔数 */
#endif

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

/* 备好下一笔要发的 MISO 内容：从环里取一段载荷，再填 8 字节头。 */
static void BuildNextTx(uint16_t hb)
{
    size_t   len = 0u;
    uint8_t *item;
    uint16_t status = 0u;

    memset(s_tx, 0, sizeof(s_tx));

    /* 0 等待：环空就发空帧（长度 0）。事务**必须照发** —— 从机不发，
     * 主机就什么都收不到，STATUS 和心跳也就传不过去了。
     * ★ BYTEBUF 的隐式约束：上一次取出的 item 未归还前再取会返回 NULL
     *   （ringbuf.c 的 prvCheckItemAvail，静默的"环空"而非报错）。
     *   所以本函数只能是"取一笔 → 拷走 → 立刻还"，别攒着多取几次。 */
    item = (uint8_t *)xRingbufferReceiveUpTo(s_rb_to_stm, &len, 0, SPI_BRIDGE_PAYLOAD_LEN);
    if (item != NULL)
    {
        memcpy(&s_tx[SPI_BRIDGE_HDR_LEN], item, len);

        /* ★ 拷完立刻归还。ReceiveUpTo 给的是**环内指针**（不拷贝），早点还
         *   就能早点给 TCP 任务腾出环空间 —— 背压靠的正是"环里还剩多少空位"。
         *   （方案原文写的是"事务成功后才归还"，那是按零拷贝设计的；这里既然
         *     已经拷进 s_tx 了，就没有理由再攥着环不放。） */
        vRingbufferReturnItem(s_rb_to_stm, item);
    }

    WrU16(&s_tx[0], (uint16_t)len);                  /* RX_LEN（F407 视角） */

    if (Wifi_Link_Ready() != 0u)      { status |= SPI_BRIDGE_ST_WIFI; }
    if (Tcp_Bridge_HasClient() != 0u) { status |= SPI_BRIDGE_ST_TCP;  }
    WrU16(&s_tx[2], status);
    WrU16(&s_tx[4], hb);
}

static void spi_bridge_task(void *arg)
{
    uint16_t hb = 0u;

    (void)arg;

    BuildNextTx(hb);                      /* 第一笔之前先把 s_tx 备好 */

    for (;;)
    {
        spi_slave_transaction_t t;
        esp_err_t r;
        uint16_t  rx_len;
        uint16_t  sq;

        memset(&t, 0, sizeof(t));
        t.length    = SPI_BRIDGE_XFER_LEN * 8;   /* ★ 单位是 bit，不是字节 */
        t.tx_buffer = s_tx;
        t.rx_buffer = s_rx;

        /* portMAX_DELAY：主机不来就安静等着，不空转。
         * F407 擦 Flash 时会停轮询 1~4 秒，此处正好把 CPU 全让给 WiFi 任务。 */
        r = spi_slave_transmit(SPI2_HOST, &t, portMAX_DELAY);
        if (r != ESP_OK)
        {
            /* ★ 退避重试，并且**不重建 s_tx** —— 这一笔没发出去，下一轮原样重发。
             *   不退避的话，持续性故障（接线/供电）会变成一个疯狂打日志的紧循环，
             *   日志本身走 UART，会把 WiFi 任务一起拖慢，最后连"为什么连不上"
             *   都看不出来。 */
            ESP_LOGW(TAG, "spi_slave_transmit 失败: %s", esp_err_to_name(r));
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        /* ---- 事务完成，s_rx 里是 F407 刚发来的内容 ---- */

        /* ★★ 必须查 trans_len，不能只看返回值。
         *   spi_slave_transmit 只在**驱动级**出错时返回非 ESP_OK —— 主机少打
         *   时钟（或只给一个 CS 边沿、根本没敲时钟），事务会带着 ESP_OK
         *   **正常返回**，实际收了多少位在 t.trans_len 里（由 HAL 的
         *   get_rcv_bitlen 填，单位 bit）。不看它就出事：
         *     - 事务被截断 → s_rx 是"上一笔尾部 + 本笔前段"的混合，头 2 字节
         *       可能还是**上一笔**的 TX_LEN（合法值 0..256）；
         *     - 事务为 0 位（CS 毛刺 / F407 复位瞬间）→ s_rx **整块**都是上一笔。
         *   两种都会解析出一个"看起来合法"的 rx_len，把最多 256B 陈旧字节塞进
         *   FromStmRing 送给 PC —— 重复或错乱的字节，而两端任何计数器都不报错。
         *   步骤③ 是裸字节流，只能靠肉眼比对发现；到步骤④ 就是"偶发校验错、
         *   重发能过"那种极难查的故障。按"漏一笔"处理：不解析、不投递。 */
        if (t.trans_len != (size_t)SPI_BRIDGE_XFER_LEN * 8u)
        {
            /* ★ 日志必须限流：这一支的节奏由**主机**决定，SCK 断线/错接这类故障
             *   会让每一笔都是 0 bit ⇒ 约 278 行/秒 × ~70B ≈ 19KB/s，超过
             *   115200 串口的容量（≈11.5KB/s）。而日志走 UART 会拖慢 WiFi 任务
             *   —— 那正是下面 `r != ESP_OK` 那支要退避的同一个理由，在这里以
             *   另一种形式复现。只打第 1 笔和每 256 笔，把总数带在日志里。
             *   （**不加退避**：这一支要保持在位等主机的下一次 CS 边沿，
             *     退避反而会白丢事务；是限流，不是退避。） */
            s_short_cnt++;
            if ((s_short_cnt == 1u) || ((s_short_cnt % 256u) == 0u))
            {
                ESP_LOGW(TAG, "短事务 %u bit（应为 %u），累计 %u 笔，按漏一笔处理",
                         (unsigned)t.trans_len, (unsigned)(SPI_BRIDGE_XFER_LEN * 8u),
                         (unsigned)s_short_cnt);
            }
            hb++;
            BuildNextTx(hb);
            continue;
        }

        /* ★★ 帧新鲜度校验：序号没变 ⇒ 这一笔什么都没捕获到，s_rx 还是上一帧。
         *   必须在这里就拦下来，**不能**往下走：
         *     - 往下走会把上一帧的载荷**再投递一遍**（PC 收到双份）；
         *     - 若错位恰好落在 F407 头的零填充区，还会解析出"TX_LEN=0"这个
         *       看着完全合法的值 —— 那会让会话边界屏障误判"F407 已经吐干净了"
         *       而提前放行（这正是屏障修完反而更差的原因）。
         *   和短事务一样按"漏一笔"处理：不解析、不投递。 */
        sq = RdU16(&s_rx[2]);
        if ((s_have_sq != 0u) && (sq == s_last_sq))
        {
            /* 先分辨"真陈旧"还是"对端固件根本不带序号"，见 SPI_BRIDGE_SQ_MISMATCH_RUN */
            s_same_run++;
            if ((s_sq_check_on != 0u) && (s_same_run >= SPI_BRIDGE_SQ_MISMATCH_RUN))
            {
                s_sq_check_on = 0u;
                ESP_LOGE(TAG, "连续 %u 笔序号都是 %u ⇒ 对端没有帧序号（F407 烧的还是旧固件？）"
                              "帧新鲜度校验已关闭，请重烧 F407",
                         (unsigned)s_same_run, (unsigned)sq);
            }

            if (s_sq_check_on != 0u)
            {
                s_stale_cnt++;
                if ((s_stale_cnt == 1u) || ((s_stale_cnt % 256u) == 0u))
                {
                    ESP_LOGW(TAG, "陈旧帧（序号仍是 %u），累计 %u 笔，按漏一笔处理",
                             (unsigned)sq, (unsigned)s_stale_cnt);
                }
                hb++;
                BuildNextTx(hb);
                continue;
            }
            /* 校验已关闭：当作正常帧往下走（此时对端是旧固件，行为与改动前一致） */
        }
        else
        {
            s_same_run = 0u;
        }
        s_last_sq = sq;
        s_have_sq = 1u;

        /* ★★ 头部结构校验：[4..7]（RSV 后 4 字节）**恒为 0**。
         *   F407 在 echo 测试入口 memset 一次之后再没写过这 4 个字节（[ota_spi.c:508]），
         *   所以只要是"对齐的、当场发出的"那一帧，这里必然全 0。
         *   非 0 就说明这一笔捕获到的是**错位/混合**内容（旧帧尾 + 新帧头）——
         *   这时 s_rx 整块都是别人的载荷字节，长度字段可能恰好是个 ≤256 的合法值
         *   （实测漏过 84 B 的残留就是这么来的），序号校验也抓不住（序号变了）。
         *   判据取 4 字节全 0：随机数据误判概率 2^-32，可以当零。
         *   ★ 这一条对**新旧 F407 都成立**（旧固件这 4 字节同样是 memset 后的 0），
         *     所以它不额外增加"必须两边一起重烧"的约束。 */
        if ((s_rx[4] | s_rx[5] | s_rx[6] | s_rx[7]) != 0u)
        {
            s_bad_hdr_cnt++;
            if ((s_bad_hdr_cnt <= 8u) || ((s_bad_hdr_cnt % 256u) == 0u))
            {
                ESP_LOGW(TAG, "帧错位（第 %u 笔）：头=%02X %02X %02X %02X %02X %02X %02X %02X，trans_len=%u bit",
                         (unsigned)s_bad_hdr_cnt,
                         s_rx[0], s_rx[1], s_rx[2], s_rx[3],
                         s_rx[4], s_rx[5], s_rx[6], s_rx[7],
                         (unsigned)t.trans_len);
            }
            hb++;
            BuildNextTx(hb);
            continue;
        }

        rx_len = RdU16(&s_rx[0]);
        if (rx_len > SPI_BRIDGE_PAYLOAD_LEN)
        {
            /* 长度字段被噪声打坏时不能让后面越界读 —— 夹到上限并记一笔。
             * ★ 把整条 8 字节头和 trans_len 一起打出来：这一支是"帧被错位"
             *   的直接证据，光看一个越界的数字分不清是错位、重复还是噪声。 */
            s_oob_cnt++;
            ESP_LOGW(TAG, "TX_LEN=%u 越界（第 %u 次），头=%02X %02X %02X %02X %02X %02X %02X %02X，trans_len=%u bit",
                     (unsigned)rx_len, (unsigned)s_oob_cnt,
                     s_rx[0], s_rx[1], s_rx[2], s_rx[3],
                     s_rx[4], s_rx[5], s_rx[6], s_rx[7],
                     (unsigned)t.trans_len);
            rx_len = SPI_BRIDGE_PAYLOAD_LEN;
        }

        if (rx_len > 0u)
        {
            s_rx_total += rx_len;       /* 记账（宏外），见上方说明 */

#if SPI_BRIDGE_DUMP_FROM_STM
            /* 把 F407 这一笔实际发来的字节打出来，见上方宏的说明。 */
            s_rx_cnt++;
            if ((s_rx_cnt <= 8u) || ((s_rx_cnt % 128u) == 0u))
            {
                char     hex[3u * 16u + 1u];
                uint16_t n = (rx_len < 16u) ? rx_len : 16u;
                uint16_t i;

                for (i = 0u; i < n; i++)
                {
                    (void)snprintf(&hex[i * 3u], 4u, "%02X ",
                                   s_rx[SPI_BRIDGE_HDR_LEN + i]);
                }
                hex[(n * 3u) - 1u] = '\0';      /* 去掉末尾空格 */

                ESP_LOGI(TAG, "F407→C3 第 %u 笔 %u B（累计 %u B），前 %u B: %s",
                         (unsigned)s_rx_cnt, (unsigned)rx_len,
                         (unsigned)s_rx_total, (unsigned)n, hex);
            }
#endif
            /* 环满就丢（0 等待）。SPI 的字节已经被时钟打进来了，无法拒收 ——
             * 这是 UART 模型，不是 TCP 模型。
             * ★ 丢掉的**没有兜底**：步骤③ 是裸字节流，没有帧级重传；上位机的
             *   "超时重发"是帧级的，对字节流无从下手。步骤④ 接上逐帧 ACK 之后
             *   才真正有兜底。（实际不会到这里：PC 逐帧确认，同一时刻最多 1 帧
             *   在路上约 1036B，对 4KB 环来说远远够。） */
            if (xRingbufferSend(s_rb_from_stm, &s_rx[SPI_BRIDGE_HDR_LEN], rx_len, 0) != pdTRUE)
            {
                s_drop_cnt += rx_len;
                ESP_LOGW(TAG, "from_stm 环满，丢弃 %u B（累计 %u B）",
                         (unsigned)rx_len, (unsigned)s_drop_cnt);
            }
        }

        /* 这一笔的使命完成，备下一笔 */
        hb++;
        BuildNextTx(hb);

        /* ---- 会话边界屏障：清到 F407 亲口说"我一笔都没发"为止 ----
         * ★★ 为什么**一笔不够**（这里是最容易想漏的地方）：
         *   置位那一刻，F407 手里可能攒着**两块**东西 ——
         *     ① 上一会话残留在它 `pending` 里的那一块；
         *     ② 本笔 s_tx 若是在 flush 之前就 BuildNextTx 好的，它还会把这
         *        块旧数据再喂进去，于是 pending 又被填上。
         *   所以必须**每笔都清**，直到某一笔 rx_len == 0 —— 那才说明 F407 的
         *   pending 真的清零了、且上一会话的残渣已经全部吐完并被丢弃。
         *   实测：这一步漏掉的话，新 PC 会先收到**一块 256B 的上会话残留**
         *   （`STM→PC` 比 `PC→STM` 正好多 264 字节协议里的一块），
         *   而步骤③ 是裸字节流，PC 只能把它当成自己的回显去比对。
         *   事务每 2ms 一笔，最多三笔就清完，对连接建立时间无影响。 */
        if (s_flush_req != 0u)
        {
            FlushRing(s_rb_from_stm);
            if (rx_len == 0u)
            {
                s_flush_req = 0u;
                (void)xSemaphoreGive(s_flush_done);
            }
        }

        /* ★ 这里**故意不延时**。从机越早重新挂回 spi_slave_transmit，主机打进来
         *   时落在"未挂载窗口"里丢掉一笔的概率就越低 —— 那个窗口是 ESP-IDF 从机
         *   的固有特性（事务入队后由 ISR 真正挂载），消除不了，只能尽量压小。
         *   节流是**主机侧**的事（F407 的 OTA_SPI_POLL_MS），从机只负责尽快就位。 */
    }
}

void Spi_Bridge_Start(void)
{
    spi_bus_config_t buscfg = {
        .mosi_io_num   = PIN_MOSI,
        .miso_io_num   = PIN_MISO,
        .sclk_io_num   = PIN_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    spi_slave_interface_config_t slvcfg = {
        .mode         = 0,              /* Mode 0：CPOL=0 / CPHA=0，与 F407 对齐 */
        .spics_io_num = PIN_CS,         /* 硬件管片选：F407 拉低即开始一笔事务 */
        .queue_size   = 1,              /* 单笔同步模型：处理完一笔才挂下一笔 */
        .flags        = 0,              /* 0 = MSB first，与 F407 的 SPI_FIRSTBIT_MSB 一致 */
    };

    /* 先建环，再起任务 —— TCP 任务也要用这两个环 */
    s_rb_to_stm   = xRingbufferCreate(SPI_BRIDGE_RING_SIZE, RINGBUF_TYPE_BYTEBUF);
    s_rb_from_stm = xRingbufferCreate(SPI_BRIDGE_RING_SIZE, RINGBUF_TYPE_BYTEBUF);
    if ((s_rb_to_stm == NULL) || (s_rb_from_stm == NULL))
    {
        ESP_LOGE(TAG, "环形缓冲创建失败（各 %u B）—— 内存不够",
                 (unsigned)SPI_BRIDGE_RING_SIZE);
        abort();                        /* 没有环就没有桥，继续跑没有意义 */
    }

    /* 会话边界屏障用的信号量：二进制、初值为空（TCP 任务必须先等到 SPI 任务放行） */
    s_flush_done = xSemaphoreCreateBinary();
    if (s_flush_done == NULL)
    {
        ESP_LOGE(TAG, "屏障信号量创建失败 —— 内存不够");
        abort();
    }

    /* SPI_DMA_CH_AUTO：DMA 通道交给驱动挑。
     * ★ 走 DMA 就意味着"每笔长度必须是 4 的倍数"这条约束生效，见上方静态断言。 */
    ESP_ERROR_CHECK(spi_slave_initialize(SPI2_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO));

    memset(s_tx, 0, sizeof(s_tx));
    memset(s_rx, 0, sizeof(s_rx));

    /* 栈 4096 够用：本任务没有大局部变量（收发缓冲都是 static 的）。
     * 优先级 5 低于 WiFi 任务（23）—— 从机是被动的，不该跟 WiFi 抢 CPU。
     * 代价是冷启动 WiFi 起 AP/DHCP 时会漏掉几笔事务，只损失紧随其后的一笔且会自愈。 */
    xTaskCreate(spi_bridge_task, "spi_bridge", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "SPI 桥已启动：Mode0 CS=IO%d SCK=IO%d MOSI=IO%d MISO=IO%d 每笔 %u B",
             PIN_CS, PIN_SCK, PIN_MOSI, PIN_MISO, (unsigned)SPI_BRIDGE_XFER_LEN);
}

RingbufHandle_t Spi_Bridge_ToStmRing(void)
{
    return s_rb_to_stm;
}

RingbufHandle_t Spi_Bridge_FromStmRing(void)
{
    return s_rb_from_stm;
}

void Spi_Bridge_GetStats(Spi_Bridge_Stats_t *out)
{
    if (out == NULL)
    {
        return;
    }
    out->short_cnt   = s_short_cnt;
    out->stale_cnt   = s_stale_cnt;
    out->bad_hdr_cnt = s_bad_hdr_cnt;
    out->oob_cnt     = s_oob_cnt;
    out->drop_cnt    = s_drop_cnt;
    out->rx_total    = s_rx_total;
}

/* 把环里已有的字节全部丢弃（逐个 item 取出再归还，不做拷贝）。 */
static void FlushRing(RingbufHandle_t rb)
{
    for (;;)
    {
        size_t   len = 0u;
        uint8_t *item = (uint8_t *)xRingbufferReceiveUpTo(rb, &len, 0, SPI_BRIDGE_PAYLOAD_LEN);

        if (item == NULL)
        {
            return;                     /* 空了 */
        }
        vRingbufferReturnItem(rb, item);
    }
}

void Spi_Bridge_FlushRings(void)
{
    /* ★★ 为什么必须有这一步（步骤③ 实测踩到）：
     *   两个环都是**进程级静态**的，跨 TCP 连接一直活着。上一轮 PC 没读完
     *   （或读得慢、被 send_all 背压顶住）时留在 FromStmRing 里的回显，
     *   在新 PC 一连上就会**先于本会话的数据**被 pump_to_pc 倒出去。
     *   而步骤③ 是裸字节流 —— 新旧字节在内容上没有任何区别，上位机只能
     *   把它们当成自己的回显去比对，于是新会话开头若干字节**必然是错的**。
     *
     *   实测症状（很有欺骗性）：PC 每次收都是上一轮用例4 的那 48 字节
     *   "PKT000..PKT007" 的循环，看着像"字节被改坏了"或"SPI 错位"，
     *   但 F407 屏上 err 不涨、RX 在涨 —— 链路完全健康，纯粹是陈旧积压。
     *
     * ★★ 光清 C3 的环**不够**（第二版实测踩到，4418 发出了 4674，多一块 256B）：
     *   还有一块残留**不在 C3**，而在 **F407 的 `pending`** 里 —— 上一会话最后
     *   那一块它已经收下、正等着下一笔发出去。清空 ToStmRing 只能让 C3 在下一笔
     *   说"没数据了"（RX_LEN=0），F407 据此把 pending 清 0 **不再重复**；
     *   但**它手里已经攒着的那一块，仍会在那一笔被原样发出来**。
     *   ⇒ 所以真正的清理要**等它吐完**，而那只能跑事务才知道 —— 见 SPI 任务里
     *     那段"会话边界屏障"。**修正一句曾经写错的话：F407 侧确实不需要改代码，
     *     但需要 C3 等它一拍，不能清完自己就走。**
     *
     * ★ 步骤④ 更要紧：不清的话，上一会话残留的**半个帧**会当成这一会话的
     *   第一个帧去校验，表现为"每次重连后第一帧必然校验错/超时"。 */
    FlushRing(s_rb_to_stm);         /* 先让下一笔 BuildNextTx 发出 RX_LEN=0 */
    (void)xSemaphoreTake(s_flush_done, 0);   /* 丢掉可能残留的信号，保证这次真的等 */

    /* ★ 计数**按会话清零**。累计值看着像"趋势信息"，实际没有可操作性：
     *   断连那行真正要回答的是"**本会话** F407 实发了多少、坏了多少笔"，
     *   只有和同一行的 `PC→STM` / `STM→PC` 口径一致才能对账
     *   （上一版打的是跨会话累计，实测 10182 对 4502，根本没法比）。
     *   会话内的限流日志仍会带"本会话第几笔"，一路看下来并不丢信息。
     *   s_last_sq / s_have_sq **不清**：F407 的序号跨会话连续涨，
     *   清了反而要多放行一帧、白丢一次新鲜度判断。 */
    s_short_cnt   = 0u;
    s_stale_cnt   = 0u;
    s_bad_hdr_cnt = 0u;
    s_oob_cnt     = 0u;
    s_drop_cnt    = 0u;
    s_rx_total    = 0u;

    s_flush_req = 1u;

    /* ★ 必须**阻塞**等到 SPI 任务清干净再返回：返回之后 TCP 任务立刻开始
     *   pump_to_pc，此时若那块残留还在路上，就会被当成新会话的数据发给 PC ——
     *   于是又回到"开头若干字节必错"。这一步是屏障，不是尽力而为。
     *   超时只是兜底（SPI 断线时事务走短事务分支，永远不会报 rx_len==0），
     *   500ms ≈ 250 笔事务，正常情况两三笔（约 6ms）就放行了。 */
    if (xSemaphoreTake(s_flush_done, pdMS_TO_TICKS(500)) != pdTRUE)
    {
        s_flush_req = 0u;
        ESP_LOGW(TAG, "会话边界屏障超时（500ms）—— F407 没在应答？按清完处理继续");
    }
}
