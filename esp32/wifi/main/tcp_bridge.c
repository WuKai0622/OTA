/* SPI 从机 ↔ TCP 桥 —— TCP 侧。
 *
 * ★ 本文件**与内容完全无关**：它只搬字节，不知道搬的是回显载荷（步骤③）
 *   还是 OTA 帧（步骤④）。接真 OTA 时 F407 那头换了一整套，这里一行不用改 ——
 *   这正是当时坚持"先把透传单独验通"的收益。
 *
 * 一个任务同时照顾两个方向：
 *   PC → 环 ToStmRing → (SPI 任务) → MISO → STM32
 *   STM32 → MOSI → (SPI 任务) → 环 FromStmRing → PC
 *
 * 结构上刻意与 boot 侧 ota_net.c 对齐：做**服务器**、**单连接模型**、关 Nagle。
 * ⚠ 单连接的实际行为是"**排队**"不是"顶掉"：旧连接还活着时，新连接握手能成
 *   （进 backlog）但要等旧的先结束才被服务。这正是必须开 keepalive 的原因，
 *   否则一个静默消失的 PC 会把桥卡到天荒地老（详见 serve_client 里那段）。
 */
#include "tcp_bridge.h"

#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "lwip/sockets.h"

#include "spi_bridge.h"

static const char *TAG = "tcp_bridge";

#define RXBUF_SIZE      1460u       /* 单个 TCP 段最大约 1460B，按这个开就够 */
#define LISTEN_BACKLOG  1           /* 单连接模型，backlog 给 1 即可 */

/* recv 的超时：没数据就返回，好让循环转去照顾反方向。
 *
 * ★★ 必须 ≥ **一个 tick**，两个方向都不能违反，下面那条 #error 会盯着：
 *   ① 太小（但非 0）会被**整除成 0**：lwIP 的 FreeRTOS port 里是
 *        `timeout_ticks = timeout / portTICK_PERIOD_MS`   (port/freertos/sys_arch.c)
 *      而本工程 CONFIG_FREERTOS_HZ = 100 ⇒ 一个 tick = 10ms，2/10 = 0；
 *      `xQueueReceive(..., 0)` 是**不阻塞** —— recv 会每次立刻返回 EWOULDBLOCK。
 *      表面看只是"没睡成"，实际是把整个 serve_client 循环变成**纯自旋**：
 *      两个方向就都没有阻塞点了，于是
 *        - idle 任务（优先级 0）永远拿不到 CPU → 每 5 秒刷一条
 *          "IDLE0 未喂狗"的任务看门狗告警（长得像 WiFi/系统问题，其实是这里）；
 *        - 同级的 spi_bridge 只能等 10ms 时间片才被调度 → F407 白打约 5 笔。
 *      ★ 注意这条错**不会**报任何错、也不会崩，它只是安静地让一切变慢、变可疑。
 *   ② 等于 0 更糟：lwIP 里那表示"**永不超时**"（api_lib.c 把 0 直接当无限等待
 *      传给 sys_arch_mbox_fetch），两个方向就永远互相饿死。
 *
 *   好消息：recv 是靠邮箱被 post 唤醒的，数据一到立刻返回 —— 所以"超时 10ms"
 *   **不会**给数据本身加 10ms 延迟，它只决定"多久去照顾一次反方向"。
 *
 *   想要更细的粒度就调 CONFIG_FREERTOS_HZ（改成 1000 ⇒ 2ms 正好 2 tick），
 *   但那是全局改动；**别只改这个宏**。 */
#define RX_TIMEOUT_MS   10
#if RX_TIMEOUT_MS < (1000 / CONFIG_FREERTOS_HZ)
#error "RX_TIMEOUT_MS 不足一个 tick：lwIP port 的 ms→tick 是整除，商为 0 时 recv 退化成非阻塞轮询（纯自旋）"
#endif
/* send 的超时。
 * ★ 它的**数值对行为没有任何影响**：只要非 0，lwIP 就把写操作置成 dontblock
 *   （api_lib.c 的 `if (conn->send_timeout != 0) { dontblock = 1; }`），
 *   发送缓冲一满立刻返回 EWOULDBLOCK，根本不会等这 100ms。
 *   换句话说它只起"打开 dontblock"的开关作用 —— 现在真正决定
 *   "僵死对端何时被判死"的是 send_all 的重试策略（见那里）。
 *   别再以为把它改小就能更早发现卡死的对端。 */
#define TX_TIMEOUT_MS   100

static volatile uint8_t s_has_client = 0u;

/* 本会话两个方向的字节计数，进会话时清零、断开时打一行。
 *
 * ★ 存在的唯一理由：把"**PC 的数据根本没到 C3**"和"到了 C3 却没送到 F407"
 *   这两种故障一刀切开。步骤③ 实测时 PC 只看得到"没收到回显"，F407 屏上
 *   只看得到"RX 不涨"，两边都无法区分上面这两种情况 —— 而这一个数字可以：
 *     PC→STM = 0        ⇒ 字节压根没进 C3（TCP 侧/连接没被服务）
 *     PC→STM > 0 而 RX 不涨 ⇒ 字节进了 C3 但没上 SPI（环/SPI 任务）
 */
static volatile uint32_t s_down_bytes = 0u;     /* PC → STM */
static volatile uint32_t s_up_bytes   = 0u;     /* STM → PC */

uint8_t Tcp_Bridge_HasClient(void)
{
    return s_has_client;
}

/* 重试多久算"僵死"（tick）。
 * ★ 存在理由见 send_all 里那段：keepalive 抓不到"对端内核好好活着、只是应用层
 *   不读"，而步骤④ 下这种僵持是**致命**的（下载方向被顶住 ⇒ pump_from_pc 不再
 *   被调用 ⇒ ACK 也停 ⇒ 整个 OTA 卡死，且 s_has_client 恒为 1、新连接只能排队
 *   没人服务）。5 秒 ≈ 500 tick（HZ=100）。
 * 判据取"**sent 连续没增长**"而不是"总共花了多久"：对端读得慢但一直在读时
 * 就是要一直等下去，那不是故障。 */
#define TX_STALL_TICKS  pdMS_TO_TICKS(5000)

/* 把一次要发的 n 字节**完整**发出去。
 * ★ 必须循环：send() 在 TCP 上只保证"发出去了前一部分"，返回值可能 < n。
 *   只调一次就当作发完是经典的丢数据 bug。返回 0=成功，-1=对端断了/僵死。 */
static int send_all(int fd, const uint8_t *buf, int n)
{
    int sent = 0;
    TickType_t last_progress = xTaskGetTickCount();

    while (sent < n)
    {
        int w = send(fd, buf + sent, (size_t)(n - sent), 0);

        if (w > 0)
        {
            sent += w;
            last_progress = xTaskGetTickCount();
            continue;
        }

        /* ★★ 发送缓冲满（EWOULDBLOCK）是**常规可恢复事件**，绝不能当断开。
         *
         *   这一点在 ESP-IDF 的 lwIP 里不直观，必须看源码才明白：
         *     - lwipopts.h 里**无条件** LWIP_SO_SNDTIMEO = 1；
         *     - 于是 api_lib.c 在 conn->send_timeout != 0 时把 dontblock 置 1；
         *     - api_msg.c 里发送缓冲满 + dontblock ⇒ 立刻返回 ERR_WOULDBLOCK，
         *       **不等**那 100ms。
         *   也就是说 SO_SNDTIMEO 的语义是"攒不满就马上告诉我"，不是"最多等
         *   100ms"。所以"用阻塞 socket 就能躲开 EWOULDBLOCK"是**错的** ——
         *   缓冲一满它必来。
         *
         *   而背压生效的那一刻，正是它会来的时候：PC 读得慢 → 窗口收窄 →
         *   我们的发送缓冲被 STM32 回送流量填满 → send 返回 EWOULDBLOCK。
         *   把它当断开，等于"背压一生效就把连接拆了"，且这一刻 STM→PC 的
         *   数据全丢。这是本文件最容易写错的一处。
         *
         * ★ 只能在这里重试，不能"先返回、下一轮再发"：ReceiveUpTo 拿到的
         *   BYTEBUF item 在归还之前不能再取（ringbuf.c 的 prvCheckItemAvail
         *   对 BYTEBUF 有这条），所以调我们的 pump_to_pc 根本没得选。
         *
         *   vTaskDelay(1) 也是必须的：PC 长时间不读时若纯自旋，同优先级的
         *   spi_bridge 任务（同为 5）就再也拿不到 CPU。真正已经死掉的对端由
         *   keepalive 兜底（见 serve_client），不会让我们永远赖在这里。
         *
         * ★★ 无进展上限（步骤④ 补上，原先是无界重试）——
         *   这两种"对端不读"必须区别对待：
         *     读得慢但一直在读 ⇒ 正常，就该一直等下去（背压本来就是这么工作的）
         *     再也不读了       ⇒ 必须判死，否则整条桥一起陪葬
         *   判据取"sent **连续** TX_STALL_TICKS 没增长"而不是"总共花了多久"，
         *   正好把上面两种分开。被顶住的典型是"对端进程卡死/被调试器停住"，
         *   它的**内核仍然好好活着**，所以 keepalive 探活会一直成功、抓不到。
         *
         *   ★ 为什么步骤④ 非有它不可（步骤③ 时可以不管）：
         *     下载方向被顶住 ⇒ pump_from_pc 不再被调用 ⇒ TCP 接收窗口不再消耗、
         *     上位机的帧再也进不来 ⇒ 没有新帧就没有 ACK ⇒ STM→PC 方向也没东西可发
         *     ⇒ 整个 OTA 永久卡死。而 s_has_client 恒为 1，accept 再也不被调用，
         *     新连接只能排在 backlog 里没人服务。判死之后 serve_client 正常退出、
         *     重新 accept，用户至少能重连重来。 */
        if ((w < 0) && ((errno == EAGAIN) || (errno == EWOULDBLOCK)))
        {
            if ((TickType_t)(xTaskGetTickCount() - last_progress) >= TX_STALL_TICKS)
            {
                ESP_LOGW(TAG, "STM→PC 方向僵死：一批 %d B 已发出 %d B，剩余 %d B 在 %u ms 内毫无进展 —— 判对端不读了",
                         n, sent, n - sent, (unsigned)(TX_STALL_TICKS * portTICK_PERIOD_MS));
                return -1;
            }
            vTaskDelay(1);              /* 注意 HZ=100 ⇒ 这一睡是 10ms，不是 1ms */
            continue;
        }

        return -1;                      /* 真断开：ECONNRESET / EPIPE / ... */
    }
    return 0;
}

/* STM32 → PC：把 FromStmRing 里的东西尽量倒给 PC。
 * 返回 0 = 本次没得发或发完了，-1 = 对端断了。 */
static int pump_to_pc(int fd)
{
    RingbufHandle_t rb = Spi_Bridge_FromStmRing();

    /* ★ 隐式约束（ringbuf.c 的 prvCheckItemAvail 对 BYTEBUF 有这条）：
     *   **上一次取出的 item 没归还之前，再 Receive 会返回 NULL**（不是报错，
     *   是静默的"环空"）。所以下面只能是"取一笔 → 发 → 立刻还"。
     *   将来若为了凑大批量发送而改成"先取好几个 item 再一起发"，
     *   症状是这里**静默停摆**而不是崩溃 —— 别那么改。 */
    for (;;)
    {
        size_t   len = 0u;
        uint8_t *item = (uint8_t *)xRingbufferReceiveUpTo(rb, &len, 0, RXBUF_SIZE);
        int      r;

        if (item == NULL)
        {
            return 0;                       /* 环空，本次没得发 */
        }

        r = send_all(fd, item, (int)len);
        vRingbufferReturnItem(rb, item);    /* 发完（或失败）都得还，否则环会漏 */
        if (r != 0)
        {
            return -1;
        }
        s_up_bytes += (uint32_t)len;        /* 真正发给 PC 的才算 */
    }
}

/* PC → STM32：只有在 ToStmRing 有足够空位时才 recv。
 * 返回 0 = 正常，-1 = 对端断了。 */
static int pump_from_pc(int fd)
{
    RingbufHandle_t rb = Spi_Bridge_ToStmRing();
    uint8_t         buf[512];
    size_t          free_sz;
    size_t          want;
    int             n;

    /* ★★ 背压的全部实现就在这两行：环里空位不够就**不 recv**。
     *   recv 不调，TCP 接收窗口就不会被消耗，窗口自然收窄直到关闭，
     *   PC 那边的 send() 随之变慢/阻塞 —— 环永远不会被冲爆，也不需要
     *   丢弃。与 boot 侧用 tcp_recved(accepted) 控制窗口是同一个招。
     *
     *   留 64B 余量是因为 want ≤ free_sz 才是"这次 send 一定成功"的保证，
     *   中间没有第三方会来抢这块空间（消费端 SPI 任务只会让它更空）。 */
    free_sz = xRingbufferGetCurFreeSize(rb);
    if (free_sz < 64u)
    {
        /* ★ 必须主动睡：这一轮不会调 recv ⇒ 本轮**没有任何阻塞点**，
         *   如果另一方向的环也正好空着（pump_to_pc 立即返回），循环就退化成
         *   纯自旋把 CPU 占满。窗口本身很短（SPI 任务每笔抽走 256B，环立刻
         *   腾出空间，几十毫秒内自解），但白烧 CPU 是实打实的。
         *   ★ 这一睡是 **10ms 不是 1ms**（HZ=100 ⇒ 一个 tick = 10ms），而
         *     这处粒度**真的影响延迟**：醒来后一次性把腾出的 ~1280B 全 recv 走，
         *     平均速率仍等于 SPI 的排空速率、不掉速，代价是约 10ms 的抖动。
         *     （send_all 里那处 vTaskDelay 与延迟无关，见那里的说明。） */
        vTaskDelay(1);
        return 0;                           /* 太挤了，等 SPI 任务抽走一些再来 */
    }

    want = free_sz;
    if (want > sizeof(buf))
    {
        want = sizeof(buf);
    }

    n = recv(fd, buf, want, 0);
    if (n == 0)
    {
        return -1;                          /* 对端正常关闭 */
    }
    if (n < 0)
    {
        /* 阻塞 socket + SO_RCVTIMEO：超时表现为 EAGAIN/EWOULDBLOCK，
         * 这是"暂时没数据"，不是错误 —— 绝不能当成断开。 */
        if ((errno == EAGAIN) || (errno == EWOULDBLOCK))
        {
            return 0;
        }
        ESP_LOGW(TAG, "recv 出错 errno=%d", errno);
        return -1;
    }

    if (xRingbufferSend(rb, buf, (size_t)n, 0) != pdTRUE)
    {
        /* 按下面的论证到不了这里（want ≤ free_sz，且中间只有 SPI 任务在**取走**
         * 数据，只会让空位更多）。真到了就是假设被破坏 ——
         * ★ 而这几字节**是永久丢失的**，别指望"上位机超时重发"：它们已经用
         *   recv() 从 TCP 接收缓冲里取走了，对端早就 ACK 过，TCP 绝不会重发；
         *   步骤③ 又是裸字节流，没有帧级重传。只有步骤④ 的逐帧 ACK/重传能兜。 */
        ESP_LOGW(TAG, "to_stm 环满，丢弃 %d B（永久丢失）", n);
    }
    else
    {
        s_down_bytes += (uint32_t)n;        /* 从 TCP 取走并成功入环的才算 */
    }
    return 0;
}

/* 服务一个客户端，直到它断开 */
static void serve_client(int fd, const char *peer)
{
    struct timeval tv;
    int opt = 1;
    int ka_idle = 5;
    int ka_intvl = 2;
    int ka_cnt = 3;

    s_has_client = 1u;
    s_down_bytes = 0u;
    s_up_bytes   = 0u;
    ESP_LOGI(TAG, "客户端接入 %s —— 进入 SPI↔TCP 透传", peer);

    /* ★★ 会话边界：把上一轮残留在环里的字节清掉，再开始本会话。
     *   两个环都是进程级静态的、跨连接活着；上一轮 PC 没读完时留在
     *   FromStmRing 里的回显，会在本会话的数据之前被倒给新 PC，而 PC 只能
     *   把它当成自己的回显 —— 表现是"每次重连后开头若干字节必错"。
     *   ★ 必须在**进入下面的循环之前**清：一旦开始 pump，旧字节就已经发出去了。
     *   ★ 这里也是 ToStmRing 唯一一次被清的机会 —— 清空后 SPI 任务立刻会向
     *     F407 发 RX_LEN=0，F407 据此把 pending 清 0、停止回显（所以 F407 侧
     *     不需要任何改动，它的"原样重发"本来就会被"没数据了"这句话清掉）。 */
    Spi_Bridge_FlushRings();

    /* ★ keepalive：没有它，PC 静默消失（WiFi 掉 / 进程被 kill / 笔记本休眠）
     *   时，只要双向流量都很小，recv 就永远只是"超时返回"，我们**永远发现不了**
     *   对端已经死了。后果是一连串看似无关的现象：
     *     - s_has_client 恒为 1 → LCD 一直显示 T:1 / LINK OK；
     *     - accept() 再也不被调用 → 新 PC 连接 TCP 握手能成（进 backlog），
     *       但没人去服务它 → 上位机"连上了却收不到任何回包"，然后去怀疑 SPI。
     *   ★ ESP-IDF 的 lwipopts.h 开了 LWIP_TCP_KEEPALIVE，所以下面三个参数
     *     可以**带单位秒**地调（不开的话它们是以 500ms 为单位的）。
     *     5s 空闲起探、间隔 2s、3 次不应答判死 ≈ 11s 发现。
     *   探活失败后 lwIP 会让 recv 返回错误，serve_client 正常退出、重新 accept。 */
    if (setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &opt, sizeof(opt)) != 0)
    {
        ESP_LOGW(TAG, "开 keepalive 失败 errno=%d", errno);
    }
    /* 这三个被拒时**不会报错**，只会静默退回 lwIP 默认的 2 小时 keep_idle ——
     * 等于把上面要治的病原样留着。所以逐个查返回值。 */
    if (setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE,  &ka_idle,  sizeof(ka_idle)) != 0)
    {
        ESP_LOGW(TAG, "设 KEEPIDLE 失败 errno=%d", errno);
    }
    if (setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL, &ka_intvl, sizeof(ka_intvl)) != 0)
    {
        ESP_LOGW(TAG, "设 KEEPINTVL 失败 errno=%d", errno);
    }
    if (setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT,   &ka_cnt,   sizeof(ka_cnt)) != 0)
    {
        ESP_LOGW(TAG, "设 KEEPCNT 失败 errno=%d", errno);
    }

    /* ★ 用**阻塞 socket + 超时**，而不是 O_NONBLOCK。真正的理由是接收侧：
     *   SO_RCVTIMEO 的"超时"语义正好等于我们要的"这会儿没数据，先去照顾
     *   反方向"，比轮询 EWOULDBLOCK 干净，一个循环就能同时照顾两个方向。
     *
     *   ⚠ 发送侧**不是**靠这个躲开 EWOULDBLOCK 的 —— 见 send_all 上方那段：
     *   ESP-IDF 的 lwIP 下 SO_SNDTIMEO 等于 dontblock，缓冲满时 EWOULDBLOCK
     *   照样立刻返回。它靠的是 send_all 内部重试。 */
    tv.tv_sec  = 0;
    tv.tv_usec = RX_TIMEOUT_MS * 1000;
    if (setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) != 0)
    {
        ESP_LOGW(TAG, "设 RCVTIMEO 失败 errno=%d —— 会退化成永久阻塞", errno);
    }

    tv.tv_usec = TX_TIMEOUT_MS * 1000;
    if (setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv)) != 0)
    {
        ESP_LOGW(TAG, "设 SNDTIMEO 失败 errno=%d", errno);
    }

    for (;;)
    {
        /* ★ 两个 break 各打一行：**是哪一侧先断的**决定了往哪查，而原来
         *   "客户端已断开"一句话把两种完全相反的成因合成了一种。
         *   实测教训（步骤③）：PC 侧只看到"连接没了"，C3 侧只说"断开了"，
         *   于是谁先断的、为什么断的，两边都答不上来。 */
        if (pump_to_pc(fd) != 0)
        {
            ESP_LOGW(TAG, "断开：STM→PC 方向失败（对端不再收）");
            break;
        }
        if (pump_from_pc(fd) != 0)
        {
            ESP_LOGW(TAG, "断开：PC→STM 方向失败（对端关闭或 recv 出错）");
            break;
        }
        /* 本循环**不靠这一行防自旋**。真正让出 CPU 的是 pump_from_pc 里的 recv ——
         * 它必须真的阻塞在超时上（RX_TIMEOUT_MS ≥ 1 tick 那条 #error 就是保它）。
         * 若那条约束被破坏，recv 立刻返回，整轮就一个阻塞点都没有了。
         * ★ 这里的 taskYIELD 只让给**同或更高**优先级的任务，对 idle（优先级 0）
         *   毫无作用 —— 它的实际作用是让同级的 spi_bridge（也是 5）尽快拿到 CPU
         *   去抽环。别指望它防饿死。 */
        taskYIELD();
    }

    s_has_client = 0u;

    /* ★★ 这一行是步骤③ 排查的核心证据，一次给全：
     *   PC→STM 为 0            ⇒ PC 的字节压根没进 C3（问题在 TCP，不在 SPI）
     *   PC→STM 正常但 STM→PC 大 ⇒ 单向多出来 = 重复投递或跨会话残留
     *   short/stale/oob 任一个非 0 ⇒ **SPI 事务层在丢帧/错帧**，这是与"环"完全
     *                              不同的一类故障，别再去查环形缓冲
     *   drop 非 0              ⇒ 消费端（TCP）没跟上，环满丢数，与 SPI 无关
     *   三个坏帧计数是**累计值**（跨会话不清零），用来看趋势而不是看单次。 */
    {
        Spi_Bridge_Stats_t st;

        Spi_Bridge_GetStats(&st);
        ESP_LOGI(TAG, "客户端已断开：本会话 PC→STM %u B，STM→PC %u B",
                 (unsigned)s_down_bytes, (unsigned)s_up_bytes);
        ESP_LOGI(TAG, "SPI 侧本会话：坏帧 short=%u stale=%u badhdr=%u oob=%u，环满丢弃 %u B，F407 实发 %u B",
                 (unsigned)st.short_cnt, (unsigned)st.stale_cnt, (unsigned)st.bad_hdr_cnt,
                 (unsigned)st.oob_cnt, (unsigned)st.drop_cnt, (unsigned)st.rx_total);
        if ((st.short_cnt != 0u) || (st.stale_cnt != 0u) ||
            (st.bad_hdr_cnt != 0u) || (st.oob_cnt != 0u))
        {
            ESP_LOGW(TAG, "★ 有坏帧 —— 问题在 SPI 事务层（主机时钟/挂载时序），不在环形缓冲");
        }
        /* ★ F407 实发 ≠ STM→PC，说明从 SPI 到 PC 这一段还掉了字节
         *   （环满丢弃、或会话结束前没送完）。drop 为 0 时两者本该相等。 */
        if ((st.drop_cnt == 0u) && (st.rx_total != s_up_bytes))
        {
            ESP_LOGW(TAG, "★ F407 实发 %u B 与 STM→PC %u B 不等，而环没满 —— 差额另有来源",
                     (unsigned)st.rx_total, (unsigned)s_up_bytes);
        }
    }
}

static void tcp_bridge_task(void *arg)
{
    struct sockaddr_in addr;
    struct sockaddr_in peer;
    int listen_fd;
    int opt = 1;

    (void)arg;

    listen_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (listen_fd < 0)
    {
        ESP_LOGE(TAG, "socket 失败 errno=%d —— esp_netif 是否已初始化？", errno);
        vTaskDelete(NULL);
        return;
    }

    setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    memset(&addr, 0, sizeof(addr));
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);       /* 哪个网口都收 */
    addr.sin_port        = htons(TCP_BRIDGE_PORT);

    if (bind(listen_fd, (struct sockaddr *)&addr, sizeof(addr)) != 0)
    {
        ESP_LOGE(TAG, "bind 端口 %d 失败 errno=%d", TCP_BRIDGE_PORT, errno);
        close(listen_fd);
        vTaskDelete(NULL);
        return;
    }

    if (listen(listen_fd, LISTEN_BACKLOG) != 0)
    {
        ESP_LOGE(TAG, "listen 失败 errno=%d", errno);
        close(listen_fd);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "TCP 桥已就绪，监听 %d 端口", TCP_BRIDGE_PORT);

    for (;;)
    {
        socklen_t plen = sizeof(peer);
        int fd = accept(listen_fd, (struct sockaddr *)&peer, &plen);

        if (fd < 0)
        {
            ESP_LOGW(TAG, "accept 失败 errno=%d", errno);
            vTaskDelay(pdMS_TO_TICKS(100));          /* 防忙等 */
            continue;
        }

        /* ★ 关 Nagle：OTA 的 ACK 都是小包，攒批会平白引入几十毫秒延迟。
         *   与 boot 侧 ota_accept_cb 里的 tcp_nagle_disable() 同一个理由。 */
        setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &opt, sizeof(opt));

        /* ★ inet_ntoa 返回的是**静态缓冲**，非线程安全。当前只有本任务调用它
         *   所以安全 —— 别在别处再调（ESP_LOG 里混用会让日志里的 IP 串掉）。 */
        serve_client(fd, inet_ntoa(peer.sin_addr));

        close(fd);
    }
}

void Tcp_Bridge_Start(void)
{
    /* 4096 栈够用：接收缓冲在栈上只占 512B，其余都是小局部变量 */
    xTaskCreate(tcp_bridge_task, "tcp_bridge", 4096, NULL, 5, NULL);
}
