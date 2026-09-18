#include "ota_net.h"

#include "lwip.h"              /* heth（ETH 句柄，跳转前要靠它停 DMA） */
#include "lwip/tcp.h"
#include "lwip/ip_addr.h"
#include "lwip/pbuf.h"
#include "lwip/err.h"

/* ==================== 内部状态 ==================== */

/* 监听 PCB：常驻，不随连接建立/断开而销毁 */
static struct tcp_pcb *s_listen = NULL;

/* 当前连接 PCB：无连接时为 NULL */
static struct tcp_pcb *s_conn = NULL;

/* 接收汇点：由 OTA_Net_Init() 注入 */
static OTA_RxSink_t s_sink = NULL;

/* ==================== 发送出口 ==================== */

/* 非阻塞发送。buf 必须在本函数返回后即可失效（内部按值拷贝）。 */
uint8_t OTA_Net_Send(const uint8_t *buf, uint16_t len)
{
    if (s_conn == NULL)
    {
        return 1u;                       /* 无上位机连接 */
    }
    if (len == 0u)
    {
        return 0u;
    }
    if (tcp_sndbuf(s_conn) < len)
    {
        return 1u;                       /* 发送窗口不足，本次发不出 */
    }
    /* TCP_WRITE_FLAG_COPY：buf 一般是调用方的栈缓冲，必须让 lwIP 拷贝一份 */
    if (tcp_write(s_conn, buf, len, TCP_WRITE_FLAG_COPY) != ERR_OK)
    {
        return 1u;
    }
    tcp_output(s_conn);                  /* 立即发出，不等攒批 */
    return 0u;
}

uint8_t OTA_Net_Connected(void)
{
    return (s_conn != NULL) ? 1u : 0u;
}

/* ==================== 接收 ==================== */

/* 收到 TCP 数据：只做"入汇点"，不解析协议（解析由上层 Proto_Feed 负责） */
static err_t ota_recv_cb(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err)
{
    struct pbuf *q;
    uint16_t accepted = 0u;

    (void)arg;

    if (err != ERR_OK)
    {
        if (p != NULL)
        {
            pbuf_free(p);
        }
        return err;                      /* 异常：交给 err 回调统一清理 */
    }

    if (p == NULL)                       /* 对端关闭连接（FIN） */
    {
        if (s_conn == pcb)
        {
            s_conn = NULL;
        }
        tcp_close(pcb);
        return ERR_OK;
    }

    /* 遍历 pbuf 链：一个 TCP 段可能被拆成多个 pbuf，
     * 必须整条链喂进去，否则中间会丢数据。 */
    for (q = p; q != NULL; q = q->next)
    {
        if (s_sink != NULL)
        {
            accepted += s_sink((const uint8_t *)q->payload, (uint16_t)q->len);
        }
    }

    /* ★ 只确认"真正被接收"的字节数：
     *   环满时 accepted < p->tot_len → TCP 窗口不放大 → 对端自动降速。
     *   这是 TCP 通道的背压来源，也是应用层不能静默丢弃的原因。 */
    tcp_recved(pcb, accepted);

    pbuf_free(p);
    return ERR_OK;
}

/* ==================== 异常 ==================== */

static void ota_err_cb(void *arg, err_t err)
{
    (void)arg;
    (void)err;

    /* ★ lwIP 进入本回调时 PCB 已被释放，只能清指针，
     *   绝不能再调用 tcp_close()/tcp_abort()（use-after-free）。 */
    s_conn = NULL;
}

/* ==================== 接受连接 ==================== */

static err_t ota_accept_cb(void *arg, struct tcp_pcb *newpcb, err_t err)
{
    (void)arg;

    if (err != ERR_OK || newpcb == NULL)
    {
        return ERR_VAL;
    }

    /* 单连接模型：新连接顶掉旧连接（一次只服务一个上位机） */
    if (s_conn != NULL)
    {
        tcp_abort(s_conn);               /* abort 立即断开，不等 FIN 握手 */
        s_conn = NULL;
    }
    s_conn = newpcb;

    tcp_recv(newpcb, ota_recv_cb);
    tcp_err(newpcb, ota_err_cb);
    tcp_nagle_disable(newpcb);           /* OTA 帧密集且多为小 ACK，关 Nagle 降延迟 */

    return ERR_OK;
}

/* ==================== 初始化 ==================== */

void OTA_Net_Init(OTA_RxSink_t sink)
{
    err_t e;

    s_sink = sink;

    s_listen = tcp_new();
    if (s_listen == NULL)
    {
        return;
    }

    e = tcp_bind(s_listen, IP_ADDR_ANY, OTA_TCP_PORT);
    if (e != ERR_OK)
    {
        tcp_close(s_listen);
        s_listen = NULL;
        return;
    }

    /* ★ tcp_listen() 会释放传入的 PCB 并返回一个新 PCB，必须接住返回值，
     *   否则监听 PCB 丢失（内存泄漏 + 无法接受连接）。 */
    s_listen = tcp_listen(s_listen);
    if (s_listen == NULL)
    {
        return;
    }

    tcp_accept(s_listen, ota_accept_cb);
}

void OTA_Net_DeInit(void)
{
    if (s_conn != NULL)
    {
        tcp_abort(s_conn);             /* abort：立即断开，不等 FIN 挥手（马上要跳转了） */
        s_conn = NULL;
    }
    if (s_listen != NULL)
    {
        tcp_close(s_listen);
        s_listen = NULL;
    }

    /* ★ 停 ETH MAC + DMA。tcp_abort/tcp_close 只清理 lwIP 层的控制块，
     *   硬件侧的 DMA 仍在按描述符环搬运数据、写 boot 的 RAM 缓冲。
     *   app 重新初始化 ETH 时会另建一套描述符环，两者指向同一片内存 →
     *   随机丢包/踩内存，且现象与 OTA 毫无关联，极难定位。 */
    (void)HAL_ETH_Stop(&heth);
}
