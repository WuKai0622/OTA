#ifndef __SPI_BRIDGE_H
#define __SPI_BRIDGE_H

#include <stdint.h>

#include "freertos/ringbuf.h"

/* ==================== SPI 从机 ↔ TCP 桥 ====================
 * PC ←TCP→ ESP32-C3 ←SPI→ F407，双向字节流透传。
 *
 * ★ 这条桥**与内容完全无关** —— 步骤③ 时它搬的是回显载荷，现在（步骤④）
 *   搬的是 OTA 帧，它自己毫无察觉，也一行都不用改。
 *   当初坚持"先把透传单独验通、再碰 OTA"就是为了这一刻：接真 OTA 时出问题，
 *   只剩"帧协议 / 会话绑定"两个变量，不会再混进"SPI 通不通""TCP 通不通"。
 *
 * 本模块管 SPI 侧：事务循环 + 两个环形缓冲；TCP 侧在 tcp_bridge.c。
 */

/* ---- 事务格式（与 F407 的 ota_spi.h 是同一份契约，改一处必须改两处）----
 *
 *   MOSI (F407 → C3，C3 侧是 rx): [TX_LEN(2)][SQ(2)][RSV(4)]             + 256B 载荷
 *   MISO (C3 → F407，C3 侧是 tx): [RX_LEN(2)][STATUS(2)][HB(2)][RSV(2)]  + 256B 载荷
 *
 * ★ SQ = F407 每笔自增的**帧序号**（占用原 RSV 的前 2 字节）。
 *   它不是锦上添花，是步骤③ 实测踩出来的必需品：从机侧判"这一笔有没有真的
 *   捕获到东西"不能只看驱动的 trans_len —— 完全没有时钟进来时它会**保留上一笔
 *   的值**，s_rx 于是原封不动还是上一帧，长度字段看着完全合法，同一块载荷被
 *   又投递了一遍（PC 收到双份，两端计数器都不报错）。C3 靠比对 SQ 拦下这种帧。
 *
 * ★ 两个长度名都是**站在 F407 的角度**命名的：
 *     TX_LEN = "F407 这一笔发了多少有效字节"
 *     RX_LEN = "F407 这一笔收到多少有效字节"（也就是 C3 发了多少）
 *   C3 侧正好反过来用，别被名字绕进去。
 *
 * 全部**小端**。两端都是小端机，把字节序定死比省一次转换更重要 ——
 * 否则将来换个大端平台就会变成极难查的隐式 bug。
 *
 * ★ 为什么定长、不协商：变长要求两端各自算出**同一个数**，算错就时序错乱，
 *   是极难查的隐式耦合；两阶段协商又有循环依赖（master 发起事务前就得知道
 *   长度，但长度要在事务里才拿到）。定长直接把这个坑删掉。
 */
#define SPI_BRIDGE_HDR_LEN      8u
#define SPI_BRIDGE_PAYLOAD_LEN  256u
#define SPI_BRIDGE_XFER_LEN     (SPI_BRIDGE_HDR_LEN + SPI_BRIDGE_PAYLOAD_LEN)

/* STATUS 位（MISO 头的第 2、3 字节）。
 * 有了它，LCD 上就能直接区分"WiFi 断了"和"SPI 断了" —— 否则两种故障现象
 * 都是"PC 发了没反应"，只能盲猜。 */
#define SPI_BRIDGE_ST_WIFI      0x0001u     /* WiFi 已就绪 */
#define SPI_BRIDGE_ST_TCP       0x0002u     /* 有 PC 连着 */

/* 环形缓冲容量。
 * ★ 4KB 够用：PC 是逐帧 stop-and-wait（同一时刻最多 1 帧 1036B 在路上），
 *   环不会满。真满了就丢 —— SPI 的字节一旦被时钟打进来就无法拒收，
 *   这是 UART 那套模型，不是 TCP 的背压模型。丢掉的由上位机超时重发兜底。 */
#define SPI_BRIDGE_RING_SIZE    4096u

/* 建环 + 初始化 SPI 从机 + 起 SPI 任务。必须在 Tcp_Bridge_Start() 之前调用
 * （TCP 任务要用这里建的两个环）。 */
void Spi_Bridge_Start(void);

/* 两个环的句柄，SPI 任务与 TCP 任务各用一头：
 *   ToStmRing   : TCP → SPI。PC 发来的字节，等着被装进 MISO 送给 STM32。
 *   FromStmRing : SPI → TCP。STM32 发来的字节，等着被 TCP 任务送回 PC。
 * 由本模块创建，tcp_bridge 通过这两个函数取用。 */
RingbufHandle_t Spi_Bridge_ToStmRing(void);
RingbufHandle_t Spi_Bridge_FromStmRing(void);

/* 清空跨会话残留的数据，**阻塞**到确认清干净为止（最多 500ms）。
 * ★ 必须在**每次新客户端接入**时、**开始 pump 之前**调用 —— 这条桥没有会话
 *   边界，上一轮留下的回显会被新 PC 当成自己的回显读走（步骤③ 是裸字节流，
 *   无法分辨新旧）。
 * ★ 它清的不只是两个环：还有一块残留在 **F407 的 `pending`** 里（它已收下、
 *   正等着发出去），那块只能靠继续跑事务才勾得出来 —— 所以本函数会**等**，
 *   不能在 TCP 任务里"清完就走"。详见 spi_bridge.c 的实现注释。 */
void Spi_Bridge_FlushRings(void);

/* SPI 侧的**本会话**计数，一次取全 —— 断连时打一行，故障归类就不必再翻整段日志。
 * ★ 口径是**本会话**（每次客户端接入时在 Spi_Bridge_FlushRings 里清零），
 *   这样 rx_total 才能和同一行的 PC→STM / STM→PC 直接对账。
 *
 *   四个坏帧计数**任意一个不为 0**，都说明 SPI 事务层在丢帧或错帧
 *   （不是会话边界问题，别再去查环）。四者是四种不同的坏法：
 *     short_cnt   : 事务被截断或零位（主机没敲够时钟）
 *     stale_cnt   : 序号没变，这一笔什么都没捕获到（trans_len 骗了我们）
 *     bad_hdr_cnt : 帧换了但**错位/混合**（旧帧尾 + 新帧头）—— 序号抓不住
 *     oob_cnt     : TX_LEN 越界，帧被错位打坏
 *   drop_cnt 不为 0 则是**消费端**（TCP）没跟上，环满丢数，与 SPI 无关。 */
typedef struct
{
    uint32_t short_cnt;     /* 短事务（被截断/零位）笔数 */
    uint32_t stale_cnt;     /* 陈旧帧（序号未变）笔数 */
    uint32_t bad_hdr_cnt;   /* 头部结构校验失败（帧错位/混合）笔数 */
    uint32_t oob_cnt;       /* TX_LEN 越界次数 */
    uint32_t drop_cnt;      /* from_stm 环满丢弃的字节数 */
    uint32_t rx_total;      /* 本会话从 F407 收到的有效字节数 */
} Spi_Bridge_Stats_t;

void Spi_Bridge_GetStats(Spi_Bridge_Stats_t *out);

#endif /* __SPI_BRIDGE_H */
