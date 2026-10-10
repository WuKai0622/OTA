/* ============================================================================
 * app 侧的 OTA 触发口（文件头的背景说明在 app_ota.h 里）
 *
 * ★ 本文件是 boot 侧 `ota_spi.c` 的**极简表亲**：只保留"开一笔事务、看 MISO 头"
 *   这一层，砍掉发送缓冲、帧投递、屏上诊断 —— 因为 app 侧的活只有一件：
 *   发现有人在跟我们说话就复位。所以这里没有 socket、没有帧状态机、没有 ACK。
 * ==========================================================================*/
#include "app_ota.h"

#include <string.h>

#include "main.h"
#include "spi.h"      /* hspi2 */
#include "FreeRTOS.h"
#include "task.h"

/* ==================== 片选（PB12） ====================
 * ★★ app 的 CubeMX 工程里**没有这个脚**，必须在这里自己配。
 *   boot 侧是 CubeMX 里的一个 GPIO_Output 标签（SPI2_CS，见 boot 的 main.h），
 *   app 侧当时没加 —— 漏了它 PB12 就停在复位后的**浮空输入**，
 *   主机（我们）不发片选时 ESP32-C3 无从判断"这一笔算不算一笔"，
 *   表现就是"SPI 完全没反应"。而它和"没接线""C3 没起来"长得一模一样。
 *
 * ★ 为什么不干脆再去 CubeMX 里加一个脚：加了更好（那边配的和这里配的一模一样，
 *   重复配同一个脚是无害的），但代码**不能依赖**它 —— .ioc 一重新生成，
 *   gpio.c 里的非 USER CODE 行会被整段覆盖，配在里面的东西会被悄悄删掉，
 *   而这次改动不留下任何痕迹。配在这里则永远成立。
 * ★ 参数逐项抄 boot 的 gpio.c：推挽输出 / 无上下拉 / 最高速 / 初值高。 */
#define APP_OTA_CS_PORT   GPIOB
#define APP_OTA_CS_PIN    GPIO_PIN_12

#define APP_OTA_CS_LOW()  HAL_GPIO_WritePin(APP_OTA_CS_PORT, APP_OTA_CS_PIN, GPIO_PIN_RESET)
#define APP_OTA_CS_HIGH() HAL_GPIO_WritePin(APP_OTA_CS_PORT, APP_OTA_CS_PIN, GPIO_PIN_SET)

/* ==================== 事务格式（与 boot 的 ota_spi.h / C3 的 spi_bridge.h 同一份契约）====
 *
 *   MOSI (F407 → C3): [TX_LEN(2)][SQ(2)][RSV(4)]             + 256B 载荷
 *   MISO (C3 → F407): [RX_LEN(2)][STATUS(2)][HB(2)][RSV(2)]  + 256B 载荷
 *
 * ★ 这四个宏在 app 侧是**重新写一遍**的，不是 include 来的：boot 与 app 是两个
 *   独立的 Keil 工程，app 看不见 ota_spi.h。所以**改契约要改三处**
 *   （ota_spi.h / spi_bridge.h / 这里）—— 这也是下面每个数都标出来源的原因。
 *
 * ★ 长度名是站在 F407 角度的：TX_LEN = 我们发了多少有效字节（app 恒为 0），
 *   RX_LEN = 我们收到多少有效字节（= C3 发了多少）。 */
#define APP_OTA_HDR_LEN       8u
#define APP_OTA_PAYLOAD_LEN   256u

/* 一笔事务 264B。★ 必须是 4 的倍数（ESP32-C3 侧启 DMA 时，主机写入长度非 4 的
 * 倍数会被直接丢弃）。264 = 4 × 66 ✓。 */
#define APP_OTA_XFER_LEN      (APP_OTA_HDR_LEN + APP_OTA_PAYLOAD_LEN)

/* HAL 超时（ms）。/32 预分频 → 1.3125MHz，264B ≈ 1.61ms。抄 boot 的 100。 */
#define APP_OTA_TIMEOUT_MS    100u

/* 轮询间隔（ms）。见 vAppOtaTask 末尾那段关于余量的推算。 */
#define APP_OTA_POLL_MS       100u

/* 起来之后先等多久才开第一笔事务（ms）。**这个数不是礼貌性延时**，理由见 vAppOtaTask */
#define APP_OTA_START_DELAY_MS  2500u

/* ==================== 事务层状态 ==================== */

/* 两个缓冲都是**静态**的：264B × 2 放栈上要吃掉这个任务栈的一半，
 * 而它们本来就该常驻（每笔都要用，重新 memset 也是浪费）。 */
static uint8_t  s_mosi[APP_OTA_XFER_LEN];
static uint8_t  s_miso[APP_OTA_XFER_LEN];

/* MOSI 帧序号，每笔自增。★ 不是可选装饰，见 AppOta_OneTransaction 里的说明。 */
static uint16_t s_sq = 0u;

/* ==================== 物理层：一笔定长事务 ==================== */

static uint8_t AppOta_Transfer(const uint8_t *tx, uint8_t *rx, uint16_t len)
{
    HAL_StatusTypeDef st;

    APP_OTA_CS_LOW();

    /* 必须 TransmitReceive，不能 Transmit + Receive 分两次：全双工是**同一段时钟
     * 里**收发，C3 要回送的头（RX_LEN/STATUS/HB）只在这段时钟里出现在 MISO 上。
     * tx 转掉 const：HAL 的签名是 uint8_t*，但它只读不写。 */
    st = HAL_SPI_TransmitReceive(&hspi2, (uint8_t *)tx, rx, len, APP_OTA_TIMEOUT_MS);

    APP_OTA_CS_HIGH();

    if (st != HAL_OK)
    {
        /* ★★ 与 boot 的 ota_spi.c 逐字同因，而且 app 这里**更要**做 —— 因为 app 是
         *   "一直轮询"的，一次错位会被永久重复下去：
         *
         *   HAL 的超时分支只做 State=READY + 解锁就返回，**既不禁用外设、也不读走
         *   DR**。于是超时那一刻若已有一个字节进了接收寄存器，它会**留在 DR 里**；
         *   下一笔事务的头一次读就把这个陈旧字节当成 s_miso[0]，其后所有字节整体
         *   错位 1 字节，而且错位自持（每笔都多留一个），不会自愈。
         *   后果：s_miso[6..7] 的结构校验从此**每一笔都失败** ⇒ RX_LEN 永远不可信
         *   ⇒ **本文件存在的唯一目的（发现 PC 来了就复位）彻底失效**，
         *   而 SPI 表面上还在正常跑、屏上还在正常显示 app。
         *
         *   禁用外设会复位它的内部状态机，读掉 DR/SR 清掉 OVR 与残留数据。
         *   此刻 CS 已拉高（从机已释放），软复位不会在总线上产生毛刺；
         *   SPE 之外的 CR1 位（CPOL/CPHA/BR/MSTR/DFF...）不受影响，配置不会被改掉。 */
        __HAL_SPI_DISABLE(&hspi2);
        (void)hspi2.Instance->DR;
        (void)hspi2.Instance->SR;
        __HAL_SPI_ENABLE(&hspi2);
        return 1u;
    }

    return 0u;
}

/* ==================== 事务层：跑一笔，回答"PC 来了吗" ==================== */

/* 返回 1 = 从 C3 收到了字节（有人在我们这条链路的另一头说话）；
 * 返回 0 = 没有 / 这一笔不可信。 */
static uint8_t AppOta_OneTransaction(void)
{
    uint16_t rx_len;

    /* ---- MOSI 头 [TX_LEN=0][SQ][RSV=0 ×4] ----
     *
     * TX_LEN 恒 0：app 没有东西要发给 PC，这是**实话**，也正是 C3 期望的
     *   "F407 手里没东西"信号（它在会话边界屏障里要靠这个值判断我们吐干净了）。
     *   s_mosi[0..1] 因此从头到尾没被写过，靠 AppOta_Init 的 memset 保持 0。
     *
     * ★ RSV 那 4 字节（s_mosi[4..7]）**必须一直是 0**，同样靠"从没写过"。
     *   C3 拿它们做结构校验：非 0 就判成"帧错位/混合"而整笔丢弃
     *   （见 spi_bridge.c 里那一支）。往这里写任何东西都会让 C3 把每一笔都丢掉。
     *
     * ★★ SQ 每笔自增，这是**必需品**：C3 判断"这一笔有没有真的捕获到东西"不能
     *   只看驱动给的长度 —— 完全没有时钟进来时驱动会保留上一笔的值，于是同一块
     *   载荷被又投递一遍。C3 靠比对 SQ 拦下这种帧，**序号不变就当作陈旧帧丢掉**。
     *   而"丢掉"并不等于"放过"：C3 照样会把 PC 那 256B 从接收环里取走并丢弃
     *   （BuildNextTx 在每笔事务后都会取），也就是**那一帧被无声地吃掉**。
     *   ⇒ 如果这里发一个固定的 SQ，PC 的 START 就会被连续吃掉，
     *     而"吃掉"恰好是我们要的效果（见 vAppOtaTask），所以更要让它**真的**发生：
     *     序号在变，这一笔才算"有效的一笔"，C3 才会正常走完流程把 RX_LEN 报回来。
     *     详见 spi_bridge.c 的帧新鲜度校验那一段。 */
    s_sq++;
    s_mosi[2] = (uint8_t)(s_sq & 0xFFu);
    s_mosi[3] = (uint8_t)((s_sq >> 8) & 0xFFu);

    if (AppOta_Transfer(s_mosi, s_miso, APP_OTA_XFER_LEN) != 0u)
    {
        return 0u;      /* 这一笔没成：当作"什么都没发生"，不复位 */
    }

    /* ---- MISO 头 [RX_LEN(2)][STATUS(2)][HB(2)][RSV(2)] ----
     * ★ 先做结构校验，过了才认 RX_LEN —— 顺序不能反：
     *   RSV 非 0 说明这一笔收到的是错位/噪声，那么 RX_LEN 同样不可信。
     *   不看这一条，一个被打坏的字节就可能伪造出"RX_LEN>0"把设备复位掉。
     *   （误复位本身不致命，但"随机复位"是最难查的那类现象，能免则免。）
     * ★ C3 的 BuildNextTx 整块 memset 之后只写 [0..5]，所以 [6..7] 恒为 0。 */
    if ((s_miso[6] | s_miso[7]) != 0u)
    {
        return 0u;
    }

    rx_len = (uint16_t)((uint16_t)s_miso[0] | ((uint16_t)s_miso[1] << 8));

    /* 越界只可能是帧被打坏（C3 侧上限就是 256）。按坏帧处理，不复位。 */
    if (rx_len > APP_OTA_PAYLOAD_LEN)
    {
        return 0u;
    }

    /* ★ 判据就这一条：**收到任何字节**。
     *   C3 的接收环（PC → F407 那个方向）唯一的上游是它的 TCP 客户端，
     *   所以"这里有字节" ⟺ "PC 连上 5001 并且正在发东西"。
     *   STATUS / 心跳都走 MISO 头，不进这个载荷，不会伪装成 PC 的字节。 */
    return (rx_len > 0u) ? 1u : 0u;
}

/* ==================== 对外：初始化 + 轮询任务 ==================== */

void AppOta_Init(void)
{
    GPIO_InitTypeDef gi = {0};

    /* 再使能一次 GPIOB 时钟：MX_GPIO_Init 已经开过了，重复开是无害的置位操作。
     * 放在这里是为了让本函数**自己能站住**（不依赖调用顺序）。 */
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* ★ 顺序要紧：先把 ODR 写成高，再把引脚切成输出。
     *   反过来的话，从"浮空输入"切到"输出"的那一瞬间引脚电平不确定，
     *   而 C3 的片选是电平敏感的 —— 它可能把这一个毛刺当成一笔事务的开头。
     *   HAL_GPIO_WritePin 写的是 ODR，此刻引脚还是输入，不会真的驱动；
     *   切到输出的那一刻就直接输出这个高电平，中间没有过渡态。
     *   boot 的 gpio.c 也是这个顺序。 */
    APP_OTA_CS_HIGH();

    gi.Pin   = APP_OTA_CS_PIN;
    gi.Mode  = GPIO_MODE_OUTPUT_PP;
    gi.Pull  = GPIO_NOPULL;
    gi.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(APP_OTA_CS_PORT, &gi);

    memset(s_mosi, 0, sizeof(s_mosi));
    memset(s_miso, 0, sizeof(s_miso));

    /* ★ SQ 初值取 0 ⇒ **第一笔事务发出去的序号是 1**（先自增再写，与 boot 的
     *   ota_spi.c 同一个约定）。
     *   C3 只判"和上一笔相不相同"，所以初值本身不影响后续 —— 唯一能撞上的是
     *   **跨固件的那一次交接**：app 复位后 boot 的第一笔 SQ 也是 1，若 app 最后
     *   一笔恰好也是 1（16 位计数器走一圈会经过每一个值，概率约 1/65536），
     *   C3 会把 boot 的第一笔当陈旧帧丢掉，连带把 PC 那一帧一起吃掉。
     *   后果只是"上位机那一次重发落空、再重发一次"，START_RETRY = 3 兜得住，
     *   所以这里不为它引入任何特殊处理（不值得，也解释不清）。 */
    s_sq = 0u;
}

/* ---- 任务本体用的静态存储（见下面 CreateTask 的说明）----
 * 256 字 = 1KB。给到别的任务（128 字）的两倍：这笔调用链是 HAL 里最深的一条
 * （HAL_SPI_TransmitReceive 之下还有若干层），而本工程的栈溢出检查是关掉的
 * （configCHECK_FOR_STACK_OVERFLOW = 0）—— 真溢出不会被发现，踩的是**别人**的栈。
 * 多花 512B 换掉一类"查不出来的偶发故障"，很划算。 */
#define APP_OTA_TASK_STACK   256u

static StaticTask_t s_ota_tcb;
static StackType_t  s_ota_stack[APP_OTA_TASK_STACK];

void AppOta_CreateTask(void)
{
    /* ★★ 走**静态分配**而不是 xTaskCreate，理由是堆的余量：
     *   本工程的堆只有 configTOTAL_HEAP_SIZE = 4096B（FreeRTOSConfig.h），
     *   而现有 4 个任务 + 队列已经用掉约 2.5KB（idle 与 timer 任务不占它 ——
     *   configKERNEL_PROVIDED_STATIC_MEMORY = 1，走内核静态缓冲）。
     *   这个任务要 1KB 栈 + TCB，走动态分配再吃掉约 1.1KB，余量只剩 0.4KB ——
     *   而**堆不够时 xTaskCreate 只是返回失败**（本工程连返回值都没查），
     *   表现是"这个任务从来不存在"＝"SPI 完全没反应"，
     *   与"没接线""C3 没起来"完全无法区分。
     * ★ 放 .bss 代替放堆：F407 有 128KB RAM，这点占用可以忽略；堆的余量和
     *   本次改动之前一模一样；而且静态分配的成败在**编译期**就定了，
     *   不存在"运行时拿不到内存"这条路。
     *   （configSUPPORT_STATIC_ALLOCATION 本来就是 1，没有新增任何配置。） */
    (void)xTaskCreateStatic(vAppOtaTask, "Ota", APP_OTA_TASK_STACK, NULL, 1u,
                            s_ota_stack, &s_ota_tcb);
}

void vAppOtaTask(void *arg)
{
    (void)arg;

    /* ★★ 先等 APP_OTA_START_DELAY_MS 再开第一笔事务 —— 这不是"客气一下"，
     *   它挡掉的后果是**刚升上去的版本被回退**：
     *
     *   vHealthTask 在 2s 时把 APP_HEALTH_MAGIC 写进 APP_HEALTH_ADDR（0x08020018），
     *   而 boot 的 Upd_StartupLaunch() 那处分岔正好拿它当判据（update.c:222/227/244）：
     *     · run 与 B 版本相同            ⇒ 快速启动（常态，与本文件无关）
     *     · run_ver > b_ver 且**标记已写** ⇒ 固化进 B，成为新的好版本
     *     · run_ver > b_ver 且**标记没写** ⇒ 走 else ⇒ **回退 B→run**
     *       也就是把刚升上来、正在跑的这个版本用旧的盖回去（update.c:248）
     *
     *   于是"刚升完级、app 才跑了一秒就被复位"这件事的代价不只是"这次没确认"，
     *   而是"升级成果被撤销" —— 用户会看到刚刷进去的版本自己变回旧的，
     *   而且这种现象只在"刚升完级马上又点升级"时出现，极难与别的原因区分。
     *   而这恰恰是本文件引入了**新的复位路径**之后才可能发生的事：改动之前 app
     *   从不自己复位，走进那条分支只可能是新固件真的起不来（那正是回退要处理的）。
     *
     *   ★ 2500 而不是 2000：两个任务都是 2000ms 醒，谁先跑靠的是"健康任务优先级 4
     *     > 我们 1"这个巧合。这里不想依赖巧合 —— 多等 500ms 零成本。
     *   ★ 代价：这 2.5 秒里 PC 发的帧留在 C3 的接收环里（4096B，一帧 START 才 24B），
     *     第一次轮询就取到了，什么都不丢。 */
    vTaskDelay(pdMS_TO_TICKS(APP_OTA_START_DELAY_MS));

    for (;;)
    {
        if (AppOta_OneTransaction() != 0u)
        {
            /* ★★ 收到字节 ⇒ PC 正对着 C3 说话 ⇒ 它要做 OTA，而现在跑的是 app
             *   （boot 那个 5 秒窗口早就关了）。复位，把控制权交回 boot：
             *   boot 起来后重新打开窗口、自己轮询 SPI，把 PC 那一帧
             *   （以及 PC 因超时重发的那一帧）收下来，升级照常走完。
             *
             * ★ 为什么**不解析帧**、连包头 0xA5 0x5A 都不看：
             *   app 侧不需要"看懂"这个请求，只需要知道"有人来了"。
             *   判断请求合不合法是 boot 的事（它本来就只认真正的 START 帧，
             *   而且只有它会去擦 Flash）。在这里再解析一遍，等于把帧格式这个
             *   跨端契约在 app 里再复制一份，将来改协议就要改三处 ——
             *   而这里**判错**的代价只是多做一次复位，判对的收益却换不来什么。
             *   副作用是"误判"：PC 上用别的 TCP 工具连过来敲几个字节也会让设备
             *   复位一次。代价是几秒钟的重启；好处是行为变得极好预测。
             *
             * ★ 复位前 CS 已经拉高（AppOta_Transfer 的收尾做的），这一笔在
             *   总线上是**完整的一笔**，C3 不会留下半个事务。
             * ★ NVIC_SystemReset 不返回。 */
            NVIC_SystemReset();
        }

        /* ★ 节流必须由任务自己做，不能指望"事务本身耗时"把节奏拖慢：
         *   正常一笔 264B @1.3125MHz 只要 1.61ms（HAL 是阻塞式轮询收发），
         *   放在 100ms 里只占 1.6% 的 CPU —— 不延时就是每秒 600 笔的紧循环。
         *   失败的那一笔则要吃满 100ms 的 HAL 超时，所以坏链路上节奏会自然
         *   降到 5 笔/秒：这没关系，它本来也不该空转。
         *
         * ★ 100ms 这个数的余量（判据来自 bootloader/tools/ota_timing_sim.py）：
         *   上位机的 START_TIMEOUT 现在是 4.5s，而它真正约束的是一个延迟
         *       D = "设备吃掉这一帧 → boot 窗口重新打开"
         *         ≈ 0.2s + 0.3~0.6s（0.2s 是"帧进 C3 的接收环"到"我们在 MISO 上看见"
         *             的最坏延迟：它要先经 C3 的一次 BuildNextTx 才摆上总线，
         *             所以最多跨两笔轮询；后半段是复位 + boot 初始化），
         *   判据是 **T ≥ D**（并且 T ≤ D + 5s，5s 是 BOOT_WAIT_MS）。
         *   100ms 的轮询给 D 只加了 0.1s，对 4.5s 的预算来说是 10 倍以上的余量，
         *   而"更快"没有任何好处：PC 那一帧要么被我们吃掉、要么被 boot 收下，
         *   中间没有第三种状态。 */
        vTaskDelay(pdMS_TO_TICKS(APP_OTA_POLL_MS));
    }
}
