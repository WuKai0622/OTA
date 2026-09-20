/* ==================== ESP32-C3 OTA 桥接 ====================
 * 把三条链路串起来：PC ←WiFi/TCP→ C3 ←SPI→ F407，双向字节流透传。
 *
 * ★ 本固件**完全不涉及 OTA**，也永远不需要涉及：它只是一条与内容无关的
 *   字节管道。F407 那头从"回显自检"换成"真 OTA"时，这里一行都没改。
 *
 * 分步履历：① WiFi+TCP echo（四用例全 PASS）→ ② SPI 从机回环
 * （F407 侧自检 `PHY OK`）→ ③ SPI↔TCP 透传 + F407 回显（端到端字节流无损）
 * → ④ F407 侧接真 OTA + 会话绑定（本固件不变）。
 */
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"     /* esp_reset_reason() —— 上电原因探针，见 app_main 开头 */
#include "nvs_flash.h"

#include "spi_bridge.h"
#include "tcp_bridge.h"
#include "wifi_link.h"

static const char *TAG = "app";

void app_main(void)
{
    /* ★★ 复位原因：**上电第一件事**就查，晚于任何初始化都会被后续日志淹没。
     *
     * 为什么非要打这一行：C3 峰值电流约 350mA 而 WiFi 发射是脉冲式的，若它和
     * F407 共用一路 3.3V，轨/地会被拉塌 —— 表现是**只在跑测试时**间歇性地出现
     * SPI 帧错位或丢字节（2026-09-19 实测：共用电源时 `stale=310`、字节对不上；
     * 独立供电时四项坏帧计数全 0、字节账一分不差）。
     * 那种情况下最要紧的问题是"到底是**电源把芯片打复位了**，还是**只是信号被
     * 噪声打坏了**" —— 两者的修法完全不同（前者换供电，后者查接线/降时钟）。
     * 靠现象猜不可靠：复位会断 TCP，而纯信号噪声不会。
     * 这一行直接把答案说出来：BROWNOUT / 上电 / 软件 / 看门狗，一目了然。
     * （C3 真复位过的话，下面还会再打一次全部启动日志 —— 两行对照即知。） */
    {
        esp_reset_reason_t rr = esp_reset_reason();
        const char *why =
            (rr == ESP_RST_POWERON)   ? "上电" :
            (rr == ESP_RST_BROWNOUT)  ? "★ 布朗复位（供电被拉塌）—— 换独立供电" :
            (rr == ESP_RST_SW)        ? "软件复位" :
            (rr == ESP_RST_PANIC)     ? "★ 异常/看门狗复位" :
            (rr == ESP_RST_TASK_WDT)  ? "★ 任务看门狗复位" :
            (rr == ESP_RST_INT_WDT)   ? "★ 中断看门狗复位" :
            (rr == ESP_RST_DEEPSLEEP) ? "深睡唤醒" :
            (rr == ESP_RST_EXT)       ? "外部复位脚" : "其他";
        ESP_LOGI(TAG, "本次上电原因：%s（code=%d）", why, (int)rr);
    }

    /* WiFi 驱动要靠 NVS 存校准数据和配置，必须最先初始化。
     * 首次烧录或换过分区表后 NVS 可能不合法，按官方套路擦掉重来。 */
    esp_err_t r = nvs_flash_init();
    if (r == ESP_ERR_NVS_NO_FREE_PAGES || r == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_LOGW(TAG, "NVS 需要重新初始化，擦除中 ...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        r = nvs_flash_init();
    }
    ESP_ERROR_CHECK(r);

    /* 顺序不能反，三个都有依赖：
     *   ① Wifi_Link_Init() 内部会 esp_netif_init()，不先做后面 socket() 直接失败
     *   ② Spi_Bridge_Start() 建两个环，TCP 任务要用
     *   ③ Tcp_Bridge_Start() 才能起 accept 循环 */
    Wifi_Link_Init();
    Spi_Bridge_Start();
    Tcp_Bridge_Start();

    /* wifi_link.c 是在 WIFI_EVENT_AP_START 回调里才去读 IP 的（那里读没有竞态），
     * 所以 Wifi_Link_Init() 刚返回时 IP 还是 "0.0.0.0"。
     * 这里最多等 5 秒，让下面那行日志打出真实 IP。
     * 只影响日志好看与否 —— TCP 服务器绑的是 INADDR_ANY，不依赖这个。 */
    for (int i = 0; i < 50 && !Wifi_Link_Ready(); i++)
    {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    /* 上电顺序提示：F407 的 boot 只有 5 秒等待窗口，而 C3 的 STA 连接 + DHCP
     * 要 2~5 秒。所以正常顺序是「先给 ESP 上电（或复位），再复位 F407」——
     * 反过来的话窗口期可能赶不上，PC 连上来时 boot 已经去跳 app 了。 */
    ESP_LOGI(TAG, "桥已启动：PC 连 %s:%d 即可做 OTA（或任何字节流）",
             Wifi_Link_IpStr(), TCP_BRIDGE_PORT);

    /* 两个任务已经在跑，app_main 可以直接返回（ESP-IDF 允许） */
}
