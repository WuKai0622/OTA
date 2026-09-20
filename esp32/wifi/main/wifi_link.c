/* WiFi 连接：AP（自己开热点）/ STA（连路由器），由 wifi_link.h 的 WIFI_USE_AP 切换。
 *
 * 上下文提醒：下面所有事件回调都跑在 **esp_event 的默认事件循环任务**里，
 * 不是 app_main。所以回调里不要做耗时操作，也不要在回调里等网络。 */
#include "wifi_link.h"

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"      /* MACSTR / MAC2STR —— esp_wifi.h 并不包含它，必须显式加 */
#include "esp_netif.h"
#include "esp_wifi.h"

static const char *TAG = "wifi";

static esp_netif_t *s_netif = NULL;

/* 事件任务写、别的任务读，用 volatile 防止编译器把读缓存进寄存器。 */
static volatile uint8_t s_ready = 0;
static char s_ip[16] = "0.0.0.0";

/* 把 netif 当前的 IPv4 地址抓成字符串 */
static void refresh_ip(void)
{
    esp_netif_ip_info_t info;

    if (s_netif != NULL && esp_netif_get_ip_info(s_netif, &info) == ESP_OK)
    {
        snprintf(s_ip, sizeof(s_ip), IPSTR, IP2STR(&info.ip));
    }
}

/* ==================== WiFi 事件 ==================== */
static void on_wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;

    if (base != WIFI_EVENT)
    {
        return;
    }

#if WIFI_USE_AP
    switch (id)
    {
    case WIFI_EVENT_AP_START:
        /* 热点起来了才去读 IP —— 在 esp_wifi_start() 之后立刻读会有竞态 */
        refresh_ip();
        s_ready = 1u;
        ESP_LOGI(TAG, "热点已启动  SSID=%s  密码=%s  本机 IP=%s",
                 WIFI_AP_SSID, WIFI_AP_PASS, s_ip);
        ESP_LOGI(TAG, "→ PC 连上这个热点后，连 %s:%d", s_ip, 5001);
        break;

    case WIFI_EVENT_AP_STACONNECTED: {
        wifi_event_ap_staconnected_t *e = (wifi_event_ap_staconnected_t *)data;
        ESP_LOGI(TAG, "有设备连入  MAC=" MACSTR, MAC2STR(e->mac));
        break;
    }

    case WIFI_EVENT_AP_STADISCONNECTED: {
        wifi_event_ap_stadisconnected_t *e = (wifi_event_ap_stadisconnected_t *)data;
        ESP_LOGW(TAG, "设备断开  MAC=" MACSTR, MAC2STR(e->mac));
        break;
    }

    default:
        break;
    }
#else
    switch (id)
    {
    case WIFI_EVENT_STA_START:
        ESP_LOGI(TAG, "STA 启动，正在连接 \"%s\" ...", WIFI_STA_SSID);
        esp_wifi_connect();
        break;

    case WIFI_EVENT_STA_DISCONNECTED: {
        wifi_event_sta_disconnected_t *e = (wifi_event_sta_disconnected_t *)data;
        /* 掉线就重连。reason 见 esp_wifi_types.h，最常遇到的是
         * 201(NO_AP_FOUND 密码/SSID 错) 与 205(CONNECTION_FAIL)。 */
        s_ready = 0u;
        ESP_LOGW(TAG, "断开（reason=%d），1s 后重连 ...", e->reason);
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_wifi_connect();
        break;
    }

    default:
        break;
    }
#endif
}

/* ==================== IP 事件（只有 STA 会用到） ==================== */
static void on_ip_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;

    if (base != IP_EVENT)
    {
        return;
    }

#if !WIFI_USE_AP
    if (id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
        snprintf(s_ip, sizeof(s_ip), IPSTR, IP2STR(&e->ip_info.ip));
        s_ready = 1u;
        ESP_LOGI(TAG, "已连上路由器，拿到 IP=%s", s_ip);
        ESP_LOGI(TAG, "→ PC 连到同一个路由器后，连 %s:%d", s_ip, 5001);
    }
#else
    (void)id;
    (void)data;
#endif
}

/* ==================== 初始化 ==================== */
void Wifi_Link_Init(void)
{
    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    wifi_config_t      wc;

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

#if WIFI_USE_AP
    s_netif = esp_netif_create_default_wifi_ap();
#else
    s_netif = esp_netif_create_default_wifi_sta();
#endif

    ESP_ERROR_CHECK(esp_wifi_init(&init_cfg));

    /* 用 ESP_EVENT_ANY_ID 一次注册：事件 id 不多，比逐个注册清楚 */
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, on_wifi_event, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, ESP_EVENT_ANY_ID, on_ip_event, NULL, NULL));

    memset(&wc, 0, sizeof(wc));

#if WIFI_USE_AP
    /* snprintf 而不是 strncpy：ssid/password 都是定长数组，用 strncpy 触发
     * -Wstringop-truncation 警告，而且不能保证结尾有 '\0'。 */
    snprintf((char *)wc.ap.ssid, sizeof(wc.ap.ssid), "%s", WIFI_AP_SSID);
    snprintf((char *)wc.ap.password, sizeof(wc.ap.password), "%s", WIFI_AP_PASS);
    wc.ap.ssid_len       = strlen(WIFI_AP_SSID);
    wc.ap.channel        = WIFI_AP_CHANNEL;
    wc.ap.max_connection = WIFI_AP_MAX_CONN;
    /* 密码不足 8 位时 WPA2 会启动失败，此时退化成开放网络 */
    wc.ap.authmode       = (strlen(WIFI_AP_PASS) >= 8) ? WIFI_AUTH_WPA2_PSK
                                                       : WIFI_AUTH_OPEN;
    wc.ap.pmf_cfg.required = false;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wc));
#else
    snprintf((char *)wc.sta.ssid, sizeof(wc.sta.ssid), "%s", WIFI_STA_SSID);
    snprintf((char *)wc.sta.password, sizeof(wc.sta.password), "%s", WIFI_STA_PASS);
    wc.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
#endif

    ESP_ERROR_CHECK(esp_wifi_start());
}

uint8_t Wifi_Link_Ready(void)
{
    return s_ready;
}

const char *Wifi_Link_IpStr(void)
{
    return s_ip;
}
