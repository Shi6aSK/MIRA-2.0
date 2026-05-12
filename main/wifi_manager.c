#include <string.h>
#include "wifi_manager.h"
#include "vision_config.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "lwip/ip4_addr.h"

static const char *TAG = "wifi";
static EventGroupHandle_t s_wifi_evt;
static char s_ip[20] = "0.0.0.0";
static int s_retry = 0;

#define CONNECTED_BIT BIT0
#define FAIL_BIT      BIT1

static void on_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry < WIFI_MAX_RETRY) {
            esp_wifi_connect();
            s_retry++;
            ESP_LOGI(TAG, "Retry %d/%d...", s_retry, WIFI_MAX_RETRY);
        } else {
            xEventGroupSetBits(s_wifi_evt, FAIL_BIT);
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
        snprintf(s_ip, sizeof(s_ip), IPSTR, IP2STR(&e->ip_info.ip));
        ESP_LOGW(TAG, "Connected – IP: %s", s_ip);
        s_retry = 0;
        xEventGroupSetBits(s_wifi_evt, CONNECTED_BIT);
    }
}

esp_err_t wifi_init_sta(void)
{
    s_wifi_evt = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t h1, h2;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &on_event, NULL, &h1));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &on_event, NULL, &h2));

    wifi_config_t wcfg = {};
    strncpy((char *)wcfg.sta.ssid,     WIFI_SSID, sizeof(wcfg.sta.ssid) - 1);
    strncpy((char *)wcfg.sta.password, WIFI_PASS,  sizeof(wcfg.sta.password) - 1);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wcfg));
    ESP_ERROR_CHECK(esp_wifi_start());
    esp_wifi_connect();

    EventBits_t bits = xEventGroupWaitBits(
        s_wifi_evt, CONNECTED_BIT | FAIL_BIT,
        pdFALSE, pdFALSE, pdMS_TO_TICKS(20000));

    if (bits & CONNECTED_BIT) return ESP_OK;
    ESP_LOGE(TAG, "Failed to connect to %s", WIFI_SSID);
    return ESP_FAIL;
}

const char *wifi_get_ip(void) { return s_ip; }
