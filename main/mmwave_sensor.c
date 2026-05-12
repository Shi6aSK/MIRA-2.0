/*
 * mmwave_sensor.c
 * ---------------
 * UART driver for Seeed 24 GHz mmWave MR24HPC1.
 *
 * The sensor outputs a proprietary binary frame at 115200 baud.
 * This driver:
 *   1. Opens UART1 (configurable) and installs a ring-buffer RX driver.
 *   2. Spawns a background FreeRTOS task (mmwave_rx_task) pinned to Core 0.
 *   3. Parses each incoming frame, validates CRC-8, extracts presence state.
 *   4. Stores the result in a mutex-protected mmwave_reading_t struct.
 *   5. Increments a drop counter on framing errors or timeouts.
 *
 * CRC-8 polynomial: 0x31 (Maxim / Dallas 1-Wire).
 * The sensor manufacturer uses this CRC over the payload bytes only.
 *
 * Stale detection: if no valid frame arrives within MMWAVE_STALE_MS,
 *   mmwave_is_stale() returns true so the fault-tolerance layer can act.
 */

#include "mmwave_sensor.h"
#include "vision_config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>

static const char *TAG = "mmwave";

/* -- Frame constants ---------------------------------------------- */
#define FRAME_SOF          0x53
#define FRAME_EOF          0x54
#define FRAME_MAX_LEN      64
#define RX_BUF_SIZE        256

/* Presence type bytes (Type field in frame) */
#define TYPE_NO_ONE        0x01
#define TYPE_MOTION        0x02
#define TYPE_STATIC        0x03

/* Stale threshold: if no frame arrives in this many ms, declare stale */
#define MMWAVE_STALE_MS    500

/* Max consecutive dropped frames before we log a transport fault */
#define MMWAVE_DROP_WARN   5

/* -- Internal state ----------------------------------------------- */
static mmwave_reading_t  s_reading  = {0};
static SemaphoreHandle_t s_mutex    = NULL;
static uint32_t          s_drops    = 0;

/* -- CRC-8 (poly 0x31, init 0x00) -------------------------------- */
static uint8_t crc8(const uint8_t *data, int len)
{
    uint8_t crc = 0x00;
    for (int i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
    }
    return crc;
}

/* -- Parse one complete frame buffer ------------------------------ */
static void parse_frame(const uint8_t *buf, int len)
{
    if (len < 6) { s_drops++; return; }           /* minimum: SOF+len2+type+head+crc+EOF */
    if (buf[0] != FRAME_SOF || buf[len-1] != FRAME_EOF) { s_drops++; return; }

    int payload_len = (int)buf[1] | ((int)buf[2] << 8);
    if (payload_len < 0 || payload_len + 5 > len) { s_drops++; return; }

    uint8_t type     = buf[3];
    /* buf[4] = head byte */
    const uint8_t *payload = buf + 5;
    uint8_t expected_crc   = buf[len - 2];   /* CRC is 2nd-to-last byte */
    uint8_t calc_crc       = crc8(buf + 3, len - 4); /* CRC over type..data */

    if (calc_crc != expected_crc) {
        ESP_LOGW(TAG, "CRC mismatch: got 0x%02x expected 0x%02x", expected_crc, calc_crc);
        s_drops++;
        return;
    }

    mmwave_reading_t r;
    memset(&r, 0, sizeof(r));
    r.timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000);
    r.frame_valid  = true;
    r.drop_count   = s_drops;
    r.raw_energy   = (payload_len > 0) ? payload[0] : 0;

    switch (type) {
        case TYPE_NO_ONE: r.state = MMWAVE_STATE_NO_ONE; break;
        case TYPE_MOTION: r.state = MMWAVE_STATE_MOTION; break;
        case TYPE_STATIC: r.state = MMWAVE_STATE_STATIC; break;
        default:          r.state = MMWAVE_STATE_UNKNOWN; break;
    }

    /* Reset drop counter on any valid frame */
    s_drops = 0;

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_reading = r;
    xSemaphoreGive(s_mutex);

    ESP_LOGD(TAG, "state=%d energy=%d ts=%lu", (int)r.state, r.raw_energy, (unsigned long)r.timestamp_ms);
}

/* -- Background RX task ------------------------------------------- */
static void mmwave_rx_task(void *arg)
{
    uint8_t  buf[FRAME_MAX_LEN];
    uint8_t  byte;
    int      idx = 0;
    bool     in_frame = false;

    for (;;) {
        int rx = uart_read_bytes(MMWAVE_UART_NUM, &byte, 1, pdMS_TO_TICKS(200));

        if (rx <= 0) {
            /* Timeout - check if we should mark stale */
            uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
            xSemaphoreTake(s_mutex, portMAX_DELAY);
            uint32_t last_ts = s_reading.timestamp_ms;
            xSemaphoreGive(s_mutex);
            if (last_ts > 0 && (now - last_ts) > MMWAVE_STALE_MS) {
                s_drops++;
                if (s_drops % MMWAVE_DROP_WARN == 0)
                    ESP_LOGW(TAG, "No radar frame for %lu ms (drops=%lu)",
                             (unsigned long)(now - last_ts), (unsigned long)s_drops);
            }
            in_frame = false;
            idx = 0;
            continue;
        }

        if (!in_frame) {
            if (byte == FRAME_SOF) {
                buf[0]   = byte;
                idx      = 1;
                in_frame = true;
            }
            continue;
        }

        if (idx < FRAME_MAX_LEN) {
            buf[idx++] = byte;
        }

        if (byte == FRAME_EOF) {
            parse_frame(buf, idx);
            in_frame = false;
            idx      = 0;
        }
    }
}

/* -- Public API --------------------------------------------------- */

esp_err_t mmwave_init(void)
{
    s_mutex = xSemaphoreCreateMutex();
    if (!s_mutex) return ESP_ERR_NO_MEM;

    uart_config_t cfg = {
        .baud_rate  = MMWAVE_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(MMWAVE_UART_NUM, RX_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(MMWAVE_UART_NUM, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(MMWAVE_UART_NUM, MMWAVE_TX_GPIO, MMWAVE_RX_GPIO,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    xTaskCreatePinnedToCore(mmwave_rx_task, "mmwave_rx", 4096, NULL,
                            configMAX_PRIORITIES - 2, NULL, 0 /* Core 0 */);

    ESP_LOGI(TAG, "mmWave UART%d ready (TX=%d RX=%d)",
             MMWAVE_UART_NUM, MMWAVE_TX_GPIO, MMWAVE_RX_GPIO);
    return ESP_OK;
}

mmwave_reading_t mmwave_get_reading(void)
{
    mmwave_reading_t r;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    r = s_reading;
    xSemaphoreGive(s_mutex);
    return r;
}

bool mmwave_presence_detected(void)
{
    mmwave_reading_t r = mmwave_get_reading();
    if (!r.frame_valid) return false;
    uint32_t age = (uint32_t)(esp_timer_get_time() / 1000) - r.timestamp_ms;
    if (age > MMWAVE_STALE_MS) return false;
    return (r.state == MMWAVE_STATE_MOTION || r.state == MMWAVE_STATE_STATIC);
}

bool mmwave_is_stale(void)
{
    mmwave_reading_t r = mmwave_get_reading();
    if (!r.frame_valid || r.timestamp_ms == 0) return true;
    uint32_t age = (uint32_t)(esp_timer_get_time() / 1000) - r.timestamp_ms;
    return age > MMWAVE_STALE_MS;
}

uint32_t mmwave_drop_count(void)
{
    return s_drops;
}
