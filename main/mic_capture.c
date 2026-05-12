#include "mic_capture.h"
#include "vision_config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s_pdm.h"
#include "driver/i2s_common.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/stat.h>

static const char *TAG = "mic";

#define MIC_SAMPLE_RATE   16000
#define MIC_BITS          16
#define MIC_CHANNELS      1

static i2s_chan_handle_t s_rx_chan = NULL;
static bool s_inited = false;
static volatile bool s_busy = false;

esp_err_t mic_init(void)
{
    if (s_inited) return ESP_OK;

    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num  = 8;
    chan_cfg.dma_frame_num = 512;
    esp_err_t err = i2s_new_channel(&chan_cfg, NULL, &s_rx_chan);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2s_new_channel failed: %s", esp_err_to_name(err));
        return err;
    }

    i2s_pdm_rx_config_t pdm_cfg = {
        .clk_cfg  = I2S_PDM_RX_CLK_DEFAULT_CONFIG(MIC_SAMPLE_RATE),
        .slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                    I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .clk = MIC_CLK_GPIO,
            .din = MIC_DATA_GPIO,
            .invert_flags = { .clk_inv = false },
        },
    };
    err = i2s_channel_init_pdm_rx_mode(s_rx_chan, &pdm_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2s_channel_init_pdm_rx_mode failed: %s", esp_err_to_name(err));
        i2s_del_channel(s_rx_chan);
        s_rx_chan = NULL;
        return err;
    }

    err = i2s_channel_enable(s_rx_chan);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2s_channel_enable failed: %s", esp_err_to_name(err));
        i2s_del_channel(s_rx_chan);
        s_rx_chan = NULL;
        return err;
    }

    s_inited = true;
    ESP_LOGI(TAG, "PDM mic ready (clk=GPIO%d data=GPIO%d %dHz)",
             MIC_CLK_GPIO, MIC_DATA_GPIO, MIC_SAMPLE_RATE);
    return ESP_OK;
}

/* ── WAV writer ───────────────────────────────────────────────── */
static void write_wav_header(FILE *f, uint32_t num_samples)
{
    uint32_t byte_rate  = MIC_SAMPLE_RATE * MIC_CHANNELS * (MIC_BITS / 8);
    uint16_t block_align = (uint16_t)(MIC_CHANNELS * (MIC_BITS / 8));
    uint32_t data_size  = num_samples * MIC_CHANNELS * (MIC_BITS / 8);
    uint32_t chunk_size = 36 + data_size;

    fwrite("RIFF", 1, 4, f);
    fwrite(&chunk_size, 4, 1, f);
    fwrite("WAVE", 1, 4, f);
    fwrite("fmt ", 1, 4, f);
    uint32_t sub1 = 16;  fwrite(&sub1, 4, 1, f);
    uint16_t fmt  =  1;  fwrite(&fmt,  2, 1, f);   /* PCM */
    uint16_t ch   = MIC_CHANNELS; fwrite(&ch, 2, 1, f);
    uint32_t sr   = MIC_SAMPLE_RATE; fwrite(&sr, 4, 1, f);
    fwrite(&byte_rate,   4, 1, f);
    fwrite(&block_align, 2, 1, f);
    uint16_t bps  = MIC_BITS; fwrite(&bps, 2, 1, f);
    fwrite("data", 1, 4, f);
    fwrite(&data_size,   4, 1, f);
}

/* ── Capture task ─────────────────────────────────────────────── */
typedef struct { int duration_ms; } cap_args_t;

static void capture_task(void *arg)
{
    cap_args_t *ca = (cap_args_t *)arg;
    int dur_ms = ca->duration_ms;
    free(ca);

    if (!s_inited) {
        ESP_LOGE(TAG, "Mic not initialised");
        s_busy = false;
        vTaskDelete(NULL);
        return;
    }

    /* Make sure recordings directory exists */
    mkdir("/sdcard/recordings", 0777);

    /* Find next filename */
    static int s_file_idx = 0;
    char path[64];
    snprintf(path, sizeof(path), "/sdcard/recordings/%04d.wav", s_file_idx++);

    uint32_t num_samples  = (uint32_t)((MIC_SAMPLE_RATE * dur_ms) / 1000);
    uint32_t total_bytes  = num_samples * MIC_CHANNELS * (MIC_BITS / 8);

    /* Allocate DMA-friendly chunk buffer (4 KB) */
    const size_t CHUNK = 4096;
    int16_t *buf = malloc(CHUNK);
    if (!buf) {
        ESP_LOGE(TAG, "OOM for mic buffer");
        s_busy = false;
        vTaskDelete(NULL);
        return;
    }

    FILE *f = fopen(path, "wb");
    if (!f) {
        ESP_LOGE(TAG, "Cannot open %s", path);
        free(buf);
        s_busy = false;
        vTaskDelete(NULL);
        return;
    }

    /* Write placeholder header – will patch after we know exact sample count */
    write_wav_header(f, num_samples);

    uint32_t written = 0;
    ESP_LOGI(TAG, "Recording %d ms → %s", dur_ms, path);
    while (written < total_bytes) {
        size_t want = CHUNK;
        if (want > total_bytes - written) want = total_bytes - written;
        size_t got = 0;
        esp_err_t err = i2s_channel_read(s_rx_chan, buf, want, &got, pdMS_TO_TICKS(500));
        if (err != ESP_OK || got == 0) break;
        fwrite(buf, 1, got, f);
        written += (uint32_t)got;
    }

    /* Patch actual sizes in header */
    uint32_t actual_samples = written / (MIC_CHANNELS * (MIC_BITS / 8));
    fseek(f, 0, SEEK_SET);
    write_wav_header(f, actual_samples);
    fclose(f);
    free(buf);

    ESP_LOGI(TAG, "Saved %lu bytes to %s", (unsigned long)written, path);
    s_busy = false;
    vTaskDelete(NULL);
}

bool mic_is_busy(void) { return s_busy; }

void mic_capture_async(int duration_ms)
{
    if (s_busy) {
        ESP_LOGW(TAG, "Mic already busy – skipping");
        return;
    }
    s_busy = true;
    cap_args_t *ca = malloc(sizeof(cap_args_t));
    if (!ca) { s_busy = false; return; }
    ca->duration_ms = duration_ms;
    if (xTaskCreate(capture_task, "mic_cap", 4096, ca, 3, NULL) != pdPASS) {
        free(ca);
        s_busy = false;
    }
}
