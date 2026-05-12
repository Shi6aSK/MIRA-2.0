#include "sd_card.h"
#include "vision_config.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdspi_host.h"
#include "driver/spi_master.h"
#include <sys/stat.h>
#include <string.h>
#include <stdio.h>

static const char *TAG  = "sd";
#define MOUNT_POINT "/sdcard"

static bool s_mounted = false;

bool sd_is_mounted(void) { return s_mounted; }

bool sd_init(void)
{
    esp_vfs_fat_sdmmc_mount_config_t mcfg = {
        .format_if_mount_failed = false,
        .max_files              = 8,
        .allocation_unit_size   = 16 * 1024,
    };

    spi_bus_config_t bus = {
        .mosi_io_num     = SD_MOSI_GPIO,
        .miso_io_num     = SD_MISO_GPIO,
        .sclk_io_num     = SD_CLK_GPIO,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = 4096,
    };

    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "SPI bus init failed: 0x%x", ret);
        return false;
    }

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.max_freq_khz  = 4000;  /* 4 MHz – safe conservative speed */

    sdspi_device_config_t slot = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot.gpio_cs  = SD_CS_GPIO;
    slot.host_id  = SPI2_HOST;

    sdmmc_card_t *card = NULL;
    ret = esp_vfs_fat_sdspi_mount(MOUNT_POINT, &host, &slot, &mcfg, &card);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "SD mount failed (0x%x) – training saves disabled", ret);
        spi_bus_free(SPI2_HOST);
        return false;
    }

    s_mounted = true;
    ESP_LOGI(TAG, "SD mounted at %s  (%.1f MB)",
             MOUNT_POINT,
             (float)((uint64_t)card->csd.capacity * card->csd.sector_size)
             / (1024.0f * 1024.0f));
    return true;
}

bool sd_mkdir(const char *path)
{
    if (!s_mounted || !path) return false;
    char buf[128];
    int n = snprintf(buf, sizeof(buf), "%s%s", MOUNT_POINT, path);
    if (n <= 0 || n >= (int)sizeof(buf)) return false;

    /* Walk and create each directory component */
    int base = (int)strlen(MOUNT_POINT);
    for (int i = base + 1; i <= n; i++) {
        if (buf[i] == '/' || buf[i] == '\0') {
            char save = buf[i];
            buf[i] = '\0';
            mkdir(buf, 0755);
            buf[i] = save;
        }
    }
    return true;
}

bool sd_save_bytes(const char *path, const uint8_t *data, size_t len)
{
    if (!s_mounted || !path || !data || len == 0) return false;
    char full[128];
    int n = snprintf(full, sizeof(full), "%s%s", MOUNT_POINT, path);
    if (n <= 0 || n >= (int)sizeof(full)) return false;

    FILE *f = fopen(full, "wb");
    if (!f) { ESP_LOGE(TAG, "Cannot open %s", full); return false; }
    size_t written = fwrite(data, 1, len, f);
    fclose(f);
    if (written != len) {
        ESP_LOGE(TAG, "Short write %s (%u/%u)", full, (unsigned)written, (unsigned)len);
        return false;
    }
    return true;
}
