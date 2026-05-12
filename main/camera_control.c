#include "camera_control.h"
#include "vision_config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "cam";

esp_err_t camera_init(void)
{
    camera_config_t cfg = {
        .pin_pwdn  = PWDN_GPIO_NUM,
        .pin_reset = RESET_GPIO_NUM,
        .pin_xclk  = XCLK_GPIO_NUM,
        .pin_sccb_sda = SIOD_GPIO_NUM,
        .pin_sccb_scl = SIOC_GPIO_NUM,
        .pin_d7 = Y9_GPIO_NUM,  .pin_d6 = Y8_GPIO_NUM,
        .pin_d5 = Y7_GPIO_NUM,  .pin_d4 = Y6_GPIO_NUM,
        .pin_d3 = Y5_GPIO_NUM,  .pin_d2 = Y4_GPIO_NUM,
        .pin_d1 = Y3_GPIO_NUM,  .pin_d0 = Y2_GPIO_NUM,
        .pin_vsync = VSYNC_GPIO_NUM,
        .pin_href  = HREF_GPIO_NUM,
        .pin_pclk  = PCLK_GPIO_NUM,
        .xclk_freq_hz  = 20000000,  /* 20 MHz – standard XCLK for OV3660; 16 MHz caused
                                       sub-optimal PLL lock in RGB565 mode */
        .ledc_timer    = LEDC_TIMER_0,
        .ledc_channel  = LEDC_CHANNEL_0,
        .pixel_format  = PIXFORMAT_RGB565,
        .frame_size    = FRAMESIZE_QVGA,      /* 320x240 – OV3660 native; 240x240 not supported */
        .jpeg_quality  = JPEG_QUALITY,
        .fb_count      = 2,
        .grab_mode     = CAMERA_GRAB_LATEST,      /* discard stale frames – prevents EV-VSYNC-OVF DMA overflow */
        .fb_location   = CAMERA_FB_IN_PSRAM,
    };

    esp_err_t err = esp_camera_init(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed: %s", esp_err_to_name(err));
        return err;
    }

    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        /* OV3660 outputs correctly-oriented data on the XIAO ESP32S3 Sense board.
         * No vflip/hmirror needed; the original CSS rotate(180deg) in the web UI
         * was erroneously flipping an already-correct image. */

        /* Auto-exposure / auto-gain / auto-white-balance */
        s->set_exposure_ctrl(s, 1);   // AEC enable
        s->set_aec2(s, 1);            // AEC DSP enable
        s->set_ae_level(s, 2);        // +2 exposure bias for brighter indoor image
        s->set_aec_value(s, 600);     // seed value
        s->set_gain_ctrl(s, 1);       // AGC enable
        s->set_agc_gain(s, 2);        // lower start gain – less noise
        s->set_whitebal(s, 1);        // AWB enable
        s->set_awb_gain(s, 1);        // AWB gain enable
        s->set_wb_mode(s, 0);         // auto WB

        /* Image quality – Seeed-official OV3660 values + denoise */
        s->set_brightness(s, 2);      // +2 for better indoor exposure
        s->set_contrast(s, 1);        // +1
        s->set_saturation(s, -2);     // -2 (OV3660 oversaturates)
        s->set_sharpness(s, 3);       // max (range -3 to +3 per driver source)
        s->set_denoise(s, 2);         // light denoise (OV3660-specific, range 0-8)
        s->set_special_effect(s, 0);  // no colour effect
        s->set_bpc(s, 1);             // bad-pixel correction
        s->set_wpc(s, 1);             // white-pixel correction
        s->set_raw_gma(s, 1);         // raw gamma
        s->set_lenc(s, 1);            // lens correction
        s->set_dcw(s, 1);             // downscale+crop
        s->set_gainceiling(s, (gainceiling_t)2); // 8x ceiling – lower gain = less noise
        s->set_colorbar(s, 0);

        vTaskDelay(pdMS_TO_TICKS(2500));
    }

    ESP_LOGW(TAG, "Camera ready %dx%d RGB565", FRAME_WIDTH, FRAME_HEIGHT);
    return ESP_OK;
}

camera_fb_t *camera_capture(void) { return esp_camera_fb_get(); }
void         camera_return(camera_fb_t *fb) { esp_camera_fb_return(fb); }
