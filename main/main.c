/*
 * main.c -- MIRA 2.0 (CPRE 5450 Fault-Tolerant Edition)
 * Source: https://github.com/Shi6aSK/MIRA-2.0
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_task_wdt.h"
#include "img_converters.h"
#include <string.h>

#include "esp_camera.h"
#include "vision_config.h"
#include "camera_control.h"
#include "vision_pipeline.h"
#include "web_server.h"
#include "wifi_manager.h"
#include "servo_control.h"
#include "oled_control.h"
#include "sd_card.h"
#include "training.h"
#include "mic_capture.h"

/* -- CPRE 5450 additions ---------------------------------------- */
#include "mmwave_sensor.h"
#include "ft_activation.h"

static const char *TAG = "app";

static uint8_t *s_frame_copy  = NULL;
static int      s_null_streak = 0;

/* -- Gesture auto-trigger state ---------------------------------- */
static char     s_prev_gesture[16]    = "none";
static int      s_gesture_count       = 0;
static uint32_t s_last_trigger_ms     = 0;

/* -- FT diagnosis task -----------------------------------------
 * Runs on Core 0 at medium priority.  Gathers mmWave + camera
 * health observations and drives the FT state machine.
 * --------------------------------------------------------------- */
static void ft_task(void *arg)
{
    esp_task_wdt_add(NULL);
    static uint32_t s_last_fid  = 0;
    static int      s_fid_stall = 0;
    static ft_state_t s_prev_state = FT_IDLE;

    for (;;) {
        esp_task_wdt_reset();

        bool radar_present = mmwave_presence_detected();
        bool radar_stale   = mmwave_is_stale();

        detection_t det  = vision_get_detection();
        bool cam_present = det.object_present;
        if (det.frame_id == s_last_fid) {
            s_fid_stall++;
        } else {
            s_fid_stall = 0;
            s_last_fid  = det.frame_id;
        }
        bool cam_stale = (s_fid_stall > 3);

        ft_update(radar_present, radar_stale, cam_present, cam_stale, /*wdt_ok=*/true);

        ft_status_t st = ft_get_status();
        if (st.state != s_prev_state) {
            ESP_LOGI(TAG, "[FT] %s  conf=%.2f  mismatch=%lu  radar=%s  cam=%s",
                     ft_state_name(st.state),
                     (double)st.fused_confidence,
                     (unsigned long)st.mismatch_count,
                     radar_present ? "PRESENT" : "absent",
                     cam_present   ? "PRESENT" : "absent");
            s_prev_state = st.state;
        }
        vTaskDelay(pdMS_TO_TICKS(FT_TASK_INTERVAL_MS));
    }
}

static void vision_task(void *arg)
{
    for (;;) {
        camera_fb_t *fb = camera_capture();
        if (!fb) {
            /* Camera timed out - restart DMA if it happens repeatedly */
            if (++s_null_streak >= 3) {  /* reinit after 3?4 s DMA timeouts = 12 s */
                ESP_LOGW(TAG, "Camera stalled - reinitialising");
                esp_camera_deinit();
                vTaskDelay(pdMS_TO_TICKS(200));
                camera_init();
                s_null_streak = 0;
            }
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        s_null_streak = 0;
        if (fb && s_frame_copy) {
            // Skip truncated frames - FB-SIZE mismatch crashes fmt2jpg
            if (fb->len != (size_t)(FRAME_WIDTH * FRAME_HEIGHT * 2)) {
                camera_return(fb);
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }

            /* While mic is recording, I2S DMA and camera DMA both hit PSRAM
             * simultaneously and saturate the bus, causing EV-VSYNC-OVF.
             * Drain the camera frame immediately and throttle to ~2 fps
             * so the DMA bus is not overloaded. */
            if (mic_is_busy()) {
                camera_return(fb);
                vTaskDelay(pdMS_TO_TICKS(500));
                continue;
            }
            // Copy pixels and release the camera buffer BEFORE detection
            size_t len = fb->len;
            int w = (int)fb->width, h = (int)fb->height;
            memcpy(s_frame_copy, fb->buf, len);
            camera_return(fb);          // return ASAP - frees the DMA buffer

            // Encode JPEG from PSRAM copy and push to HTTP shared buffer.
            // This happens before inference so /frame is always fresh.
            {
                uint8_t *jpg = NULL; size_t jpg_len = 0;
                if (fmt2jpg(s_frame_copy, len, (uint16_t)w, (uint16_t)h,
                            PIXFORMAT_RGB565, STREAM_JPEG_QUALITY, &jpg, &jpg_len)) {
                    web_server_update_frame(jpg, jpg_len);
                    free(jpg);
                }
            }

            /* -- FT gate --------------------------------------- */
            if (!ft_activation_allowed()) {
                ESP_LOGV(TAG, "FT gate: inference suppressed (%s)",
                         ft_state_name(ft_get_status().state));
                vTaskDelay(pdMS_TO_TICKS(100));
                continue;
            }

            // Run (slow) inference on the PSRAM copy
            camera_fb_t fake;
            memset(&fake, 0, sizeof(fake));
            fake.buf    = s_frame_copy;
            fake.len    = len;
            fake.width  = (size_t)w;
            fake.height = (size_t)h;
            fake.format = PIXFORMAT_RGB565;
            vision_process_frame(&fake);

            // Training hooks (no-ops when TRAIN_IDLE)
            detection_t det = vision_get_detection();
            training_maybe_capture_face(&fake, &det);
            training_maybe_capture_gesture(&det);

            /* -- Gesture auto-trigger ---------------------------- */
            /* Only fire when KNN templates exist - prevents false positives
             * from the skin-blob detector before any gesture training. */
            if (gesture_knn_count() > 0) {
                if (strcmp(det.gesture, s_prev_gesture) == 0) {
                    s_gesture_count++;
                } else {
                    strncpy(s_prev_gesture, det.gesture, sizeof(s_prev_gesture) - 1);
                    s_gesture_count = 1;
                }
                uint32_t now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
                if (s_gesture_count == GESTURE_CONFIRM_FRAMES &&
                    (now_ms - s_last_trigger_ms) >= GESTURE_COOLDOWN_MS) {

                    if (strcmp(det.gesture, "point") == 0) {
                        ESP_LOGI(TAG, "point gesture -> Gemma snap");
                        web_server_auto_trigger_gemma("point");
                        s_last_trigger_ms = now_ms;
                    } else if (strcmp(det.gesture, "open_palm") == 0) {
                        ESP_LOGI(TAG, "open_palm gesture -> mic record");
                        mic_capture_async(MIC_DURATION_MS);
                        s_last_trigger_ms = now_ms;
                    }
                }
            }

        } else if (fb) {
            camera_return(fb);
        }
        vTaskDelay(pdMS_TO_TICKS(20));  // yield to IDLE - prevents task WDT trigger
    }
}

void app_main(void)
{
    ESP_LOGW(TAG, "MIRA 2.0 booting -- CPRE 5450 FT Edition");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    oled_init();
    servo_init();
    servo_set_tracking(true);

    /* SD card - non-fatal if absent */
    sd_init();
    training_init();
    training_load_gesture_templates();

    /* PDM microphone - non-fatal if absent */
    mic_init();

    /* -- CPRE 5450: mmWave sensor + FT controller ---------------- */
    ESP_ERROR_CHECK(mmwave_init());  /* starts mmwave_rx_task on Core 0 */

    ft_config_t ft_cfg = {
        .w_agreement    = FT_W_AGREEMENT,
        .w_freshness    = FT_W_FRESHNESS,
        .w_transport    = FT_W_TRANSPORT,
        .min_confidence = FT_MIN_CONFIDENCE,
        .stale_ms       = FT_STALE_MS,
        .soft_retry_ms  = FT_SOFT_RETRY_MS,
        .k_hard         = FT_K_HARD,
        .k_window       = FT_K_WINDOW,
    };
    ft_init(&ft_cfg);
    /* ----------------------------------------------------------- */

    /* Configure Task Watchdog (5 s, panic on trigger) */
    esp_task_wdt_config_t twdt = {
        .timeout_ms     = 5000,
        .idle_core_mask = 0,
        .trigger_panic  = true,
    };
    esp_task_wdt_reconfigure(&twdt);

    // Allocate the PSRAM frame copy buffer
    s_frame_copy = heap_caps_malloc(FRAME_WIDTH * FRAME_HEIGHT * 2,
                                    MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_frame_copy)
        ESP_LOGE(TAG, "Failed to allocate frame copy buffer");

    vision_init();

    if (wifi_init_sta() != ESP_OK) {
        ESP_LOGE(TAG, "WiFi failed - HTTP server unavailable");
    } else {
        ESP_ERROR_CHECK(web_server_start());
        ESP_LOGW(TAG, "Open http://%s/ in a browser", wifi_get_ip());
    }

    vTaskDelay(pdMS_TO_TICKS(3000));
    ESP_ERROR_CHECK(camera_init());

    xTaskCreatePinnedToCore(vision_task, "vision",  32768, NULL,
                            configMAX_PRIORITIES - 1, NULL, 1 /* Core 1 */);
    xTaskCreatePinnedToCore(ft_task,     "ft_diag",  4096, NULL,
                            configMAX_PRIORITIES - 2, NULL, 0 /* Core 0 */);

    ESP_LOGI(TAG, "MIRA 2.0 ready -- https://github.com/Shi6aSK/MIRA-2.0");
}
