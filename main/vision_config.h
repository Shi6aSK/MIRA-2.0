#pragma once

/* ── WiFi ───────────────────────────────────────────────── */
#define WIFI_SSID        "Cribbo"
#define WIFI_PASS        "7fc5xzb38v4"
#define WIFI_MAX_RETRY   10

/* ── HTTP server ────────────────────────────────────────── */
#define HTTP_PORT        80

/* ── Camera pins (XIAO ESP32S3 Sense) ──────────────────── */
#define PWDN_GPIO_NUM   -1
#define RESET_GPIO_NUM  -1
#define XCLK_GPIO_NUM   10
#define SIOD_GPIO_NUM   40
#define SIOC_GPIO_NUM   39
#define Y9_GPIO_NUM     48
#define Y8_GPIO_NUM     11
#define Y7_GPIO_NUM     12
#define Y6_GPIO_NUM     14
#define Y5_GPIO_NUM     16
#define Y4_GPIO_NUM     18
#define Y3_GPIO_NUM     17
#define Y2_GPIO_NUM     15
#define VSYNC_GPIO_NUM  38
#define HREF_GPIO_NUM   47
#define PCLK_GPIO_NUM   13

/* ── Frame ──────────────────────────────────────────────── */
#define FRAME_WIDTH     320
#define FRAME_HEIGHT    240
#define JPEG_QUALITY    8    /* sensor hw JPEG register (0=best, 63=worst) – irrelevant in RGB565 mode */
#define STREAM_JPEG_QUALITY 80  /* fmt2jpg / jpge quality (1-100, higher=better) for MJPEG web stream */

/* ── Face detection thresholds ─────────────────────────── */
#define FACE_SCORE_MSR  0.45f   /* OV3660 cleaner image; higher threshold reduces false positives */
#define FACE_SCORE_MNP  0.55f

/* ── Hand/skin blob ─────────────────────────────────────── */
#define BLOCK_SIZE           8
#define MIN_BLOB_CELLS       6
#define BLOCK_MIN_SKIN_RATIO 0.33f
#define GESTURE_ASPECT_WIDE  1.30f
#define GESTURE_ASPECT_NARROW 0.75f

/* ── Servo ──────────────────────────────────────────────── */
#define SERVO_PAN_GPIO        3   /* XIAO D2 = GPIO3 */
#define SERVO_TILT_GPIO       4   /* XIAO D3 = GPIO4 */
#define SERVO_MIN_DEG         0
#define SERVO_MAX_DEG         180
#define SERVO_CENTER_DEG      90
#define SERVO_TRACK_STEP_DEG  2
#define SERVO_TRACK_DEADBAND_PX 5   /* tighter deadband so tilt/pan react sooner */
#define SERVO_TRACK_INTERVAL_MS 100
#define SERVO_PAN_INVERT      0   /* OV3660 outputs correctly-oriented frames; pan servo direction correct */
#define SERVO_TILT_INVERT     1   /* tilt servo is physically mounted inverted – mechanical, not camera-related */
#define SERVO_PULSE_MIN_US    500
#define SERVO_PULSE_MAX_US    2500

/* ── OLED SSD1306 128x32 ────────────────────────────────── */
#define OLED_SDA_GPIO   5   /* XIAO ESP32S3 D4 = GPIO5 = SDA */
#define OLED_SCL_GPIO   6   /* XIAO ESP32S3 D5 = GPIO6 = SCL */
#define OLED_I2C_PORT   0
#define OLED_I2C_ADDR   0x3C

/* ── SD Card SPI (XIAO ESP32S3 Sense expansion board) ──────── */
#define SD_CLK_GPIO   7
#define SD_MOSI_GPIO  9
#define SD_MISO_GPIO  8
#define SD_CS_GPIO    21

/* ── PDM Microphone (XIAO ESP32S3 Sense built-in) ───────────── */
#define MIC_CLK_GPIO  42   /* PDM clock – D8 on Sense expansion */
#define MIC_DATA_GPIO 41   /* PDM data  – D7 on Sense expansion */
#define MIC_DURATION_MS 5000   /* capture duration for open_palm gesture (5 s) */

/* ── Gesture trigger cooldown ───────────────────────────────── */
#define GESTURE_CONFIRM_FRAMES  3    /* consecutive frames before trigger */
#define GESTURE_COOLDOWN_MS  5000   /* ms between re-triggers */

/* ── 24 GHz mmWave Sensor (Seeed MR24HPC1) ──────────────────── */
#define MMWAVE_UART_NUM   1          /* UART peripheral number */
#define MMWAVE_TX_GPIO    43         /* XIAO D6 – UART1 TX */
#define MMWAVE_RX_GPIO    44         /* XIAO D7 – UART1 RX */
/* Note: D7/D8 on XIAO Sense expansion are used for MIC (PDM).
 * Reassign MIC to software I2S or use XIAO D9/D10 for mmWave if needed.
 * Default assignment: GPIO43/44 are unused on base XIAO ESP32S3 Sense. */

/* ── Fault-Tolerant Activation Controller ────────────────────── */
#define FT_W_AGREEMENT    0.50f   /* confidence weight: cross-sensor agreement */
#define FT_W_FRESHNESS    0.30f   /* confidence weight: timestamp freshness */
#define FT_W_TRANSPORT    0.20f   /* confidence weight: transport/watchdog health */
#define FT_MIN_CONFIDENCE 0.60f   /* below this fused score → suppress activation */
#define FT_STALE_MS       300     /* max sensor age (ms) before freshness = 0 */
#define FT_SOFT_RETRY_MS  3000    /* max time in SOFT_SAFE before escalation */
#define FT_K_HARD         4       /* consecutive mismatches → HARD_SAFE */
#define FT_K_WINDOW       8       /* sliding window size for mismatch tracking */
#define FT_TASK_INTERVAL_MS 100   /* diagnosis task period */
