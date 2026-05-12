// oled_control.c
//
// SSD1306 128x32 OLED driver.
// I2C_NUM_0, SDA=GPIO4, SCL=GPIO5, address 0x3C.
//
// Eye animation matches Theo1.0.ino:
//   EYE_RADIUS 15, EYE_Y 16, LEFT_EYE_X 34, RIGHT_EYE_X 94
//   PUPIL_RADIUS 7 (open) / 0 (sleep = horizontal lines)
//
// White background, black eye circles, black filled pupils.

#include "oled_control.h"
#include "vision_config.h"

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>
#include <math.h>

static const char *TAG = "oled";

// ---------------------------------------------------------------
// Display geometry
// ---------------------------------------------------------------
#define OLED_W          128
#define OLED_H           32
#define OLED_PAGES        4      // 32 rows / 8 bits per page
#define FB_SIZE         (OLED_W * OLED_PAGES)  // 512 bytes

#define EYE_RADIUS       15
#define EYE_Y            16
#define LEFT_EYE_X       34
#define RIGHT_EYE_X      94
#define PUPIL_RADIUS      7

// ---------------------------------------------------------------
// State
// ---------------------------------------------------------------
static bool                     s_ready   = false;
static i2c_master_dev_handle_t  s_dev     = NULL;

static uint8_t  s_fb[FB_SIZE];         // framebuffer (page-oriented)
static uint8_t  s_tx_buf[1 + FB_SIZE]; // 0x40 + framebuffer for data TX

#define OLED_FLUSH_MIN_US  50000LL   // max 20 fps on OLED
static int64_t s_last_flush_us = 0;

// ---------------------------------------------------------------
// Pixel helpers
// ---------------------------------------------------------------
static void set_pixel(int x, int y, int on)
{
    if (x < 0 || x >= OLED_W || y < 0 || y >= OLED_H) return;
    int page = y >> 3;
    int bit  = y & 7;
    int idx  = page * OLED_W + x;
    if (on) s_fb[idx] |=  (uint8_t)(1u << bit);
    else    s_fb[idx] &= ~(uint8_t)(1u << bit);
}

static void fill_fb(int on)
{
    memset(s_fb, on ? 0xFF : 0x00, FB_SIZE);
}

// Bresenham circle outline
static void draw_circle(int cx, int cy, int r, int on)
{
    int x = r, y = 0, err = 0;
    while (x >= y) {
        set_pixel(cx + x, cy + y, on); set_pixel(cx - x, cy + y, on);
        set_pixel(cx + x, cy - y, on); set_pixel(cx - x, cy - y, on);
        set_pixel(cx + y, cy + x, on); set_pixel(cx - y, cy + x, on);
        set_pixel(cx + y, cy - x, on); set_pixel(cx - y, cy - x, on);
        ++y;
        err += 2 * y + 1;
        if (err > 0) { --x; err -= 2 * x + 1; }
    }
}

// Filled circle
static void fill_circle(int cx, int cy, int r, int on)
{
    for (int dy = -r; dy <= r; ++dy) {
        int dx = (int)sqrtf((float)(r * r - dy * dy));
        for (int x = cx - dx; x <= cx + dx; ++x)
            set_pixel(x, cy + dy, on);
    }
}

// Horizontal line
static void draw_hline(int x0, int x1, int y, int on)
{
    if (x0 > x1) { int t = x0; x0 = x1; x1 = t; }
    for (int x = x0; x <= x1; ++x) set_pixel(x, y, on);
}

// ---------------------------------------------------------------
// I2C helpers
// ---------------------------------------------------------------
static esp_err_t send_cmd(uint8_t cmd)
{
    uint8_t buf[2] = { 0x00, cmd };
    return i2c_master_transmit(s_dev, buf, 2, 20);
}

static void oled_flush(void)
{
    int64_t now = esp_timer_get_time();
    if (now - s_last_flush_us < OLED_FLUSH_MIN_US) return;
    s_last_flush_us = now;

    // Set column + page address window (horizontal mode)
    uint8_t addr_cmds[] = { 0x00, 0x21, 0x00, 0x7F, 0x22, 0x00, 0x03 };
    i2c_master_transmit(s_dev, addr_cmds, sizeof(addr_cmds), 20);

    // Send all 512 framebuffer bytes in one transaction
    s_tx_buf[0] = 0x40;
    memcpy(&s_tx_buf[1], s_fb, FB_SIZE);
    i2c_master_transmit(s_dev, s_tx_buf, 1 + FB_SIZE, 100);
}

// ---------------------------------------------------------------
// Init sequence (SSD1306, 128x32)
// ---------------------------------------------------------------
static const uint8_t OLED_INIT_CMDS[] = {
    0xAE,        // display off
    0xD5, 0x80,  // set display clock / oscillator
    0xA8, 0x1F,  // set multiplex ratio (31 = 32 rows - 1)
    0xD3, 0x00,  // set display offset = 0
    0x40,        // set start line = 0
    0x8D, 0x14,  // charge pump ON
    0x20, 0x00,  // horizontal addressing mode
    0xA1,        // segment remap (mirror horizontally)
    0xC8,        // COM output reversed (mirror vertically)
    0xDA, 0x02,  // COM pins: sequential, no remap (32-row variant)
    0x81, 0xCF,  // contrast = 0xCF
    0xD9, 0xF1,  // pre-charge period
    0xDB, 0x40,  // VCOMH deselect = 0.77*Vcc
    0xA4,        // output follows RAM
    0xA6,        // normal display (1=on)
    0xAF,        // display ON
};

// ---------------------------------------------------------------
// Public API
// ---------------------------------------------------------------
void oled_init(void)
{
    i2c_master_bus_config_t bus_cfg;
    memset(&bus_cfg, 0, sizeof(bus_cfg));
    bus_cfg.i2c_port         = (i2c_port_num_t)OLED_I2C_PORT;
    bus_cfg.sda_io_num       = (gpio_num_t)OLED_SDA_GPIO;
    bus_cfg.scl_io_num       = (gpio_num_t)OLED_SCL_GPIO;
    bus_cfg.clk_source       = I2C_CLK_SRC_DEFAULT;
    bus_cfg.glitch_ignore_cnt = 7;
    bus_cfg.flags.enable_internal_pullup = 1;

    i2c_master_bus_handle_t bus;
    esp_err_t err = i2c_new_master_bus(&bus_cfg, &bus);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "I2C bus init failed: %s", esp_err_to_name(err));
        return;
    }

    /* Probe 0x3C first, then 0x3D – some SSD1306 boards use either address */
    static const uint8_t OLED_ADDRS[] = {0x3C, 0x3D};
    uint8_t oled_addr = 0;
    for (int ai = 0; ai < 2; ai++) {
        if (i2c_master_probe(bus, OLED_ADDRS[ai], 20) == ESP_OK) {
            oled_addr = OLED_ADDRS[ai];
            ESP_LOGI(TAG, "OLED found at 0x%02X", oled_addr);
            break;
        }
    }
    if (!oled_addr) {
        ESP_LOGW(TAG, "No OLED found at 0x3C or 0x3D, OLED disabled");
        return;
    }

    i2c_device_config_t dev_cfg;
    memset(&dev_cfg, 0, sizeof(dev_cfg));
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address  = oled_addr;
    dev_cfg.scl_speed_hz    = 400000;

    err = i2c_master_bus_add_device(bus, &dev_cfg, &s_dev);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "OLED device add failed: %s", esp_err_to_name(err));
        return;
    }

    // Send init commands one by one
    for (size_t i = 0; i < sizeof(OLED_INIT_CMDS); ++i) {
        err = send_cmd(OLED_INIT_CMDS[i]);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "OLED init cmd %d failed, OLED disabled", (int)i);
            s_dev = NULL;
            return;
        }
    }

    // Clear to white (all bits = 1 → pixels lit)
    fill_fb(1);
    s_tx_buf[0] = 0x40;
    memcpy(&s_tx_buf[1], s_fb, FB_SIZE);
    uint8_t addr[] = { 0x00, 0x21, 0x00, 0x7F, 0x22, 0x00, 0x03 };
    i2c_master_transmit(s_dev, addr, sizeof(addr), 20);
    i2c_master_transmit(s_dev, s_tx_buf, 1 + FB_SIZE, 100);

    s_ready = true;
    ESP_LOGW(TAG, "OLED ready");
}

void oled_draw_eyes(int pupil_dx, int pupil_dy, bool face_seen)
{
    if (!s_ready) return;

    fill_fb(1);  // white background

    // Eye outlines (black)
    draw_circle(LEFT_EYE_X,  EYE_Y, EYE_RADIUS, 0);
    draw_circle(RIGHT_EYE_X, EYE_Y, EYE_RADIUS, 0);

    if (face_seen) {
        // Filled pupils (black) tracking the face
        int pdx = pupil_dx;
        int pdy = pupil_dy;
        // Clamp pupils inside eye circle
        float dist = sqrtf((float)(pdx * pdx + pdy * pdy));
        int max_move = EYE_RADIUS - PUPIL_RADIUS - 2;
        if (max_move < 0) max_move = 0;
        if (dist > (float)max_move && dist > 0.0f) {
            pdx = (int)((float)pdx / dist * (float)max_move);
            pdy = (int)((float)pdy / dist * (float)max_move);
        }
        fill_circle(LEFT_EYE_X  + pdx, EYE_Y + pdy, PUPIL_RADIUS, 0);
        fill_circle(RIGHT_EYE_X + pdx, EYE_Y + pdy, PUPIL_RADIUS, 0);
    } else {
        // Centre pupils (idle / relaxed)
        fill_circle(LEFT_EYE_X,  EYE_Y, PUPIL_RADIUS, 0);
        fill_circle(RIGHT_EYE_X, EYE_Y, PUPIL_RADIUS, 0);
    }

    oled_flush();
}

void oled_draw_sleep(void)
{
    if (!s_ready) return;

    fill_fb(1);  // white background

    // Half-closed eyes: just horizontal lines where the eye centre is
    draw_hline(LEFT_EYE_X  - EYE_RADIUS, LEFT_EYE_X  + EYE_RADIUS, EYE_Y, 0);
    draw_hline(RIGHT_EYE_X - EYE_RADIUS, RIGHT_EYE_X + EYE_RADIUS, EYE_Y, 0);
    draw_hline(LEFT_EYE_X  - EYE_RADIUS, LEFT_EYE_X  + EYE_RADIUS, EYE_Y + 1, 0);
    draw_hline(RIGHT_EYE_X - EYE_RADIUS, RIGHT_EYE_X + EYE_RADIUS, EYE_Y + 1, 0);

    oled_flush();
}
