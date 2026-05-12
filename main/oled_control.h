#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Initialise SSD1306 128x32 OLED on I2C_NUM_0 (GPIO SDA=4, SCL=5).
// Safe to call if OLED is absent – subsequent calls become no-ops.
void oled_init(void);

// Draw animated eyes.  pupil_dx/dy are pixel offsets from centre (-20..20, -6..6).
// face_seen=true draws alert open eyes; face_seen=false draws half-closed eyes.
void oled_draw_eyes(int pupil_dx, int pupil_dy, bool face_seen);

// Draw closed/sleep eyes (no face present).
void oled_draw_sleep(void);

#ifdef __cplusplus
}
#endif
