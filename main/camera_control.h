#pragma once
#include "esp_err.h"
#include "esp_camera.h"

#ifdef __cplusplus
extern "C" {
#endif

// Initialise OV3660 in RGB565 mode (320x240, PSRAM frame buffers).
esp_err_t camera_init(void);

// Get the latest frame buffer (call camera_return when done).
camera_fb_t *camera_capture(void);

// Return a frame buffer to the driver.
void camera_return(camera_fb_t *fb);

#ifdef __cplusplus
}
#endif
