#pragma once

#include "esp_camera.h"
#include "vision_types.h"
#include "vision_config.h"

/* Skin-blob grid dimensions (for gesture training) */
#define VISION_GW  (FRAME_WIDTH  / BLOCK_SIZE)
#define VISION_GH  (FRAME_HEIGHT / BLOCK_SIZE)

#ifdef __cplusplus
extern "C" {
#endif

// Load the ESP-DL face detection model.  Call once before vision_process_frame.
void vision_init(void);

// Process one RGB565 frame: runs face detection first, then gesture detection
// only when a face is present in the frame.
// Updates internal detection state readable via vision_get_detection().
void vision_process_frame(camera_fb_t *fb);

// Thread-safe read of the latest detection result.
detection_t vision_get_detection(void);

// Copy the last computed skin-blob grid mask.
// out must be VISION_GW * VISION_GH bytes (30*30 = 900 for 240x240 / block 8).
void vision_get_blob_mask(uint8_t *out);

#ifdef __cplusplus
}
#endif
