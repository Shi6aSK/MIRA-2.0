#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_camera.h"
#include "vision_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    TRAIN_IDLE    = 0,
    TRAIN_FACE    = 1,
    TRAIN_GESTURE = 2,
} train_mode_t;

// Initialize training subsystem (call once from app_main).
void training_init(void);

// Start collecting labeled face samples.
void training_start_face(const char *label);

// Start collecting labeled gesture templates.
void training_start_gesture(const char *label);

// Stop training and finalize.
void training_stop(void);

// Queries
train_mode_t training_get_mode(void);
const char  *training_get_label(void);
int          training_get_count(void);

// Call from vision_task after vision_process_frame().
// Saves a face JPEG to SD if in face-training mode and a face is detected.
bool training_maybe_capture_face(const camera_fb_t *fb, const detection_t *det);

// Call from vision_task after vision_process_frame().
// Saves blob-mask binary to SD if in gesture-training mode and a gesture is detected.
// Internally calls vision_get_blob_mask().
bool training_maybe_capture_gesture(const detection_t *det);

// Load all gesture template masks from SD into RAM and build the KNN index.
// Replaces the old counting-only scan. Call once after sd_init().
void training_load_gesture_templates(void);

// 1-NN classify a blob mask against the loaded templates.
// mask must be VISION_GW * VISION_GH bytes (same layout as training files).
// Returns the gesture label (e.g. "point", "open_palm") or NULL when no
// templates have been loaded (caller should fall back to aspect-ratio rule).
const char *gesture_knn_classify(const uint8_t *mask);

// Number of templates currently loaded into the KNN index.
int gesture_knn_count(void);

#ifdef __cplusplus
}
#endif
