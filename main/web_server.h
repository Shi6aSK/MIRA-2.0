#pragma once
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Start HTTP server on port 80.
esp_err_t web_server_start(void);

// Push an encoded JPEG into the shared frame buffer served at /frame.
// Called from vision_task. Copies data internally; caller may free jpg.
void web_server_update_frame(const uint8_t *jpg, size_t len);

// Auto-trigger Gemma from firmware (e.g. on gesture detection).
// ctx: "describe" or "point". No-op if a result is already pending.
void web_server_auto_trigger_gemma(const char *ctx);

#ifdef __cplusplus
}
#endif
