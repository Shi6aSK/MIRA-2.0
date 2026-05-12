#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Mount the SD card over SPI.  Returns true on success.
// Safe to call if SD is not present – returns false gracefully.
bool sd_init(void);

// Create directory (and parents) under /sdcard.
// path must start with '/', e.g. "/faces/theo"
bool sd_mkdir(const char *path);

// Write raw bytes to /sdcard<path>.  Parent directory must exist.
bool sd_save_bytes(const char *path, const uint8_t *data, size_t len);

// True if SD was successfully mounted.
bool sd_is_mounted(void);

#ifdef __cplusplus
}
#endif
