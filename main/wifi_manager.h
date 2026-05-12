#pragma once
#include "esp_err.h"

// Connect to WiFi in STA mode.  Blocks until connected or timeout.
// Returns ESP_OK on success, ESP_FAIL if unable to connect after retries.
esp_err_t wifi_init_sta(void);

// Returns the assigned IP as a string (valid after wifi_init_sta returns OK).
const char *wifi_get_ip(void);
