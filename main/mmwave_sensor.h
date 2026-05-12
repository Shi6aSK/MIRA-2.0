#pragma once
/*
 * mmwave_sensor.h
 * ---------------
 * Driver for Seeed 24 GHz mmWave Human Static Presence Module (MR24HPC1 / XIAO variant).
 * UART-based FMCW sensor.  Communicates at 115200 8N1.
 *
 * Frame format (binary protocol):
 *   SOF  [0x53] | Length [2B LE] | Type [1B] | Head [1B] | Data [...] | CRC8 | EOF [0x54]
 *
 * Presence types reported in Type byte:
 *   0x01  No one present
 *   0x02  Motion state (someone moving)
 *   0x03  Micro-motion / stationary presence
 */

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ── UART wiring (change in vision_config.h if needed) ─────────── */
#ifndef MMWAVE_UART_NUM
#define MMWAVE_UART_NUM   1
#endif
#ifndef MMWAVE_TX_GPIO
#define MMWAVE_TX_GPIO    43   /* XIAO D6 – TX1 */
#endif
#ifndef MMWAVE_RX_GPIO
#define MMWAVE_RX_GPIO    44   /* XIAO D7 – RX1 */
#endif
#define MMWAVE_BAUD       115200

/* ── Radar presence states ──────────────────────────────────────── */
typedef enum {
    MMWAVE_STATE_UNKNOWN  = 0,
    MMWAVE_STATE_NO_ONE   = 1,   /* 0x01 – no presence */
    MMWAVE_STATE_MOTION   = 2,   /* 0x02 – moving occupant */
    MMWAVE_STATE_STATIC   = 3,   /* 0x03 – stationary occupant */
} mmwave_state_t;

/* ── Per-reading snapshot ───────────────────────────────────────── */
typedef struct {
    mmwave_state_t state;        /* presence state */
    uint32_t       timestamp_ms; /* esp_timer tick ms at receipt */
    uint8_t        raw_energy;   /* range-gate energy byte (0-255) */
    bool           frame_valid;  /* CRC and framing OK */
    uint32_t       drop_count;   /* consecutive lost frames */
} mmwave_reading_t;

/* ── Public API ─────────────────────────────────────────────────── */

/**
 * @brief  Initialise UART and start background RX task.
 *         Must be called once from app_main before any reads.
 */
esp_err_t mmwave_init(void);

/**
 * @brief  Thread-safe copy of the latest radar reading.
 *         Returns the last valid reading (or UNKNOWN if none yet received).
 */
mmwave_reading_t mmwave_get_reading(void);

/**
 * @brief  Returns true if any presence (motion OR static) is detected
 *         and the reading is fresh (< MMWAVE_STALE_MS old).
 */
bool mmwave_presence_detected(void);

/**
 * @brief  Returns true if the last frame timestamp is older than
 *         MMWAVE_STALE_MS, indicating a possible transport fault.
 */
bool mmwave_is_stale(void);

/**
 * @brief  Returns the consecutive drop counter (resets on valid frame).
 */
uint32_t mmwave_drop_count(void);

#ifdef __cplusplus
}
#endif
