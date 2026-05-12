#pragma once
/*
 * ft_activation.h
 * ---------------
 * Fault-Tolerant Activation Controller for the CPRE 5450 project.
 *
 * Implements a seven-state activation FSM with cross-sensor consistency
 * checking, temporal redundancy (k-sample window), per-sensor confidence
 * scoring, and a two-tier safe-state policy (Soft Safe / Hard Safe).
 *
 * Designed to run on ESP32-S3 FreeRTOS.  All state is protected by a
 * FreeRTOS mutex so it can be queried from the web-server task.
 *
 * States:
 *   FT_IDLE            – waiting for radar trigger
 *   FT_RADAR_CANDIDATE – radar indicates possible presence
 *   FT_VISUAL_CONFIRM  – camera confirmation requested
 *   FT_ACTIVE          – all gates passed; full activation allowed
 *   FT_SOFT_SAFE       – transient inconsistency; activation suppressed
 *   FT_HARD_SAFE       – persistent fault; activation blocked
 *   FT_DEGRADED        – both sensors degraded; minimal-power mode
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Activation states ──────────────────────────────────────────── */
typedef enum {
    FT_IDLE            = 0,
    FT_RADAR_CANDIDATE = 1,
    FT_VISUAL_CONFIRM  = 2,
    FT_ACTIVE          = 3,
    FT_SOFT_SAFE       = 4,
    FT_HARD_SAFE       = 5,
    FT_DEGRADED        = 6,
} ft_state_t;

/* ── Per-sensor health snapshot (for logging / web UI) ─────────── */
typedef struct {
    float    confidence;      /* C_i = w_s*S + w_f*F + w_t*T   [0,1] */
    float    agreement;       /* S_i: 1.0 agree, 0.0 disagree  */
    float    freshness;       /* F_i: linear staleness decay    */
    float    transport;       /* T_i: 1.0 healthy, 0.0 faulted */
    bool     stale;
    bool     faulted;
} ft_sensor_health_t;

/* ── Full controller snapshot ───────────────────────────────────── */
typedef struct {
    ft_state_t         state;
    ft_sensor_health_t radar_health;
    ft_sensor_health_t camera_health;
    float              fused_confidence;   /* weighted average of both */
    uint32_t           mismatch_count;     /* consecutive mismatches in window */
    uint32_t           retry_count;        /* soft-safe retries */
    uint32_t           state_since_ms;     /* timestamp of last state change */
    bool               activation_allowed; /* true only in FT_ACTIVE */
    char               fault_reason[64];   /* human-readable last fault */
} ft_status_t;

/* ── Tunable parameters (set before ft_init) ────────────────────── */
typedef struct {
    /* Confidence weights (must sum to 1.0) */
    float  w_agreement;     /* default 0.50 */
    float  w_freshness;     /* default 0.30 */
    float  w_transport;     /* default 0.20 */
    /* Thresholds */
    float  min_confidence;  /* below this → suppress activation; default 0.60 */
    uint32_t stale_ms;      /* frame freshness deadline ms; default 300 */
    uint32_t soft_retry_ms; /* max time in soft safe before escalation ms; default 3000 */
    uint32_t k_hard;        /* consecutive mismatches → hard safe; default 4 */
    uint32_t k_window;      /* sliding window size; default 8 */
} ft_config_t;

/* ── Public API ─────────────────────────────────────────────────── */

/**
 * @brief  Initialise the fault-tolerance controller.
 *         Pass NULL to use defaults.
 */
void ft_init(const ft_config_t *cfg);

/**
 * @brief  Feed new sensor observations.  Call once per decision cycle
 *         (e.g. after each camera frame + fresh radar reading).
 *
 * @param  radar_present  true if mmwave reports presence (and reading is fresh)
 * @param  radar_stale    true if radar UART has timed out
 * @param  camera_present true if face/motion detector confirmed a person
 * @param  camera_stale   true if camera frame timestamp is too old
 * @param  watchdog_ok    true if TWDT has not triggered
 */
void ft_update(bool radar_present, bool radar_stale,
               bool camera_present, bool camera_stale,
               bool watchdog_ok);

/**
 * @brief  Thread-safe snapshot of current controller status.
 */
ft_status_t ft_get_status(void);

/**
 * @brief  Returns true iff activation is currently permitted.
 */
bool ft_activation_allowed(void);

/**
 * @brief  Force a reset to IDLE (e.g. after manual sensor reset).
 */
void ft_reset(void);

/**
 * @brief  Returns a human-readable state name string.
 */
const char *ft_state_name(ft_state_t s);

#ifdef __cplusplus
}
#endif
