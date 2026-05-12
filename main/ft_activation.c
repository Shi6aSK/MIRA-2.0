/*
 * ft_activation.c
 * ---------------
 * Fault-Tolerant Activation Controller — CPRE 5450
 *
 * Implements the seven-state FSM described in ft_activation.h.
 *
 * Decision cycle (called from ft_update):
 *   1. Compute per-sensor health scores (confidence, freshness, transport).
 *   2. Compute fused confidence C = 0.5*(C_radar + C_camera).
 *   3. Evaluate cross-sensor agreement.
 *   4. Update sliding window of mismatch counts.
 *   5. Apply state-transition rules.
 *
 * State transitions:
 *   IDLE             + radar_present              → RADAR_CANDIDATE
 *   RADAR_CANDIDATE  + camera confirms            → VISUAL_CONFIRM
 *   VISUAL_CONFIRM   + agreement + C >= min_conf  → ACTIVE
 *   VISUAL_CONFIRM   + mismatch                   → SOFT_SAFE
 *   ACTIVE           + health fault               → SOFT_SAFE
 *   SOFT_SAFE        + resolved                   → VISUAL_CONFIRM
 *   SOFT_SAFE        + mismatch_count >= k_hard   → HARD_SAFE
 *   SOFT_SAFE        + both sensors faulted       → DEGRADED
 *   HARD_SAFE        + manual reset               → IDLE
 *   ACTIVE           + session_done               → IDLE
 *   any              + radar stale + cam stale    → DEGRADED
 */

#include "ft_activation.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <string.h>
#include <math.h>

static const char *TAG = "ft";

/* ── Defaults ───────────────────────────────────────────────────── */
static const ft_config_t k_defaults = {
    .w_agreement    = 0.50f,
    .w_freshness    = 0.30f,
    .w_transport    = 0.20f,
    .min_confidence = 0.60f,
    .stale_ms       = 300,
    .soft_retry_ms  = 3000,
    .k_hard         = 4,
    .k_window       = 8,
};

/* ── Module state ───────────────────────────────────────────────── */
static ft_config_t       s_cfg;
static ft_status_t       s_status;
static SemaphoreHandle_t s_mutex = NULL;

/* Sliding window ring-buffer for mismatch tracking */
#define WIN_MAX 16
static uint8_t  s_win[WIN_MAX];  /* 1 = mismatch, 0 = agreement */
static int      s_win_head = 0;

/* ── Helpers ────────────────────────────────────────────────────── */

static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

/* Compute freshness score: 1.0 when perfectly fresh, 0.0 at stale_ms */
static float freshness_score(uint32_t age_ms, uint32_t stale_ms)
{
    if (age_ms >= stale_ms) return 0.0f;
    return 1.0f - (float)age_ms / (float)stale_ms;
}

/* Compute per-sensor confidence score: C_i = w_s*S + w_f*F + w_t*T */
static float sensor_confidence(float agreement, float fresh, float transport,
                                const ft_config_t *c)
{
    return clampf(c->w_agreement * agreement +
                  c->w_freshness * fresh    +
                  c->w_transport * transport, 0.0f, 1.0f);
}

/* Push one mismatch sample into the sliding window, return window sum */
static uint32_t window_push(uint8_t mismatch)
{
    s_win[s_win_head % s_cfg.k_window] = mismatch;
    s_win_head++;
    uint32_t sum = 0;
    for (uint32_t i = 0; i < s_cfg.k_window; i++)
        sum += s_win[i];
    return sum;
}

static void set_state(ft_state_t new_state, const char *reason)
{
    if (s_status.state != new_state) {
        ESP_LOGI(TAG, "[FT] %s → %s  (%s)",
                 ft_state_name(s_status.state), ft_state_name(new_state), reason);
        s_status.state         = new_state;
        s_status.state_since_ms = now_ms();
        snprintf(s_status.fault_reason, sizeof(s_status.fault_reason), "%s", reason);
    }
    s_status.activation_allowed = (new_state == FT_ACTIVE);
}

/* ── Public API ─────────────────────────────────────────────────── */

void ft_init(const ft_config_t *cfg)
{
    s_cfg   = cfg ? *cfg : k_defaults;
    s_mutex = xSemaphoreCreateMutex();
    memset(&s_status, 0, sizeof(s_status));
    memset(s_win,     0, sizeof(s_win));
    s_status.state = FT_IDLE;
    s_status.state_since_ms = now_ms();
    ESP_LOGI(TAG, "FT controller ready (min_conf=%.2f k_hard=%lu k_win=%lu)",
             (double)s_cfg.min_confidence,
             (unsigned long)s_cfg.k_hard,
             (unsigned long)s_cfg.k_window);
}

void ft_update(bool radar_present, bool radar_stale,
               bool camera_present, bool camera_stale,
               bool watchdog_ok)
{
    /* ── Compute health scores ──────────────────────────────────── */
    float r_fresh = freshness_score(radar_stale  ? s_cfg.stale_ms : 0, s_cfg.stale_ms);
    float c_fresh = freshness_score(camera_stale ? s_cfg.stale_ms : 0, s_cfg.stale_ms);
    float r_trans = (watchdog_ok && !radar_stale)  ? 1.0f : 0.0f;
    float c_trans = (watchdog_ok && !camera_stale) ? 1.0f : 0.0f;

    /* Agreement: 1.0 if both agree on presence/absence, else 0.0 */
    bool agree    = (radar_present == camera_present);
    float agree_f = agree ? 1.0f : 0.0f;

    ft_sensor_health_t rh, ch;
    rh.agreement  = ch.agreement  = agree_f;
    rh.freshness  = r_fresh;
    ch.freshness  = c_fresh;
    rh.transport  = r_trans;
    ch.transport  = c_trans;
    rh.stale      = radar_stale;
    ch.stale      = camera_stale;
    rh.faulted    = radar_stale  || !watchdog_ok;
    ch.faulted    = camera_stale || !watchdog_ok;
    rh.confidence = sensor_confidence(agree_f, r_fresh, r_trans, &s_cfg);
    ch.confidence = sensor_confidence(agree_f, c_fresh, c_trans, &s_cfg);
    float fused   = 0.5f * (rh.confidence + ch.confidence);

    /* Update sliding window */
    uint32_t win_sum = window_push(agree ? 0 : 1);

    xSemaphoreTake(s_mutex, portMAX_DELAY);

    s_status.radar_health     = rh;
    s_status.camera_health    = ch;
    s_status.fused_confidence = fused;
    s_status.mismatch_count   = win_sum;

    ft_state_t cur = s_status.state;
    uint32_t   t   = now_ms();

    /* ── Degraded check (overrides everything) ──────────────────── */
    if (radar_stale && camera_stale) {
        set_state(FT_DEGRADED, "both sensors stale/faulted");
        goto done;
    }

    /* ── State machine ──────────────────────────────────────────── */
    switch (cur) {

    case FT_IDLE:
        if (radar_present && !radar_stale)
            set_state(FT_RADAR_CANDIDATE, "radar trigger");
        break;

    case FT_RADAR_CANDIDATE:
        if (radar_stale) {
            set_state(FT_IDLE, "radar stale after trigger");
            break;
        }
        /* Always request camera confirmation */
        set_state(FT_VISUAL_CONFIRM, "requesting camera confirm");
        break;

    case FT_VISUAL_CONFIRM:
        if (camera_stale) {
            set_state(FT_SOFT_SAFE, "camera stale during confirm");
            break;
        }
        if (agree && fused >= s_cfg.min_confidence)
            set_state(FT_ACTIVE, "radar+camera agree, confidence OK");
        else if (!radar_present && !camera_present)
            set_state(FT_IDLE, "both sensors say absent");
        else
            set_state(FT_SOFT_SAFE, "mismatch or low confidence");
        break;

    case FT_ACTIVE:
        /* Stay active unless a fault develops */
        if (!radar_present && !camera_present)
            set_state(FT_IDLE, "session ended – no presence");
        else if (!agree || fused < s_cfg.min_confidence)
            set_state(FT_SOFT_SAFE, "health degraded during active");
        break;

    case FT_SOFT_SAFE:
        s_status.retry_count++;
        if (win_sum >= s_cfg.k_hard) {
            set_state(FT_HARD_SAFE, "persistent mismatch");
            break;
        }
        if ((t - s_status.state_since_ms) > s_cfg.soft_retry_ms) {
            set_state(FT_HARD_SAFE, "soft-safe timeout");
            break;
        }
        /* Check if both sensors recovered */
        if (agree && fused >= s_cfg.min_confidence)
            set_state(FT_VISUAL_CONFIRM, "sensors recovered – revalidate");
        break;

    case FT_HARD_SAFE:
        /* Only manual reset exits hard safe */
        break;

    case FT_DEGRADED:
        /* Exit when at least one sensor comes back */
        if (!radar_stale || !camera_stale)
            set_state(FT_IDLE, "sensor recovered from degraded");
        break;
    }

done:
    xSemaphoreGive(s_mutex);
}

ft_status_t ft_get_status(void)
{
    ft_status_t s;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s = s_status;
    xSemaphoreGive(s_mutex);
    return s;
}

bool ft_activation_allowed(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    bool ok = s_status.activation_allowed;
    xSemaphoreGive(s_mutex);
    return ok;
}

void ft_reset(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    set_state(FT_IDLE, "manual reset");
    s_status.mismatch_count = 0;
    s_status.retry_count    = 0;
    memset(s_win, 0, sizeof(s_win));
    s_win_head = 0;
    xSemaphoreGive(s_mutex);
    ESP_LOGI(TAG, "FT controller reset to IDLE");
}

const char *ft_state_name(ft_state_t s)
{
    switch (s) {
    case FT_IDLE:            return "IDLE";
    case FT_RADAR_CANDIDATE: return "RADAR_CANDIDATE";
    case FT_VISUAL_CONFIRM:  return "VISUAL_CONFIRM";
    case FT_ACTIVE:          return "ACTIVE";
    case FT_SOFT_SAFE:       return "SOFT_SAFE";
    case FT_HARD_SAFE:       return "HARD_SAFE";
    case FT_DEGRADED:        return "DEGRADED";
    default:                 return "UNKNOWN";
    }
}
