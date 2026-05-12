# MIRA 2.0 - Fault-Tolerant Multimodal Presence Verification on ESP32-S3

**Course:** CPRE 5450 - Dependable Computing Systems  
**Institution:** Iowa State University, Spring 2026  
**Author:** Shobhit Singh

> MIRA 2.0 extends a multimodal assistive robot (MIRA, from CPRE 5750) with a
> fault-tolerant activation subsystem that decides - dependably - when the system
> is allowed to activate camera inference, audio capture, or servo actuation.

---

## Table of Contents

1. [Motivation](#1-motivation)
2. [Problem Statement and Failure Modes](#2-problem-statement-and-failure-modes)
3. [Hardware Platform](#3-hardware-platform)
4. [Known Approaches and Fault-Tolerance Analysis](#4-known-approaches-and-fault-tolerance-analysis)
5. [Proposed Fault-Tolerant Architecture](#5-proposed-fault-tolerant-architecture)
6. [Fault Model and Diagnosis Logic](#6-fault-model-and-diagnosis-logic)
7. [Safe-State and Recovery Policy](#7-safe-state-and-recovery-policy)
8. [Evaluation Plan and Metrics](#8-evaluation-plan-and-metrics)
9. [Comparative Analysis](#9-comparative-analysis)
10. [Expected Contributions and Limitations](#10-expected-contributions-and-limitations)
11. [Repository Structure](#11-repository-structure)
12. [Build and Flash](#12-build-and-flash)
13. [CPRE 5450 Implementation Details](#13-cpre-5450-implementation-details)
14. [References](#14-references)

---

## 1. Motivation

Room-scale assistive systems must decide when to wake up, when to sense, and when to
interact. This is especially critical in privacy-preserving systems for blind or visually
impaired (BVI) users: unnecessary camera or audio activation can expose private data,
while missed activation can deny assistance at a moment of need.

The broader context for this work is a multimodal assistive perception platform running
on a Seeed XIAO ESP32-S3 Sense. That system integrates a camera, microphone, 24 GHz
mmWave presence sensor, pan-tilt motors, and an Android companion for gesture/audio
interaction and user tracking. Its design philosophy is privacy-centered: local edge
perception, gesture-controlled recording, minimal data transmission, and camera/audio
activation only when the user intentionally engages.

**This CPRE 5450 project isolates one dependability-critical part:** the activation and
presence-verification subsystem. The core question:

> *When is the system allowed to activate high-power or privacy-sensitive functions
> such as camera inference, audio capture, or servo actuation?*

This is not only a sensor-fusion problem - it is a **fault-tolerant decision problem**.
The subsystem must remain reliable under sensor faults, timing faults, and environmental
uncertainty.

---

## 2. Problem Statement and Failure Modes

### The Brittleness of Single-Modality Activation

Each sensor has fundamentally different failure conditions:

- A **camera** may fail under low light, motion blur, partial occlusion, or frame-buffer
  problems. The ESP32 camera driver warns that non-JPEG formats can stress PSRAM and
  image data may be lost under heavy memory/bus load.
- A **24 GHz mmWave radar** provides lighting-independent presence detection but can be
  affected by clutter, reflections from walls or furniture, moving curtains, fans, or
  incorrect range-gate tuning.

### System-Level Failure Modes

Using the classical fault-error-failure chain (Avizienis et al., 2004):

| Fault Source | Error State | System Failure | Consequence |
|---|---|---|---|
| Low light / occlusion | Camera false negative | **Missed activation** | No assistance to user |
| Fan / curtain / reflection | Radar false positive | **False activation** | Battery drain + privacy risk |
| UART packet drop | Stale radar state | Wrong decision | Unsafe or unavailable behavior |
| ESP32 task delay | Old camera frame used | Silent wrong decision | Hard-to-detect reliability failure |

**False activation** - the system activates when no valid user is present - wastes
battery and may compromise privacy. **Missed activation** - failing to activate when the
user needs help - is a denial-of-service failure for an assistive system.

Both failure modes are equally unacceptable. The design challenge is to minimize both
simultaneously, under real embedded system constraints, without a GPU or
neural-network accelerator.

---

## 3. Hardware Platform

| Component | Part | Specification |
|---|---|---|
| MCU | Seeed XIAO ESP32-S3 Sense | Dual-core Xtensa LX7 @ 240 MHz, 512 KB SRAM, 8 MB PSRAM, 8 MB flash |
| Camera | OmniVision OV2640 | DVP 8-bit, 240x240 RGB565, ~10 fps effective with inference |
| mmWave sensor | Seeed MR24HPC1 (24 GHz FMCW) | UART 115200 8N1, 0.75-6 m range, +/-60 deg, moving + stationary detection |
| Display | SSD1306 OLED (I2C) | "Eyes" bitmap, status feedback |
| Audio | I2S MEMS microphone | Async capture, I2S DMA |
| Pan-tilt | 2x servo (PWM/LEDC) | User-tracking actuation |

### mmWave Sensor Wire-Up

```
ESP32-S3 GPIO43 (UART1 TX)  ->  MR24HPC1 UART RX
ESP32-S3 GPIO44 (UART1 RX)  <-  MR24HPC1 UART TX
3.3 V                        ->  MR24HPC1 VCC
GND                          ->  MR24HPC1 GND
```

Radar firmware: V 2.44.25070917, detection range 0-3 m.

### Design Constraint

The ESP32-S3 has no hardware GPU or neural-network accelerator. All inference runs on
the LX7 cores using INT8-quantized models via Espressif's ESP-DL library. Per-frame
inference takes 100-200 ms. This motivates a **lightweight, rule-based activation
controller** rather than a heavy deep-learning pipeline.

---

## 4. Known Approaches and Fault-Tolerance Analysis

### Approach 1: mmWave-Only Presence Sensing

**Strengths:** Works without visible light; detects motion and micro-motion (breathing,
minor body shifts); does not capture identifiable images (privacy-preserving); low power.

**Weaknesses:** Vulnerable to clutter from fans, HVAC vents, moving curtains, and
reflections from metallic surfaces or glass. Static vs. moving ambiguity can cause
both false presence and missed presence.

**FT Analysis:** Radar-only activation has weak diagnosability. If radar reports
presence, there is no independent modality to determine whether the cause is a true
person, clutter, or a sensor fault. Single-modality systems cannot perform
cross-sensor consistency checks.

### Approach 2: Camera-Only Activation

**Strengths:** Provides semantic information - face/body detection, gesture recognition,
spatial confirmation. A positive face-detection result is a strong presence indicator.

**Weaknesses:** Sensitive to lighting conditions, occlusion, motion blur, and resource
limits. On the ESP32-S3, camera processing (DMA, format conversion, inference) competes
with other real-time tasks and can produce stale or corrupted frames.

**FT Analysis:** Vision should be used as a verifier, not an always-on primary trigger.
Keeping the camera always on wastes power and reduces privacy. The system should
activate the camera only when a lower-power sentinel (radar) indicates possible presence.

### Approach 3: Naive Camera + mmWave Fusion

**Strengths:** Heterogeneous redundancy - radar and camera fail under different conditions,
so combining them reduces the overall false-activation and missed-activation rates
relative to either sensor alone.

**Weaknesses:** Naive AND logic requires both sensors to agree, which increases
missed activations whenever one sensor experiences a transient fault. Naive OR logic
activates if either sensor is positive, which increases false activations when either
sensor is noisy. Neither fusion rule diagnoses why the sensors disagree.

**FT Analysis:** The project needs health-aware fusion, not just sensor fusion.
Detecting and classifying disagreements as transient vs. persistent is required.

### Approach 4: Uncertainty-Aware Sensor Fusion (Yu et al., 2026)

The MSCAF framework uses Dempster-Shafer evidence theory and Dirichlet-distribution-
based uncertainty modeling to adaptively weight high-confidence vs. low-confidence
sensor sources in an industrial fault-diagnosis setting.

**How it informs this project:** This project adapts the concept with a lightweight
confidence score computable in microseconds on the ESP32-S3:

```
C_i = w_s * S_i  +  w_f * F_i  +  w_t * T_i
```

where `S_i` is cross-sensor agreement, `F_i` is data freshness, and `T_i` is
transport/timing health.

### Approach 5: OOD Detection and Fault-Injection Evaluation

HOOD (Kahya et al., 2025) frames radar presence detection as rejecting
out-of-distribution (OOD) inputs - clutter, abnormal environmental conditions -
directly motivating OOD-aware thresholding in the radar health monitor of this project.

Moradi & Denil (2020) describe machine-learning-assisted fault injection for embedded
validation, supporting the design of this project's evaluation: deliberately injecting
faults of varying type, rate, and timing.

---

## 5. Proposed Fault-Tolerant Architecture

### Design Philosophy

The proposed architecture treats activation as a **fault-tolerant state machine**:

- The mmWave sensor acts as a low-power sentinel.
- When radar indicates possible presence, the camera is activated briefly for confirmation.
- A fusion layer evaluates agreement, confidence, freshness, and timing health before
  allowing full activation.

**Guiding principle:** When uncertain, fail into non-activation, not unsafe activation.
This is a fail-safe design - ambiguity triggers a safe state, not an active state.

### Architecture Layers

```
+------------------------------------------------------------------+
|                  Heterogeneous Sensor Inputs                      |
|  [mmWave UART Stream]              [Camera Frames]                |
|   presence, range, energy           visual confirmation            |
+-------------------+---------------------------+------------------+
                    |                           |
+-------------------v---------------------------v------------------+
|                  Input Integrity & Freshness                      |
|  [Radar Parser]                    [Frame Quality Monitor]        |
|   header/footer, CRC-8              brightness, blur, timestamp   |
|   timestamp, drop counter                                         |
+-------------------+---------------------------+------------------+
                    |                           |
+-------------------v---------------------------v------------------+
|                      Fault Diagnosis Layer                        |
|  [Cross-Sensor Consistency Check]                                 |
|  [Confidence & Health Scoring:  C_i = w_s*S + w_f*F + w_t*T]   |
|  [Temporal Redundancy: k-sample sliding window]                   |
|  [Fault Classifier: transient vs. persistent]                     |
+------+-------------+---------------+----------------+------------+
       |             |               |                |
  [Normal        [Soft Safe     [Hard Safe       [Degraded
  Activation]    State]         State]           Mode]
       |             |               |                |
  allow camera  suppress        block all        low-power
  allow audio   retry           isolate          only
  allow servo   revalidate      notify user
```

### FreeRTOS Task Model

| Task | Core | Priority | Function |
|---|---|---|---|
| `mmwave_rx_task` | Core 0 | High | UART receive, CRC parser, freshness stamp |
| `vision_task` | Core 1 | High | Frame capture, format, quality check, inference |
| `ft_task` | Core 0 | MAX-2 | Consistency check, scoring, FSM transitions |
| `web_server_task` | Core 1 | Low | HTTP dashboard, JPEG stream |

The **Task Watchdog Timer (TWDT)** monitors `ft_task` for stalls. A stalled task that
does not call `esp_task_wdt_reset()` within the 5-second watchdog period triggers a
panic and is logged as a timing fault.

---

## 6. Fault Model and Diagnosis Logic

### Fault Taxonomy

Faults are classified following Avizienis et al. (2004) along four axes:

**By Source:**
- **Sensing faults** - radar clutter, camera low-light, radar multipath reflection
- **Communication faults** - UART packet drops, PSRAM bus contention, camera DMA overrun
- **Timing faults** - stale frames, over-long inference windows, watchdog events
- **Environmental faults** - fan, moving curtain, reflective surface, sudden lighting change

**By Duration:**
- **Transient** - last one or a few decision windows; may self-resolve.
  Examples: brief radar glitch from a passing vehicle, single dropped UART packet.
- **Persistent** - persist across `k_hard` consecutive windows.
  Examples: camera initialization failure, misaligned radar installation.

**SDC-Like (Silent Data Corruption) Faults:** A particularly dangerous sub-class is
sensor outputs that appear valid but conflict with system state. Example: the radar
timestamp is fresh and the presence bit is set, but the camera has been dark for 10
seconds. These faults are addressed by invariant checks: if presence is asserted longer
than `T_max_present` without any camera visual confirmation, a fault is flagged.

### Fault Detection Rules

| Fault Class | Example | Observable Symptom | Response |
|---|---|---|---|
| Camera false negative | Low light / occlusion | Radar present; camera absent | Revalidate; soft safe state |
| Radar false positive | Fan / curtain / reflection | Radar present; camera no user | Delay activation; request confirmation |
| UART / transport fault | Dropped radar packet | Missing/stale radar timestamp | Parser reset; suppress activation |
| Timing fault | Camera task delay | Fusion uses stale frame | Reject frame; watchdog recovery |
| Persistent sensor fault | Camera init failure | Repeated visual failure | Hard safe state; isolate camera |
| SDC-like output | Stale plausible data | Valid-looking data conflicts with state | Invariant check; rollback health state |

### Diagnosis Decision Flow (Per Decision Window)

```
New Decision Window
        |
        +-- Radar data fresh & valid? --No--> [Suppress: UART fault]
        |
       Yes
        |
        +-- Radar reports presence? --No--> [Remain Idle: no presence]
        |
       Yes
        |
        +-- Trigger camera confirmation
        |
        +-- Frame fresh and valid? --No--> [Suppress: timing fault]
        |
       Yes
        |
        +-- Radar & camera agree? --Yes--> [Normal Activation]
        |
        No
        |
        +-- Inconsistency persistent (>= k_hard)?
                 |                        |
               Yes                        No
                |                         |
        [Hard Safe State]          [Soft Safe State]
```

---

## 7. Safe-State and Recovery Policy

### Seven-State Activation FSM

```
                         radar trigger
  [IDLE] ----------------------------------------------->[RADAR_CANDIDATE]
    ^                                                            |
    |                                                   request camera confirm
    |                                                           |
    |  session done                                    [VISUAL_CONFIRM]
    +<---------- [ACTIVE] <-------------------------------<-----+
                     |                         agree + C >= min_confidence
               health fault                                    |
                     |                                   mismatch/stale
                     v                                         |
                [SOFT_SAFE] <---------------------------------<+
                     |   ^
   k_hard exceeded   |   | sensors recovered, retry ok
   or soft_retry_ms  |   |
   timeout           v
               [HARD_SAFE]
                     |
                     | manual reset or fault cleared
                     v
                  [IDLE]

  (both sensors stale at any point) --> [DEGRADED]
  (watchdog reset / one sensor recovers) --> [IDLE]
```

### Soft Safe State (Transient Inconsistency)

Entered when a transient inconsistency is detected (disagreement fewer than `k_hard`
in the window):

- Full activation (camera inference, audio, actuation) is **suppressed**.
- The system collects another radar sample and camera frame.
- Timestamps and health scores are re-evaluated.
- If consistency is restored within `soft_retry_ms` (default 3000 ms), the system
  transitions back to Visual Confirm.
- If inconsistency persists beyond `soft_retry_ms` or reaches `k_hard` consecutive
  mismatches, the system escalates to Hard Safe State.

### Hard Safe State (Persistent Inconsistency)

Entered when a persistent inconsistency is detected (`k_hard` or more mismatches
in the window, or `soft_retry_ms` timeout):

- Automatic activation is **blocked completely**.
- The low-confidence or faulted sensor input is **isolated** from the fusion computation.
- The system optionally notifies the user (OLED message or audio tone).
- Minimal power operation: radar sentinel continues; camera is paused.
- Exit requires manual reset or watchdog-supervised sensor re-initialization.

### Recovery Block for Visual Confirmation

Following the classical Randell (1975) recovery block pattern:

1. **Primary confirmer:** face/body detector (ESP-DL HumanFaceDetect INT8).
   Acceptance test: confidence > tau_face and bounding box within valid detection region.
2. **Alternate confirmer:** frame-difference motion detector.
   Acceptance test: motion delta > delta_min in the center region of the frame.
3. **Recovery failure:** if the alternate also fails acceptance, the system remains
   in Soft Safe State and decrements the retry counter.

### Confidence Score Fusion

Per-sensor confidence computed each decision window:

```
C_i = w_s * S_i  +  w_f * F_i  +  w_t * T_i
```

Where:

```
S_i = 1.0  if sensors agree on presence/absence
      0.0  if they disagree

F_i = max(0,  1 - (t_now - t_sample) / T_stale)
        (freshness score, linearly decays from 1.0 to 0.0)

T_i = 1.0  if TWDT not triggered and no queue overflow
      0.0  if watchdog event or queue overflow detected
```

If the fused confidence C < min_confidence (default 0.60), activation is suppressed
regardless of the binary presence decision.

Default weights: `w_agreement = 0.50`, `w_freshness = 0.30`, `w_transport = 0.20`.

---

## 8. Evaluation Plan and Metrics

### Four Controllers Compared

1. **Radar-only baseline** - activate on any radar presence signal, no confirmation
2. **Camera-only baseline** - activate on any face-detected frame
3. **Naive AND/OR fusion** - no fault diagnosis; no safe states
4. **Proposed FT fusion** - confidence scoring, cross-sensor diagnosis,
   transient/persistent classification, soft/hard safe states, recovery blocks

### Fault Injection Tests

Faults are injected at the software level by intercepting sensor data before it reaches
the diagnosis layer (following Moradi & Denil, 2020):

| Fault Type | Injection Method | Target Metric |
|---|---|---|
| Radar packet drops | Drop 1-in-N UART frames randomly | FAR, MDR, fault coverage |
| Radar framing error | Corrupt SOF/EOF bytes | Fault coverage, detection latency |
| Camera frame blackout | Replace frame buffer with zeros | MDR, soft safe state rate |
| Camera motion blur | Convolve with Gaussian kernel | Fault coverage |
| Induced timing delay | vTaskDelay injection in sensor task | TWDT trigger rate |

### Environmental Stress Tests

- Low-light conditions (dim room, night-time)
- Reflective surfaces (glass partition, mirror in radar field)
- Moving clutter (fan, curtain in radar detection zone)
- Simultaneous I2S + camera DMA (PSRAM bus stress)

### Metrics

| Metric | Definition |
|---|---|
| **FAR** - False Activation Rate | Activations with no valid user / total trials |
| **MDR** - Missed Detection Rate | No activation with valid user present / total trials |
| **Fault Coverage** | Injected faults detected and correctly classified / total injected |
| **Detection Latency** | Time from fault injection to fault flag (ms) |
| **Activation Latency** | Time from user presence to first allowed activation (ms) |
| **Soft Safe Entry Rate** | Fraction of trials that enter soft safe state |

### Expected Results

- **Lower FAR** than radar-only - camera provides semantic rejection of clutter.
- **Lower MDR** than strict AND fusion - transient faults enter soft safe state and
  retry rather than blocking immediately.
- **Higher fault coverage** than naive fusion - explicit disagreement diagnosis catches
  faults that AND/OR rules silently pass through.
- **Moderate activation latency overhead** (<500 ms) due to soft-safe revalidation.
  Acceptable for an assistive system where false activations carry privacy costs.

---

## 9. Comparative Analysis

| Dimension | Radar-only | Camera-only | Naive Fusion | **Proposed** |
|---|---|---|---|---|
| Heterogeneous redundancy | No | No | Yes (implicit) | **Yes (explicit)** |
| Cross-sensor diagnosis | No | No | No | **Yes** |
| Transient / persistent classification | No | No | No | **Yes** |
| Soft / hard safe states | No | No | No | **Yes** |
| Confidence / freshness scoring | No | No | No | **Yes** |
| Fault-injection evaluation | No | No | No | **Yes** |
| Timing fault detection (TWDT) | No | No | No | **Yes** |
| Recovery block for vision | No | No | No | **Yes** |

**Radar-only gap:** No independent validation means radar clutter directly causes false
activation with no recourse.

**Camera-only gap:** High resource consumption and environmental sensitivity create
excessive missed activations under low-light conditions.

**Naive fusion gap:** AND logic causes MDR to increase whenever one sensor experiences
a transient fault. OR logic causes FAR to increase when either sensor is noisy.
Neither diagnoses the disagreement.

The proposed health-aware activation controller addresses all three gaps without
requiring a GPU or deep-learning pipeline.

---

## 10. Expected Contributions and Limitations

### Contributions

1. **Reduced false activation rate** relative to radar-only activation, because the
   camera provides semantic rejection of clutter.
2. **Reduced missed activation rate** relative to strict AND fusion, because transient
   faults enter a soft safe state and retry rather than blocking immediately.
3. **Reduced privacy and energy exposure** relative to camera-heavy activation, because
   the camera is activated on-demand only after radar trigger, and suppressed during
   faults.
4. **Explicit fault coverage** measured by fault injection, providing a quantified
   dependability characterization rather than only a nominal accuracy number.
5. **Demonstration of classical fault-tolerance principles** - redundancy, error
   detection, diagnosis, recovery blocks, and graceful degradation - applied to a
   modern ESP32-S3 edge AI activation pipeline.

### Why This Matters

For a privacy-preserving assistive platform, ambiguity should not automatically trigger
camera, audio, or actuator activation. The system should explicitly represent uncertainty
and enter a safe state until confidence improves. This is the correct fault-tolerant
design stance for a system where false activation carries privacy costs and missed
activation carries assistance costs.

### Limitations

- Uses a low-cost UART-output mmWave module, not a research-grade radar with raw
  Doppler cubes. Advanced OOD algorithms like HOOD (Kahya et al., 2025) require richer
  radar data and are used here as conceptual motivation only.
- Evaluation is room-scale with limited occupancy scenarios and limited environmental
  variation. Generalization to multi-room, multi-occupant, or outdoor settings is not
  evaluated.
- Soft-safe-state revalidation adds bounded but nonzero latency overhead. For
  applications requiring sub-100 ms activation latency this tradeoff needs re-evaluation.
- Confidence weights (w_s, w_f, w_t) and thresholds (min_confidence, k_hard, stale_ms)
  are tuned empirically in the target environment. Automatic threshold adaptation is
  left for future work.

---

## 11. Repository Structure

```
MIRA-2.0/
+-- main/
|   +-- main.c                  # App entry point, FT integration, FreeRTOS tasks
|   +-- vision_config.h         # Compile-time parameters (GPIO, FT weights, thresholds)
|   |
|   +-- mmwave_sensor.h/c       # [CPRE5450 NEW] Seeed MR24HPC1 UART driver
|   +-- ft_activation.h/c       # [CPRE5450 NEW] 7-state fault-tolerant activation FSM
|   |
|   +-- camera_control.h/c      # OV2640 init and frame capture
|   +-- vision_pipeline.h/.cpp  # Face detection (ESP-DL INT8) + skin-blob gesture
|   +-- vision_types.h          # detection_t struct, gesture labels
|   +-- web_server.h/c          # HTTP dashboard (JPEG stream, training controls)
|   +-- wifi_manager.h/c        # STA/AP Wi-Fi init
|   +-- servo_control.h/c       # Pan-tilt PWM (LEDC)
|   +-- oled_control.h/c        # SSD1306 I2C driver
|   +-- sd_card.h/c             # SPI SD card (training data storage)
|   +-- mic_capture.h/c         # I2S async microphone capture
|   +-- training.h/c            # KNN face/gesture training pipeline
|
+-- CMakeLists.txt              # Top-level ESP-IDF project file
+-- sdkconfig                   # ESP-IDF saved menuconfig
+-- idf_component.yml           # Component manager dependencies
+-- README.md
```

---

## 12. Build and Flash

**Requirements:** ESP-IDF v5.x or v6.x (tested on Windows with PowerShell)

```powershell
# Clone
git clone https://github.com/Shi6aSK/MIRA-2.0.git
cd MIRA-2.0

# Set up ESP-IDF environment (Windows PowerShell)
. $env:IDF_PATH/export.ps1

# Configure SSID/password in main/vision_config.h before building

# Build
idf.py build

# Flash and monitor (replace COM3 with your port)
idf.py -p COM3 flash monitor
```

The web dashboard is served at the ESP32's IP address on port 80 after Wi-Fi connects.
Default mmWave UART: GPIO43 (TX) / GPIO44 (RX), 115200 baud, UART1.

---

## 13. CPRE 5450 Implementation Details

### `main/mmwave_sensor.h` / `mmwave_sensor.c`

A from-scratch FreeRTOS UART driver for the Seeed 24 GHz MR24HPC1:

- Parses the sensor's proprietary binary frame protocol (SOF 0x53 / EOF 0x54)
- Validates each frame with **CRC-8** (polynomial 0x31, Maxim/Dallas 1-Wire variant)
- Exposes three presence states: `MMWAVE_STATE_NO_ONE`, `MMWAVE_STATE_MOTION`, `MMWAVE_STATE_STATIC`
- Runs as a **background task pinned to Core 0** with a mutex-protected snapshot struct
- Stale detection via `mmwave_is_stale()` and cumulative drop counting for the FT layer

Public API:

```c
esp_err_t   mmwave_init(void);
void        mmwave_get_reading(mmwave_reading_t *out);
bool        mmwave_presence_detected(void);
bool        mmwave_is_stale(void);
uint32_t    mmwave_drop_count(void);
```

### `main/ft_activation.h` / `ft_activation.c`

A **seven-state fault-tolerant activation FSM** driven by `ft_update()` each cycle:

1. Reads mmWave snapshot and camera health proxy (frame-ID stall detection)
2. Computes per-sensor confidence: `C_i = w_agreement*S + w_freshness*F + w_transport*T`
3. Evaluates cross-sensor agreement
4. Pushes a mismatch sample into a **sliding window** ring buffer of size `k_window`
5. Applies FSM transition rules; escalates to Hard Safe after `k_hard` consecutive
   mismatches or `soft_retry_ms` timeout

Default configuration (`ft_config_t`):

| Parameter | Default | Description |
|---|---|---|
| `w_agreement` | 0.50 | Cross-sensor agreement weight in C_i |
| `w_freshness` | 0.30 | Data freshness weight in C_i |
| `w_transport` | 0.20 | Transport health weight in C_i |
| `min_confidence` | 0.60 | Minimum fused confidence for activation |
| `stale_ms` | 300 ms | Frame freshness deadline |
| `soft_retry_ms` | 3000 ms | Max time in Soft Safe before escalation |
| `k_hard` | 4 | Consecutive window mismatches to trigger Hard Safe |
| `k_window` | 8 | Sliding window size for mismatch tracking |
| `FT_TASK_INTERVAL_MS` | 100 ms | FT task update period |

Public API:

```c
esp_err_t       ft_init(const ft_config_t *cfg);
ft_state_t      ft_update(bool radar_present, bool camera_present,
                           const mmwave_reading_t *radar, bool cam_fresh);
ft_status_t     ft_get_status(void);
bool            ft_activation_allowed(void);
void            ft_reset(void);
const char *    ft_state_name(ft_state_t s);
```

### Integration in `main.c`

- **`ft_task`** - FreeRTOS task pinned to Core 0 at MAX_PRIORITY - 2. Polls mmWave
  and uses frame-ID stall detection as a lightweight camera DMA health proxy. Calls
  `ft_update()` every 100 ms. Registered with TWDT.
- **FT gate in `vision_task`** - Inference is suppressed with `continue` when
  `ft_activation_allowed()` returns false. The JPEG stream for the web dashboard is
  unaffected.
- **TWDT** - Configured at boot with 5 s timeout and `trigger_panic = true`.

---

## 14. References

1. A. Avizienis, J.-C. Laprie, B. Randell, and C. Landwehr, "Basic Concepts and
   Taxonomy of Dependable and Secure Computing," IEEE Trans. Dependable Secure
   Comput., vol. 1, no. 1, pp. 11-33, 2004.

2. S. Li and R. Hishiyama, "An Indoor People Counting and Tracking System Using mmWave
   Sensor and Sub-sensors," IFAC-PapersOnLine, 2023.

3. Y. Yu et al., "A Novel Multi-Source Sensor Correlation Adaptive Fusion Framework
   with Uncertainty Quantification for Intelligent Fault Diagnosis," Reliability
   Engineering & System Safety, vol. 267, 2026.

4. M. Moradi and J. Denil, "Machine-Learning Assisted Model-Implemented Fault
   Injection," CEUR Workshop Proceedings, 2020.

5. S. M. Kahya, M. S. Yavuz, and E. Steinbach, "HOOD: Real-Time Human Presence and
   Out-of-Distribution Detection Using FMCW Radar," IEEE Trans. Radar Systems, 2025.

6. N. Zhang et al., "A Robust mmWave Radar Framework for Accurate People Counting and
   Motion Classification," Sensors, vol. 26, no. 4, 2026.

7. B. Randell, "System Structure for Software Fault Tolerance," IEEE Trans. Software
   Eng., vol. SE-1, no. 2, pp. 220-232, 1975.

8. H. Kopetz, Real-Time Systems: Design Principles for Distributed Embedded
   Applications, 2nd ed. Springer, 2011.

9. Espressif Systems, ESP32-S3 Technical Reference Manual, 2023.
   https://www.espressif.com

10. Espressif Systems, "Watchdogs," ESP-IDF Programming Guide, 2024.
    https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/system/wdts.html

11. Seeed Studio, "24 GHz mmWave Sensor - MR24HPC1," 2023.
    https://wiki.seeedstudio.com/Seeed-Studio-MR24HPC1-mmWave-Human-Static-Presence-Module-Lite/

---

*Iowa State University - CPRE 5450: Dependable Computing Systems - Spring 2026*