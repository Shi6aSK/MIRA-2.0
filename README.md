# MIRA 2.0 — Fault-Tolerant Multimodal Presence Verification

**Course:** CPRE 5450 — Dependable Computing Systems  
**Institution:** Iowa State University, Spring 2026  
**Author:** Shobhit Singh  
**Report:** [Singh_Shobhit_CPRE5450_Report.pdf](Singh_Shobhit_CPRE5450_Report.pdf)

---

## Overview

MIRA 2.0 extends the CPRE 5750 multimodal assistive robot (MIRA) with a
**fault-tolerant activation subsystem** that decides — dependably — when
the system is allowed to wake up and run camera inference, audio capture,
or servo actuation.

The core insight is that activation is a *safety boundary*, not just a
sensor-fusion problem. A false activation wastes battery and can compromise
privacy; a missed activation denies assistance to the user. MIRA 2.0
addresses both failure modes using:

- **Heterogeneous redundancy** — 24 GHz FMCW mmWave radar + OV2640 camera
- **Cross-sensor consistency checking** — sensors must agree before activation
- **Temporal redundancy** — k-sample sliding window for mismatch tracking
- **Two-tier safe-state policy** — Soft Safe (transient) → Hard Safe (persistent)
- **Watchdog-based timing fault detection** — ESP-IDF TWDT (5 s, panic=true)

---

## Hardware

| Component | Part | Notes |
|---|---|---|
| MCU | Seeed XIAO ESP32-S3 Sense | Dual-core LX7 @ 240 MHz, 8 MB PSRAM |
| Camera | OmniVision OV2640 | 240×240 RGB565, ~10 fps effective |
| mmWave sensor | Seeed MR24HPC1 (24 GHz) | FMCW, UART 115200, 0.75–6 m range |
| Display | SSD1306 OLED (I2C) | "Eyes" bitmap, status feedback |
| Audio | I2S MEMS microphone | Async capture, I2S DMA |
| Pan-tilt | 2× servo (PWM/LEDC) | User-tracking |

**Wiring (mmWave ↔ ESP32-S3):**

```
ESP32-S3 GPIO43 (UART1 TX)  →  MR24HPC1 UART RX
ESP32-S3 GPIO44 (UART1 RX)  ←  MR24HPC1 UART TX
3.3 V                        →  MR24HPC1 VCC
GND                          →  MR24HPC1 GND
```

---

## Repository Structure

```
MIRA-2.0/
├── main/
│   ├── main.c                  # App entry point, FT integration, FreeRTOS tasks
│   ├── vision_config.h         # Compile-time parameters (GPIO, FT weights, etc.)
│   │
│   ├── mmwave_sensor.h/c       # [NEW] Seeed MR24HPC1 UART driver
│   ├── ft_activation.h/c       # [NEW] 7-state fault-tolerant activation FSM
│   │
│   ├── camera_control.h/c      # OV2640 init and frame capture
│   ├── vision_pipeline.h/.cpp  # Face detection (ESP-DL) + skin-blob gesture
│   ├── vision_types.h          # detection_t, gesture labels
│   ├── web_server.h/c          # HTTP dashboard (JPEG stream, controls)
│   ├── wifi_manager.h/c        # STA/AP Wi-Fi init
│   ├── servo_control.h/c       # Pan-tilt PWM (LEDC)
│   ├── oled_control.h/c        # SSD1306 I2C driver
│   ├── sd_card.h/c             # SPI SD card (training data)
│   ├── mic_capture.h/c         # I2S async microphone
│   └── training.h/c            # KNN face/gesture training pipeline
│
├── CMakeLists.txt              # Top-level ESP-IDF project
├── sdkconfig                   # ESP-IDF saved config
├── idf_component.yml           # Component dependencies
└── README.md
```

---

## CPRE 5450 Additions (New in MIRA 2.0)

### `main/mmwave_sensor.h` / `mmwave_sensor.c`

A from-scratch FreeRTOS UART driver for the Seeed 24 GHz MR24HPC1:

- Parses the sensor's proprietary binary frame protocol (SOF `0x53` / EOF `0x54`)
- Validates each frame with **CRC-8** (polynomial `0x31`, Maxim/Dallas)
- Exposes three presence states: `NO_ONE`, `MOTION`, `STATIC`
- Runs as a **background task pinned to Core 0** with a mutex-protected snapshot
- Provides stale-detection (`mmwave_is_stale()`) and drop counting for the FT layer

### `main/ft_activation.h` / `ft_activation.c`

A **seven-state fault-tolerant activation FSM**:

```
FT_IDLE  →  FT_RADAR_CANDIDATE  →  FT_VISUAL_CONFIRM  →  FT_ACTIVE
                                         ↓ mismatch
                                    FT_SOFT_SAFE  →  FT_HARD_SAFE
                 (both stale) →  FT_DEGRADED
```

State transitions are driven by per-cycle calls to `ft_update()` which:
1. Computes per-sensor confidence: `C_i = w_agreement·S + w_freshness·F + w_transport·T`
2. Evaluates cross-sensor agreement
3. Pushes to a **sliding window** of size `k_window` for mismatch counting
4. Applies transition rules; escalates to Hard Safe after `k_hard` consecutive mismatches
   or `soft_retry_ms` timeout

Default parameters (all tunable via `ft_config_t`):

| Parameter | Default | Meaning |
|---|---|---|
| `w_agreement` | 0.50 | Weight of cross-sensor agreement in confidence |
| `w_freshness` | 0.30 | Weight of data freshness |
| `w_transport` | 0.20 | Weight of transport health |
| `min_confidence` | 0.60 | Minimum fused confidence to allow activation |
| `stale_ms` | 300 ms | Frame freshness deadline |
| `soft_retry_ms` | 3000 ms | Max time in Soft Safe before escalation |
| `k_hard` | 4 | Mismatches in window to trigger Hard Safe |
| `k_window` | 8 | Sliding window size |

### Integration in `main.c`

- **`ft_task`** — FreeRTOS task pinned to Core 0 at `MAX_PRIORITY - 2`. Polls mmWave
  and uses frame-ID stall detection as a lightweight camera-health proxy. Calls
  `ft_update()` every `FT_TASK_INTERVAL_MS` (100 ms).
- **FT gate in `vision_task`** — Inference is suppressed (`continue`) when
  `ft_activation_allowed()` returns false. The JPEG stream is unaffected.
- **TWDT** — Configured at boot to 5 s with `trigger_panic = true`. `ft_task`
  registers itself and calls `esp_task_wdt_reset()` each cycle.

---

## Build and Flash

**Requirements:** ESP-IDF v5.x or v6.x, MiKTeX (for PDF only)

```bash
# Clone
git clone https://github.com/Shi6aSK/MIRA-2.0.git
cd MIRA-2.0

# Set up ESP-IDF environment (Windows)
. $env:IDF_PATH/export.ps1

# Build
idf.py build

# Flash and monitor (replace PORT)
idf.py -p COM3 flash monitor
```

The web dashboard is served at the ESP32's IP address on port 80 after
Wi-Fi connects. Configure SSID/password in `main/vision_config.h`.

---

## Fault Model Summary

| Fault Source | Error | Failure | Handling |
|---|---|---|---|
| Low light / occlusion | Camera false negative | Missed activation | Radar-first trigger; FT gate |
| Fan / curtain / reflection | Radar false positive | False activation | Vision confirmation required |
| UART packet drop | Stale radar state | Wrong decision | `mmwave_is_stale()` → FT escalation |
| Camera DMA stall | Old frame reused | Silent wrong decision | Frame-ID stall detector in `ft_task` |
| Task stall / runaway | Missed deadline | Unsafe state | TWDT panic recovery |

---

## License

Academic project — Iowa State University, CPRE 5450, Spring 2026.  
For questions contact Shobhit Singh via Iowa State University.
