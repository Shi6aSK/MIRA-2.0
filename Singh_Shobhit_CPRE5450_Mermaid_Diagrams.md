# Singh_Shobhit_CPRE5450_Project — Mermaid Diagrams

Project: **Fault-Tolerant Multimodal Presence Verification and Safe Activation Policies on ESP32-S3 Using mmWave Radar and Vision**

Use these Mermaid diagrams directly in PowerPoint by exporting them as SVG/PNG from Mermaid Live Editor, VS Code Mermaid Preview, or Markdown Preview Mermaid Support.

---

## Diagram 1 — Project Scope Mapping
**Use on Slide 1 or Slide 2: Title and Scope**

```mermaid
flowchart TD
    A["Broader CPRE 5750 Project<br/>Privacy-Preserving Multimodal Assistant"] --> B["Full System Components"]
    B --> B1["ESP32-S3 Sense<br/>camera + microphone"]
    B --> B2["24 GHz mmWave<br/>presence sensing"]
    B --> B3["Pan-tilt servos<br/>user tracking"]
    B --> B4["Gesture recognition<br/>open palm / closed fist"]
    B --> B5["Android offload<br/>speech + object recognition"]

    A --> C["CPRE 5450 Focus<br/>Fault-Tolerant Activation Subsystem"]
    C --> C1["Presence verification"]
    C --> C2["Camera activation decision"]
    C --> C3["Cross-sensor consistency checking"]
    C --> C4["Safe-state policy"]
    C --> C5["Fault injection evaluation"]

    C1 --> D["Question Studied:<br/>When is the assistant allowed to wake up, sense, and interact?"]

    classDef main fill:#e8f2ff,stroke:#2b6cb0,stroke-width:2px,color:#111;
    classDef focus fill:#fff4e6,stroke:#c05621,stroke-width:2px,color:#111;
    classDef item fill:#f7fafc,stroke:#718096,color:#111;
    class A,C,D main;
    class B,B1,B2,B3,B4,B5,C1,C2,C3,C4,C5 item;
```

---

## Diagram 2 — Fault-Tolerant Systems Framing
**Use on Slide 3: Problem Statement and Failure Modes**

```mermaid
flowchart LR
    F["Fault Source"] --> E["Error State"] --> Y["System-Level Failure"] --> C["Consequence"]

    F1["Low light / occlusion"] --> E1["Camera false negative"] --> Y1["Missed activation"] --> C1["No assistance to user"]
    F2["Fan / curtain / reflection"] --> E2["Radar false positive"] --> Y2["False activation"] --> C2["Battery drain + privacy risk"]
    F3["UART packet drop"] --> E3["Stale radar state"] --> Y3["Wrong activation decision"] --> C3["Unsafe or unavailable behavior"]
    F4["ESP32 task delay"] --> E4["Old camera frame used"] --> Y4["Silent incorrect decision"] --> C4["Hard-to-detect reliability failure"]

    classDef source fill:#fff5f5,stroke:#c53030,color:#111;
    classDef error fill:#fffaf0,stroke:#dd6b20,color:#111;
    classDef fail fill:#fefcbf,stroke:#d69e2e,color:#111;
    classDef cons fill:#edf2f7,stroke:#4a5568,color:#111;
    class F,F1,F2,F3,F4 source;
    class E,E1,E2,E3,E4 error;
    class Y,Y1,Y2,Y3,Y4 fail;
    class C,C1,C2,C3,C4 cons;
```

---

## Diagram 3 — End-to-End Data Flow of the Larger System
**Use on Slide 2 or Slide 4: Application Context**

```mermaid
flowchart TD
    R["24 GHz mmWave Sensor"] --> P["Presence Trigger"]
    P --> C["ESP32-S3 Camera Capture"]
    C --> V["Vision Processing<br/>face/body/hand detection"]
    V --> T["Tracking Logic<br/>Kalman filter"]
    T --> S["Servo Control<br/>pan-tilt camera alignment"]
    V --> G["Gesture Recognition"]
    G --> A1["Open Palm:<br/>activate microphone"]
    G --> A2["Closed Fist:<br/>stop recording"]
    G --> A3["Pointing:<br/>capture object image"]
    A1 --> M["I2S Microphone Capture"]
    M --> D["Android Device<br/>speech-to-text + AI response"]
    A3 --> D
    D --> U["User Feedback<br/>audio / response / action"]

    FT["CPRE 5450 Fault-Tolerance Layer"] -. monitors .-> P
    FT -. validates .-> C
    FT -. gates .-> G
    FT -. blocks/allows .-> M

    classDef sensor fill:#e6fffa,stroke:#319795,color:#111;
    classDef compute fill:#ebf8ff,stroke:#3182ce,color:#111;
    classDef action fill:#faf5ff,stroke:#805ad5,color:#111;
    classDef ft fill:#fff5f5,stroke:#e53e3e,stroke-width:2px,color:#111;
    class R,C,M sensor;
    class P,V,T,G,D compute;
    class S,A1,A2,A3,U action;
    class FT ft;
```

---

## Diagram 4 — Proposed Fault-Tolerant Architecture
**Use on Slide 10: Proposed Architecture**

```mermaid
flowchart TD
    subgraph Inputs["Heterogeneous Sensor Inputs"]
        R1["mmWave UART Stream<br/>presence, distance gate, energy"]
        C1["Camera Frames<br/>visual confirmation"]
    end

    subgraph Precheck["Input Integrity and Freshness Checks"]
        R2["Radar Parser<br/>frame header/footer, timestamp"]
        C2["Frame Quality Monitor<br/>brightness, blur, timestamp"]
    end

    subgraph Diagnosis["Fault Diagnosis Layer"]
        D1["Cross-Sensor Consistency"]
        D2["Temporal Redundancy<br/>k-sample revalidation"]
        D3["Confidence and Health Score"]
        D4["Fault Classifier<br/>transient vs persistent"]
    end

    subgraph Policy["Safe Activation Policy"]
        S0["Normal Activation"]
        S1["Soft Safe State<br/>retry and suppress activation"]
        S2["Hard Safe State<br/>block activation + isolate input"]
        S3["Degraded Mode<br/>limited low-power sensing"]
    end

    subgraph Outputs["Controlled Outputs"]
        O1["Allow camera inference"]
        O2["Allow audio capture"]
        O3["Allow servo movement"]
        O4["Notify user if persistent fault"]
    end

    R1 --> R2 --> D1
    C1 --> C2 --> D1
    R2 --> D3
    C2 --> D3
    D1 --> D2 --> D4
    D3 --> D4
    D4 --> S0
    D4 --> S1
    D4 --> S2
    D4 --> S3
    S0 --> O1
    S0 --> O2
    S0 --> O3
    S2 --> O4
    S3 --> O4

    classDef input fill:#e6fffa,stroke:#319795,color:#111;
    classDef diag fill:#fffaf0,stroke:#dd6b20,color:#111;
    classDef safe fill:#fefcbf,stroke:#d69e2e,color:#111;
    classDef out fill:#edf2f7,stroke:#4a5568,color:#111;
    class R1,C1,R2,C2 input;
    class D1,D2,D3,D4 diag;
    class S0,S1,S2,S3 safe;
    class O1,O2,O3,O4 out;
```

---

## Diagram 5 — Runtime Sequence: Normal Activation vs Fault Handling
**Use on Slide 10 or Slide 12**

```mermaid
sequenceDiagram
    autonumber
    participant User
    participant Radar as mmWave Sensor
    participant ESP as ESP32-S3 Controller
    participant Cam as Camera Task
    participant Fuse as Diagnosis/Fusion Layer
    participant Act as Activation Outputs

    User->>Radar: Enters room or remains present
    Radar->>ESP: UART presence report + timestamp
    ESP->>Fuse: Radar state = present
    Fuse->>Cam: Request short visual confirmation
    Cam->>Fuse: Camera result + confidence + timestamp

    alt Radar and camera agree
        Fuse->>Act: Allow activation
        Act->>User: Interaction enabled
    else Temporary mismatch
        Fuse->>Act: Suppress activation
        Fuse->>ESP: Enter soft safe state
        ESP->>Radar: Request next sample window
        ESP->>Cam: Request another frame
    else Persistent mismatch
        Fuse->>Act: Block activation
        Fuse->>ESP: Enter hard safe state
        Act->>User: Notify uncertainty or fault
    end
```

---

## Diagram 6 — Diagnosis Decision Logic
**Use on Slide 11: Fault Model and Diagnosis Logic**

```mermaid
flowchart TD
    A["New Decision Window"] --> B{Radar data fresh and valid?}
    B -- No --> B1["Reject radar sample<br/>soft safe or parser reset"]
    B -- Yes --> C{Camera frame fresh and valid?}
    C -- No --> C1["Reject camera sample<br/>soft safe and retry"]
    C -- Yes --> D{Radar says presence?}

    D -- No --> E{Camera confirms user?}
    E -- No --> N["No activation<br/>idle low-power state"]
    E -- Yes --> F1["Disagreement:<br/>possible radar false negative"]

    D -- Yes --> F{Camera confirms user?}
    F -- Yes --> G["Agreement:<br/>allow activation"]
    F -- No --> F2["Disagreement:<br/>possible radar false positive or camera false negative"]

    F1 --> H{Mismatch count >= threshold?}
    F2 --> H
    B1 --> H
    C1 --> H

    H -- No --> S1["Soft safe state<br/>bounded revalidation"]
    H -- Yes --> S2["Hard safe state<br/>block activation + isolate low-confidence input"]

    classDef decision fill:#ebf8ff,stroke:#3182ce,color:#111;
    classDef normal fill:#f0fff4,stroke:#38a169,color:#111;
    classDef warn fill:#fffaf0,stroke:#dd6b20,color:#111;
    classDef safe fill:#fff5f5,stroke:#e53e3e,color:#111;
    class B,C,D,E,F,H decision;
    class G,N normal;
    class F1,F2,B1,C1,S1 warn;
    class S2 safe;
```

---

## Diagram 7 — Activation State Machine
**Use on Slide 12: Safe-State and Recovery Policy**

```mermaid
stateDiagram-v2
    [*] --> Idle

    Idle --> RadarCandidate: radar_presence_detected
    RadarCandidate --> VisualConfirm: request_camera_confirmation
    VisualConfirm --> Active: radar_agrees AND camera_confirms
    VisualConfirm --> SoftSafe: mismatch OR low_confidence OR stale_sample

    SoftSafe --> VisualConfirm: retry_within_time_window
    SoftSafe --> Idle: no_presence_after_retry
    SoftSafe --> HardSafe: mismatch_count >= N OR timeout_exceeded

    Active --> Idle: user_leaves OR session_complete
    Active --> SoftSafe: health_monitor_fault

    HardSafe --> Degraded: isolate_faulty_input
    Degraded --> Idle: manual_reset OR fault_cleared
    HardSafe --> Idle: watchdog_reset_completed

    note right of SoftSafe
        Temporary uncertainty
        Suppress full activation
        Revalidate radar + camera
    end note

    note right of HardSafe
        Persistent fault
        Block activation
        Notify user
        Preserve power and privacy
    end note
```

---

## Diagram 8 — Fault Taxonomy for the Activation Subsystem
**Use on Slide 11: Fault Model**

```mermaid
flowchart TD
    F["Faults in Activation Subsystem"]

    F --> S["Sensor Faults"]
    S --> S1["Camera false negative<br/>low light, blur, occlusion"]
    S --> S2["Camera false positive<br/>background mistaken as user"]
    S --> S3["Radar false positive<br/>fan, curtain, reflections"]
    S --> S4["Radar false negative<br/>range-gate or orientation issue"]

    F --> T["Transport and Data Faults"]
    T --> T1["UART packet drop"]
    T --> T2["Radar framing error"]
    T --> T3["Stale timestamp"]
    T --> T4["Queue overflow"]

    F --> C["Computation and Timing Faults"]
    C --> C1["Camera task deadline miss"]
    C --> C2["Fusion task delay"]
    C --> C3["Watchdog event"]
    C --> C4["Memory pressure / frame loss"]

    F --> E["Environment / OOD Faults"]
    E --> E1["Low light"]
    E --> E2["Reflective surfaces"]
    E --> E3["Moving clutter"]
    E --> E4["Unexpected human posture"]

    F --> P["Policy-Level Failures"]
    P --> P1["False activation"]
    P --> P2["Missed activation"]
    P --> P3["Unsafe privacy-sensitive sensing"]

    classDef root fill:#e8f2ff,stroke:#2b6cb0,stroke-width:2px,color:#111;
    classDef category fill:#fffaf0,stroke:#dd6b20,stroke-width:2px,color:#111;
    classDef leaf fill:#f7fafc,stroke:#718096,color:#111;
    class F root;
    class S,T,C,E,P category;
    class S1,S2,S3,S4,T1,T2,T3,T4,C1,C2,C3,C4,E1,E2,E3,E4,P1,P2,P3 leaf;
```

---

## Diagram 9 — Recovery Block Model for Visual Confirmation
**Use on Slide 12: Recovery Policy / Known Approaches**

```mermaid
flowchart TD
    A["Radar candidate detected"] --> P["Primary visual confirmer<br/>face/body detector"]
    P --> AT1{Acceptance test passed?}
    AT1 -- Yes --> OK["Accept visual confirmation"]
    AT1 -- No --> B["Backup confirmer<br/>simpler motion or frame-difference check"]
    B --> AT2{Acceptance test passed?}
    AT2 -- Yes --> DEG["Degraded confirmation<br/>allow limited activation or retry"]
    AT2 -- No --> FAIL["No reliable confirmation<br/>enter safe state"]

    AT1 -. checks .-> C1["confidence >= threshold"]
    AT1 -. checks .-> C2["frame fresh"]
    AT1 -. checks .-> C3["ROI plausible"]
    AT2 -. checks .-> C4["motion consistent with radar"]

    classDef process fill:#ebf8ff,stroke:#3182ce,color:#111;
    classDef decision fill:#fefcbf,stroke:#d69e2e,color:#111;
    classDef ok fill:#f0fff4,stroke:#38a169,color:#111;
    classDef fail fill:#fff5f5,stroke:#e53e3e,color:#111;
    class A,P,B process;
    class AT1,AT2 decision;
    class OK,DEG ok;
    class FAIL fail;
```

---

## Diagram 10 — Health-Aware Confidence Fusion Model
**Use on Slide 8 or Slide 14**

```mermaid
flowchart LR
    subgraph SensorScores["Per-Sensor Health Scores"]
        R["Radar Health R_h<br/>freshness + framing + range plausibility"]
        C["Camera Health C_h<br/>freshness + quality + visual confidence"]
        T["Timing Health T_h<br/>deadline + watchdog + queue state"]
    end

    R --> F["Fusion Score"]
    C --> F
    T --> F

    F --> Eq["C_total = w_r R_h + w_c C_h + w_t T_h"]
    Eq --> D{C_total >= threshold<br/>AND sensors consistent?}
    D -- Yes --> A["Allow activation"]
    D -- No, temporary --> S["Soft safe state"]
    D -- No, persistent --> H["Hard safe state"]

    classDef score fill:#ebf8ff,stroke:#3182ce,color:#111;
    classDef decision fill:#fefcbf,stroke:#d69e2e,color:#111;
    classDef good fill:#f0fff4,stroke:#38a169,color:#111;
    classDef safe fill:#fff5f5,stroke:#e53e3e,color:#111;
    class R,C,T,F,Eq score;
    class D decision;
    class A good;
    class S,H safe;
```

---

## Diagram 11 — ESP32-S3 FreeRTOS Task Model
**Use on Slide 4 or Slide 10**

```mermaid
flowchart TD
    subgraph Core0["ESP32-S3 Core 0"]
        RTask["Radar UART Task<br/>parse frames + timestamp"]
        FTask["Fusion/Diagnosis Task<br/>state machine + safe policy"]
        WTask["Watchdog/Health Task<br/>deadline and queue checks"]
    end

    subgraph Core1["ESP32-S3 Core 1"]
        CTask["Camera Task<br/>capture frame"]
        VTask["Vision Task<br/>quality + confirmation"]
        ATask["Audio/Activation Task<br/>enabled only after safe activation"]
    end

    RTask --> Q1["Radar Queue"] --> FTask
    CTask --> Q2["Frame Queue"] --> VTask --> FTask
    WTask --> FTask
    FTask --> G1["Activation Gate"]
    G1 --> ATask
    G1 --> Servo["Servo Enable"]
    G1 --> Notify["User Notification"]

    classDef core fill:#edf2f7,stroke:#4a5568,stroke-width:2px,color:#111;
    classDef task fill:#ebf8ff,stroke:#3182ce,color:#111;
    classDef queue fill:#fffaf0,stroke:#dd6b20,color:#111;
    classDef gate fill:#fefcbf,stroke:#d69e2e,color:#111;
    class RTask,FTask,WTask,CTask,VTask,ATask task;
    class Q1,Q2 queue;
    class G1 gate;
```

---

## Diagram 12 — Fault Injection and Evaluation Workflow
**Use on Slide 13: Evaluation Plan**

```mermaid
flowchart TD
    A["Define Baselines"] --> B1["Radar-only activation"]
    A --> B2["Camera-only activation"]
    A --> B3["Naive AND/OR fusion"]
    A --> B4["Proposed fault-tolerant fusion"]

    B1 --> C["Run Test Scenarios"]
    B2 --> C
    B3 --> C
    B4 --> C

    C --> D1["Nominal room trials"]
    C --> D2["Environmental stress<br/>low light, reflections, moving clutter"]
    C --> D3["Injected faults<br/>packet drops, blur, stale frames, timing delay"]

    D1 --> M["Collect Metrics"]
    D2 --> M
    D3 --> M

    M --> M1["False activation rate"]
    M --> M2["Missed detection rate"]
    M --> M3["Fault coverage"]
    M --> M4["Detection latency"]
    M --> M5["Energy/camera duty cycle"]

    M1 --> N["Compare Dependability Tradeoffs"]
    M2 --> N
    M3 --> N
    M4 --> N
    M5 --> N

    N --> O["Expected result:<br/>lower unsafe activation at small latency cost"]

    classDef setup fill:#ebf8ff,stroke:#3182ce,color:#111;
    classDef test fill:#fffaf0,stroke:#dd6b20,color:#111;
    classDef metric fill:#fefcbf,stroke:#d69e2e,color:#111;
    classDef result fill:#f0fff4,stroke:#38a169,color:#111;
    class A,B1,B2,B3,B4 setup;
    class C,D1,D2,D3 test;
    class M,M1,M2,M3,M4,M5,N metric;
    class O result;
```

---

## Diagram 13 — Baseline vs Proposed Solution Comparison
**Use on Slide 14: Analysis of Existing Solutions**

```mermaid
flowchart LR
    subgraph Baselines["Known / Baseline Solutions"]
        ROnly["Radar-only<br/>low power, privacy-friendly<br/>weak semantic check"]
        COnly["Camera-only<br/>semantic confirmation<br/>lighting/privacy/resource issues"]
        Naive["Naive fusion<br/>simple AND/OR<br/>no fault diagnosis"]
    end

    subgraph Proposed["Proposed Fault-Tolerant Controller"]
        HR["Health-aware radar + camera fusion"]
        TR["Temporal redundancy"]
        DX["Cross-sensor diagnosis"]
        SS["Soft/hard safe states"]
        FI["Fault-injection validation"]
    end

    ROnly --> Gap["Gap: no independent validation"]
    COnly --> Gap2["Gap: high resource and environmental sensitivity"]
    Naive --> Gap3["Gap: disagreement not diagnosed"]

    Gap --> HR
    Gap2 --> HR
    Gap3 --> DX
    HR --> SS
    TR --> SS
    DX --> SS
    SS --> FI

    classDef base fill:#fffaf0,stroke:#dd6b20,color:#111;
    classDef gap fill:#fff5f5,stroke:#e53e3e,color:#111;
    classDef prop fill:#f0fff4,stroke:#38a169,color:#111;
    class ROnly,COnly,Naive base;
    class Gap,Gap2,Gap3 gap;
    class HR,TR,DX,SS,FI prop;
```

---

## Diagram 14 — Conservative Activation Policy Model
**Use on Slide 15: Expected Contribution**

```mermaid
flowchart TD
    A["Policy Goal:<br/>avoid unsafe activation"] --> B["Only activate when all safety gates pass"]

    B --> G1{Sensor data fresh?}
    G1 -- No --> S["Suppress activation"]
    G1 -- Yes --> G2{Radar-camera consistency?}
    G2 -- No --> S
    G2 -- Yes --> G3{Confidence above threshold?}
    G3 -- No --> S
    G3 -- Yes --> G4{Deadline satisfied?}
    G4 -- No --> S
    G4 -- Yes --> ACT["Activate interaction"]

    S --> R["Revalidate or enter safe state"]
    R --> H{Persistent fault?}
    H -- No --> B
    H -- Yes --> HS["Hard safe state<br/>block + notify + isolate"]

    classDef goal fill:#e8f2ff,stroke:#2b6cb0,stroke-width:2px,color:#111;
    classDef decision fill:#fefcbf,stroke:#d69e2e,color:#111;
    classDef activate fill:#f0fff4,stroke:#38a169,color:#111;
    classDef safe fill:#fff5f5,stroke:#e53e3e,color:#111;
    class A,B goal;
    class G1,G2,G3,G4,H decision;
    class ACT activate;
    class S,R,HS safe;
```

---

## Diagram 15 — Final Report Logic Model
**Use on conclusion slide or backup slide**

```mermaid
flowchart TD
    P["Problem:<br/>single-sensor activation is brittle"] --> K["Known Approaches"]
    K --> K1["Radar-only presence sensing"]
    K --> K2["Camera-only visual confirmation"]
    K --> K3["Naive sensor fusion"]
    K --> K4["Uncertainty-aware fusion / OOD detection"]
    K --> K5["Fault injection validation"]

    K1 --> A["Analysis:<br/>average accuracy is not enough"]
    K2 --> A
    K3 --> A
    K4 --> A
    K5 --> A

    A --> S["Suggestion:<br/>health-aware activation controller"]
    S --> S1["Cross-sensor consistency"]
    S --> S2["Temporal revalidation"]
    S --> S3["Fault classification"]
    S --> S4["Soft/hard safe states"]
    S --> S5["Fault coverage evaluation"]

    S1 --> C["Contribution:<br/>dependable activation boundary for ESP32-S3 assistive system"]
    S2 --> C
    S3 --> C
    S4 --> C
    S5 --> C

    classDef prob fill:#fff5f5,stroke:#e53e3e,color:#111;
    classDef known fill:#fffaf0,stroke:#dd6b20,color:#111;
    classDef analysis fill:#ebf8ff,stroke:#3182ce,color:#111;
    classDef sugg fill:#f0fff4,stroke:#38a169,color:#111;
    class P prob;
    class K,K1,K2,K3,K4,K5 known;
    class A analysis;
    class S,S1,S2,S3,S4,S5,C sugg;
```

---

## Suggested Slide Mapping Summary

| Slide | Recommended Diagram |
|---|---|
| Slide 1–2 | Diagram 1: Project Scope Mapping |
| Slide 3 | Diagram 2: Fault-Tolerant Systems Framing |
| Slide 4 | Diagram 3 or Diagram 11 |
| Slide 5–7 | Diagram 13: Baseline vs Proposed Comparison |
| Slide 8 | Diagram 10: Confidence Fusion Model |
| Slide 9 | Diagram 12: Fault Injection Workflow |
| Slide 10 | Diagram 4: Proposed Architecture |
| Slide 11 | Diagram 8 or Diagram 6 |
| Slide 12 | Diagram 7 or Diagram 9 |
| Slide 13 | Diagram 12 |
| Slide 14 | Diagram 13 |
| Slide 15 | Diagram 14 or Diagram 15 |

