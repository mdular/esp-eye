# Mechanical‑Sphere Controller – Project Architecture

> **Version:** 0.1
> **Status:** Draft (initial scaffold)
> **Last‑updated:** 2025‑08‑03

---

## 1  Project Scope & Goals

| Objective                                                                            | Success Metric                         |
| ------------------------------------------------------------------------------------ | -------------------------------------- |
| **Self‑level** the 70 cm metal sphere to horizon (pitch = roll = 0 °) after power‑up | < 1 ° residual tilt within 3 s         |
| **Target‑point** the lit iris to any yaw angle sent over Wi‑Fi                       | ≤ ±1 ° steady‑state error              |
| **Remote control** via WebSocket API & browser UI (ESP32 access‑point &/or station)  | < 200 ms command→motion latency        |
| **Robustness** against drift & environment (steel shell, temperature, motor noise)   | Holds spec for ≥ 60 min continuous run |

---

## 2  System‑Level Block Diagram

```
┌────────────┐        I²C           ┌──────────────┐
│  MPU‑6050  │───────>──────────────│   ESP32 MCU   │
├────────────┤                     ├──────────────┤
│ 6‑DoF IMU  │          GPIO        │  FreeRTOS     │
└────────────┘          ▲           │  Tasks:       │
                       │           │  • imu_task    │
┌────────────┐  Hall    │ EXT_INT   │  • control     │
│ Hall Probe │──Pulse───┘           │  • motor_drv   │
└────────────┘                      │  • websocket   │
                                    └───────┬────────┘
                                            │ PWM
                                            ▼
                                   ┌────────────────┐
                                   │  H‑Bridge(s)   │
                                   └────────────────┘
                                            │
                                            ▼
                                       DC Motors
```

*The Hall sensor delivers an **index pulse** whenever the sphere passes mechanical front‑centre — this may occur irregularly because the yaw range is mechanically limited. The IMU handles fast tilt sensing. A complementary filter (Madgwick/Mahony) runs at 200 Hz; when an index pulse is detected yaw is re‑zeroed, otherwise the filter free‑runs until the next pulse.*

---

## 3  Hardware BOM (rev‑A)

| Ref | Part                           | Qty | Notes                                      |
| --- | ------------------------------ | --- | ------------------------------------------ |
| U1  | ESP32‑WROOM‑32E dev board      | 1   | 240 MHz dual‑core, 4 MB flash              |
| U2  | MPU‑6050 breakout              | 1   | 6‑DoF IMU, 3.3 V, 400 kHz I²C              |
| U3  | A3144 Hall switch              | 1   | Mount on fixed pole; add 10 k pull‑up      |
| M?  | DC gear‑motors                 | 2   | Pre‑installed on site – ⚠ TORQUE DATA TBD  |
| H1  | BTS7960 43 A H‑bridge          | 2   | Or reuse existing drivers (specs TBD)      |
| PWR | 12–24 V PSU                    | 1   | Size per motor stall current + 50 % margin |
| ENC | (Optional) Incremental encoder | 2   | Adds velocity feedback – future rev        |

---

## 4  Firmware Architecture (ESP‑IDF)

### 4.1  Task Map

| Task              | Core | Period        | Stack | Responsibilities                                                  |
| ----------------- | ---- | ------------- | ----- | ----------------------------------------------------------------- |
| **imu\_task**     | 0    | 5 ms (200 Hz) | 3 k   | Read MPU‑6050, run Madgwick, post quaternion to queue             |
| **control\_task** | 0    | 5 ms          | 3 k   | Compute PID for pitch/roll/yaw; apply Hall yaw‑zero; send PWM cmd |
| **motor\_task**   | 0    | event‑driven  | 3 k   | Translate duty cycle → H‑bridge; fault detect                     |
| **web\_task**     | 1    | async         | 6 k   | HTTP + WebSocket server; OTA updates                              |
| **logging\_task** | 0    | 20 ms         | 2 k   | Flush ring buffer to UART / file                                  |

> **RTOS Guidelines**
> • Pin all real‑time motion‑control tasks (imu, control, motor) to **core 0**. Run Wi‑Fi, HTTP/WebSocket, and other non‑real‑time tasks on **core 1**.
> • Use `xQueueOverwrite` for latest‑only sensor data (no backlog).
> • Keep ISR work < 50 µs; defer to tasks.

### 4.2  Module Layout

```
/components
  mpu6050_driver/
  hall_index/
  pid/
  controller/
  motor_drv/
  websocket_srv/
  utils/
main/
  app_main.c
```

### 4.3  Practical Tips (core‑allocation)

1. **Affinity macros** – pin tasks explicitly so scheduling is deterministic:

   ```c
   xTaskCreatePinnedToCore(imu_task,    "imu",   3072, NULL, 6, NULL, 0);
   xTaskCreatePinnedToCore(control_task,"ctrl",  3072, NULL, 6, NULL, 0);
   xTaskCreatePinnedToCore(motor_task,  "motor", 3072, NULL, 5, NULL, 0);
   xTaskCreatePinnedToCore(web_task,    "web",   6144, NULL, 3, NULL, 1);
   ```
2. **Shared mailbox** – Use a single‑element `xQueueOverwrite` for control → motor PWM commands to avoid queuing delays.
3. **Watchdog** – Enable the task watchdog (`CONFIG_ESP_TASK_WDT`) on core 0 so any stall in the real‑time loop triggers a reset.

Each component owns its header; public APIs live in `/include`.  Use **C ++** (.cpp) for class‑heavy modules (e.g., sensor drivers) but keep **C linkage** at the boundary to ESP‑IDF.

---

## 5  Repository & Build Conventions

The repository is a **mono‑repo** with a **single top‑level `docs/` folder**.  Each project keeps its design docs in a sub‑folder there, letting us share cross‑project ADRs while avoiding scattered documentation.

| Path           | Purpose                                                                                          | Build System    |
| -------------- | ------------------------------------------------------------------------------------------------ | --------------- |
| `/controller/` | ESP‑IDF firmware that lives on the sphere itself (everything described in `docs/controller/`).   | ESP‑IDF + CMake |
| `/remote/`     | (Future) dedicated remote‑control application or firmware (ESP32, Flutter app, or desktop tool). | TBD             |
| `/docs/`       | Repository‑wide documentation root.                                                              | Markdown        |

### 5.1  Root layout (rev‑B)  Root layout (v‑A)

```text
.
├── docs/
│   ├── controller/
│   │   └── architecture.md   # ← this file
│   └── remote/               # Docs for future remote project
├── controller/
│   ├── CMakeLists.txt        # esp‑idf default
│   ├── sdkconfig.defaults
│   ├── components/
│   ├── main/
│   ├── test/
│   └── tools/
└── remote/
    ├── README.md             # Placeholder until remote project kicks off
    └── src/
```

* CI walks each first‑level folder (`controller`, `remote`) and runs the appropriate build/test pipeline.
* Docs **live only in `/docs/`**—link or include them where needed in CI artefacts.
* `sdkconfig.defaults` is local to `/controller/`; the remote app chooses its own stack.
* `sdkconfig.defaults` stays local to `/controller/` so the remote app can pick any stack it needs.
* Docs live under the project they belong to; cross‑project ADRs go in `/docs/common/` if needed.

---

## 6  Coding Rules & Style  Coding Rules & Style

1. **Language** – C17 or C++17 as needed; prefer plain C for RTOS glue.
2. **Formatting** – clang‑format (`.clang-format`) runs pre‑commit. 80‑col soft, 120‑col hard.
3. **Naming** – `snake_case` for C symbols; `CamelCase` for C++ classes.
4. **Assertions** – Use `ESP_ERROR_CHECK` for all IDF calls; keep asserts enabled in release.
5. **Logging** – `ESP_LOGx` macros; tag = module name. No printf in ISR.
6. **Unit tests** – Write Unity tests for pure functions; hardware mocks live under `test/mocks`.
7. **Documentation** – Doxygen comments for every public API; markdown ADRs under `/docs/adr‑YYYY‑MM‑DD‑title.md` for significant decisions.

---

## 7  Calibration & Test Strategy & Test Strategy

1. **Gyro bias** – Auto‑average 3 s at boot; store in NVS.
2. **Accel bias** – Six‑face calibration CLI command; persist.
3. **Hall index** – CLI: rotate manually, verify pulse at yaw 0.
4. **End‑to‑end** – Hardware‑in‑loop script spins sphere ±180 ° and checks after return drift < 1 °.

---

## 8  Open Questions / TODO Questions / TODO

| ID      | Topic                   | Notes                                                                         |                                                       |
| ------- | ----------------------- | ----------------------------------------------------------------------------- | ----------------------------------------------------- |
| HW‑01   | **Motor driver specs**  | Confirm voltage, stall current, control signal type of existing site drivers. |                                                       |
| HW‑02   | Encoder availability    | Are shaft encoders present? If so, spec & wiring.                             |                                                       |
| FW‑01   | OTA update policy       | Allow remote firmware push? Security implications.                            |                                                       |
| DOC‑01  | Add ADRs                | Capture decision to use Hall index over magnetometer.                         |                                                       |
| PROJ‑01 | Remote project scaffold | Decide tech stack & repo structure for `/remote/`.                            |                                                       |
| MECH‑01 | Rotation limits         | Determine mechanical travel limits for each axis and safe operating range.    | Capture decision to use Hall index over magnetometer. |

---

## 9  Next Steps Steps

1. Gather motor‑driver datasheets → update `/docs/adr‑2025‑xx‑motor‑driver.md`.
2. Flesh out `mpu6050_driver` component (use Jeff Rowberg I2Cdev as reference).
3. Create `hall_index` ISR on GPIO → event group.
4. Implement basic leveling loop & log quaternion to WebSocket for visual debug.
5. Draft CI workflow under `/tools/ci`.

---
