# Project Epics & Back‑log (master list)

> **Purpose:** Single backlog snapshot covering all functionality discussed so far.
> **How to use:** Split each epic into its own `epic‑XXXX.md` when you start planning sprints.
> \*\*Last‑updated: 2025‑08‑03 (rev 0.2)

---

## Legend

* **E#** – Epic identifier
* **T#** – Task index inside the epic
* **⏳** – Needs hardware arrival or external info
* **🔍** – Research/spike
* **💻** – Implementation
* **🧪** – Test
* **📄** – Docs

---

## E1  Hardware Bring‑up & Platform Migration

| ID    | Type | Description                                             | Notes                           |
| ----- | ---- | ------------------------------------------------------- | ------------------------------- |
| E1‑T1 | 💻   | ESP32‑CAM minimal build: blink + serial + Wi‑Fi STA     | Already builds; baseline commit |
| E1‑T2 | 💻   | Integrate camera component; confirm stream FPS ≥ 15 FPS | Pinout check AI‑Thinker board   |
| E1‑T3 | 🔍   | Measure current draw (idle / streaming)                 | Add to power budget sheet       |
| E1‑T4 | ⏳ 💻 | Port build to ESP32 dev board (camera disabled)         | After board arrives             |
| E1‑T5 | 🧪   | Verify boot/flash on both boards via CI matrix          | GitHub Actions cross‑build      |

---

## E2  Sensor Fusion & Drift Control

| ID    | Type | Description                                             |
| ----- | ---- | ------------------------------------------------------- |
| E2‑T1 | 💻   | Add `mpu6050_driver` component (I2Cdev base)            |
| E2‑T2 | 💻   | Implement `imu_task` @ 200 Hz (Madgwick)                |
| E2‑T3 | 🧪   | Unit test: quaternion stability vs simulated gyro drift |
| E2‑T4 | 💻   | `hall_index` GPIO ISR + queue                           |
| E2‑T5 | 💻   | Yaw reset logic in `control_task`                       |
| E2‑T6 | 📄   | Write calibration CLI commands (gyro, accel)            |
| E2‑T7 | 🔍   | Evaluate runtime bias tracking vs fixed offset          |

---

## E3  Motor Control & PID Loops

| ID    | Type | Description                                          | Notes           |
| ----- | ---- | ---------------------------------------------------- | --------------- |
| E3‑T1 | 🔍   | Gather motor driver specs from on‑site hardware      | ⏳ external info |
| E3‑T2 | 💻   | Create `motor_drv` component (BTS7960 OR existing)   |                 |
| E3‑T3 | 💻   | Implement `control_task` with dual PID (pitch/roll)  |                 |
| E3‑T4 | 💻   | Add yaw PID & slew‑rate limit                        |                 |
| E3‑T5 | 🧪   | Hardware‑in‑loop test: step response, overshoot < 5° |                 |
| E3‑T6 | 🔍   | Investigate encoder add‑on feasibility               |                 |

---

## E4  Web UI & API

| ID    | Type | Description                                                     |
| ----- | ---- | --------------------------------------------------------------- |
| E4‑T1 | 💻   | ESPAsyncWebServer + WebSocket under `/ws`                       |
| E4‑T2 | 💻   | JSON command schema: `{cmd:"level"}`, `{cmd:"point",yaw,pitch}` |
| E4‑T3 | 💻   | Stream quaternion @ 10 Hz to UI                                 |
| E4‑T4 | 💻   | Browser 3‑D viewer (three‑js) prototype                         |
| E4‑T5 | 🧪   | Latency < 200 ms end‑to‑end test                                |
| E4‑T6 | 📄   | User guide: Wi‑Fi setup & web controls                          |

---

## E5  Calibration & Diagnostics

| ID    | Type | Description                                      |
| ----- | ---- | ------------------------------------------------ |
| E5‑T1 | 💻   | CLI command: `calib gyro` (3 s average)          |
| E5‑T2 | 💻   | CLI command: `calib accel six‑face`              |
| E5‑T3 | 💻   | CLI command: `calib hall` (record zero offset)   |
| E5‑T4 | 🧪   | Verify offsets saved / restored from NVS         |
| E5‑T5 | 💻   | Web UI panel: last pulse timestamp & drift alert |

---

## E6  Remote Control Tool (Future Project)

| ID    | Type | Description                                                   |
| ----- | ---- | ------------------------------------------------------------- |
| E6‑T1 | 🔍   | Choose tech stack (Flutter, React Native, second ESP32, etc.) |
| E6‑T2 | 📄   | Draft `/remote/architecture.md`                               |
| E6‑T3 | ⏳    | PoC: connect to WebSocket & send point commands               |

---

## E7  CI / DevOps

| ID    | Type | Description                                           |
| ----- | ---- | ----------------------------------------------------- |
| E7‑T1 | 💻   | GitHub Actions: build matrix (ESP32‑CAM vs dev board) |
| E7‑T2 | 💻   | Unit test runner (Unity)                              |
| E7‑T3 | 💻   | Upload firmware artefacts as build outputs            |
| E7‑T4 | 💻   | CodeQL / static‑analysis step                         |

---

## E8  Documentation & AI Integration

| ID    | Type | Description                                              |
| ----- | ---- | -------------------------------------------------------- |
| E8‑T1 | 📄   | Finish splitting epics into individual `epic‑XXXX.md`    |
| E8‑T2 | 📄   | Maintain `/docs/adr-*` for major decisions               |
| E8‑T3 | 📄   | Keep `/docs/rules/ai_guidelines.md` up‑to‑date           |
| E8‑T4 | 💻   | `.github/copilot-instructions.md` stub pointing to rules |

---

## E9  Safety & Robustness

| ID    | Type | Description                                          |
| ----- | ---- | ---------------------------------------------------- |
| E9‑T1 | 💻   | Enable task watchdog on core 0                       |
| E9‑T2 | 💻   | Add Hall‑pulse timeout alert (60 s)                  |
| E9‑T3 | 💻   | Emergency‑stop CLI & Web UI command (`cmd:"estop"`)  |
| E9‑T4 | 🧪   | Fault‑injection test: kill Hall ISR, verify fallback |

---

## E10  Mechanical & Power

| ID     | Type | Description                                         |
| ------ | ---- | --------------------------------------------------- |
| E10‑T1 | 🔍   | Document mechanical rotation limits for each axis   |
| E10‑T2 | 💻   | Firmware soft‑limits (stop commands near end‑stops) |
| E10‑T3 | 🔍   | Finalise power‑supply sizing & fuse selection       |

---

## E11  OTA Update & Image Signing

| ID     | Type | Description                                                       | Notes                           |
| ------ | ---- | ----------------------------------------------------------------- | ------------------------------- |
| E11‑T1 | 🔍   | Research ESP‑IDF OTA APIs (HTTP/S) and partition strategy         | Include dual‑bank layout sizing |
| E11‑T2 | 📄   | **ADR‑0002**: Decide on OTA approach (signed images vs unsecured) | Pending decision                |
| E11‑T3 | 💻   | Implement OTA WebSocket command (`cmd:"ota",url`)                 |                                 |
| E11‑T4 | 💻   | Add OTA page in Web UI with progress bar and checksum validation  |                                 |
| E11‑T5 | 🧪   | Fault‑injection test: corrupt image triggers rollback             |                                 |

---

## E12  Power‑On Self Test (POST)

| ID     | Type | Description                                       |
| ------ | ---- | ------------------------------------------------- |
| E12‑T1 | 💻   | LED blink heartbeat + UART log at boot            |
| E12‑T2 | 💻   | Sensor self‑check: gyro variance, I²C bus scan    |
| E12‑T3 | 💻   | Report POST status via WebSocket JSON (`post_ok`) |
| E12‑T4 | 🧪   | Inject IMU failure, verify POST error path        |

---

## E13  User Documentation & Manual

| ID     | Type | Description                                                 |
| ------ | ---- | ----------------------------------------------------------- |
| E13‑T1 | 📄   | Draft operator manual: setup, calibration, control commands |
| E13‑T2 | 📄   | Create quick‑start sheet (PDF generated from Markdown)      |
| E13‑T3 | 📄   | Add troubleshooting flowchart section                       |
| E13‑T4 | 📄   | Version docs per firmware release                           |

---

## E14  Logging & Monitoring Enhancements

| ID     | Type | Description                                                    | Notes |
| ------ | ---- | -------------------------------------------------------------- | ----- |
| E14‑T1 | 💻   | Implement log levels (ERROR/WARN/INFO/DEBUG) via ESP\_LOGx     |       |
| E14‑T2 | 💻   | Web UI: scrollbox displaying real‑time logs with on/off toggle |       |
| E14‑T3 | 💻   | Socket message type `log:{level,msg}`; CLI toggle command      |       |
| E14‑T4 | 🧪   | Stress test: 500 log lines/s without WebSocket drops           |       |

---
