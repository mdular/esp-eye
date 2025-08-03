# ADR‑0001: Hall Index Pulse vs Magnetometer for Yaw Reference

> **Status:** Accepted
> **Date:** 2025‑08‑03
> **Supersedes / Relates‑to:** none
> **Projects:** `/controller`

---

## 1  Context

The 70 cm metal sphere (“eye”) must hold a stable yaw heading. Two candidate sensors provide an **absolute reference** to clamp drift from the 6‑DoF IMU (MPU‑6050):

| Option                                                                                           | Pros                                                                                                                     | Cons                                                                                                                                                                      |
| ------------------------------------------------------------------------------------------------ | ------------------------------------------------------------------------------------------------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **Hall‑effect index pulse** (tiny magnet at mechanical front‑centre, A3144 sensor on fixed pole) | • Immune to steel shell distortion<br>• Cheap (<1 €) & simple<br>• No I²C bus traffic<br>• Easy ISR (one pulse per pass) | • Reference is **intermittent**—only when motion crosses centre<br>• Requires mechanical mount for magnet & sensor<br>• Cannot supply continuous yaw (gap between pulses) |
| **3‑axis magnetometer** (QMC5883L/BMM150 on short boom)                                          | • Continuous absolute yaw (100 Hz) <br>• No moving parts                                                                 | • Soft‑/hard‑iron distortion from steel shell & motors<br>• Needs calibration and error detection<br>• Adds I²C traffic and fusion complexity<br>• \~3 € cost             |

Physical inspection shows the sphere’s steel shell and nearby motors severely distort the magnetic field (< 0.3 G offset). The Hall sensor, mounted externally on the pole, is unaffected.

---

## 2  Decision

We will **use a Hall‑effect index pulse as the primary absolute yaw reference**, resetting the Mahony/Madgwick filter’s yaw state whenever the pulse is detected.

* Magnet: 3 × 2 mm NdFeB glued inside the shell at mechanical 0°.
* Sensor: A3144 latch; pull‑up to 3 V3; rising edge → GPIO INT.
* Firmware: ISR posts timestamp to queue; `control_task` sets `fuse.setYaw(0)`.

Magnetometer support remains optional behind `CONFIG_FEATURE_MAGNETOMETER`, gated off by default.

---

## 3  Consequences

* **Pros**

  * Yaw drift is bounded to ±(ω·Δt/2) where Δt is time between centre crossings; acceptable for the planned motion profile.
  * Simplifies BOM; fewer calibration steps during setup.
  * Reduced I²C bus loading (important with camera running on ESP32‑CAM).
* **Cons / Mitigations**

  * If the sphere never crosses centre during a show, yaw can drift unchecked. → Operator UI includes a “recall centre” command.
  * Loss of Hall signal (loose wire) leaves system without absolute reference. → Add watchdog: if no pulse for 60 s, display warning in Web UI.
  * Cannot recover yaw immediately at boot until first centre crossing. → On boot, zero yaw; expect slight error until first pulse.

---

## 4  Alternatives Considered

1. **Outside‑boom magnetometer** — Rejected due to cabling complexity, risk of accidental knock‑off, still needs Hall for backup.
2. **Optical marker + photodiode** — More expensive and sensitive to lighting conditions.
3. **Absolute rotary encoder on yaw axis** — Mechanically intrusive; add slip ring; overkill cost.

---

## 5  Follow‑ups

* Update `/controller/components/hall_index/` driver spec.
* Draft unit tests: simulate index pulses at varying intervals and verify yaw reset.
* Web UI: show last‑pulse timestamp & error if >60 s.

---

*End of ADR‑0001*
