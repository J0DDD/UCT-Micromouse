# System Identification Config Security & Anti-Gaming Specification

## Executive Summary
This document defines the security boundaries, physical plausibility limits, and anti-gaming validation rules for student-submitted System Identification configuration files (`sim_config.json` / `simulation_config.json`) in the UCT Micromouse autograder.

---

## 1. Problem Statement & Threat Model
To achieve parity between the physical robot and the simulation environment, students are permitted to perform System Identification (SysID) on their hardware and submit custom chassis parameters.

### Exploit Vectors
Without strict server-side validation, adversarial prompts or students could submit artificial parameters to game the autograder:
* **Superhuman Speeds:** Setting `max_speed: 50.0 m/s` to explore the maze in $< 0.5\text{ s}$.
* **Zero-Inertia / Instant Acceleration:** Setting `dead_band: 0.0` and `tau: 0.001 s`.
* **Inflated Wheel Radii:** Setting `wheel_radius: 10.0 m` so one encoder tick covers dozens of meters.
* **Noise / Perturbation Evasion:** Setting `slip: 0.0` and `gyro_noise: 0.0` to avoid implementing complementary/Kalman filtering.
* **Environment Tampering:** Overriding `maze.block_dim` to shrink the maze geometry.

---

## 2. Multi-Tier Security & Validation Architecture

### Tier 1: Immutable Environment & Protected Attributes
The autograder simulator strictly ignores student overrides for all environmental constants:
* **Maze Dimensions:** `grid_rows = 4`, `grid_cols = 6`, `block_dim = 0.20 m`, `wall_thickness = 0.006 m`.
* **Target Room Location:** Fixed to the central 2x2 target plaza.
* **Sensor Bounding Box:** TOF sensors cannot be placed outside the robot's physical collision radius ($r \le 0.07\text{ m}$).

---

### Tier 2: Physical Plausibility Bounding Box (Whitelist Clamping)
All submitted parameters in `sim_config.json` are clamped to strict physical ranges corresponding to the standard 6V N20 DC motor and 65 mm wheel chassis:

| Parameter | Nominal | Allowed Range | Physical Constraint / Rationale |
| :--- | :--- | :--- | :--- |
| `wheel_radius` | $0.0325\text{ m}$ | $[0.0300, 0.0350]\text{ m}$ | 65 mm rubber wheel manufacturing variance ($\pm 7\%$) |
| `axle_half_length` | $0.0540\text{ m}$ | $[0.0480, 0.0600]\text{ m}$ | Wheelbase track width mechanical tolerance ($\pm 10\%$) |
| `max_speed` | $0.40\text{ m/s}$ | $[0.20, 0.55]\text{ m/s}$ | 6V N20 motor mechanical RPM ceiling |
| `dead_band_l` / `dead_band_r` | $58.0 / 62.0$ | $[30.0, 75.0]\text{ PWM}$ | Physical DC motor brush breakaway stiction |
| `tau` (Motor Time Constant) | $0.088\text{ s}$ | $[0.040, 0.160]\text{ s}$ | Rotor + gearbox electro-mechanical inertia |
| `ticks_per_rot` | $1170.0$ | $[1000.0, 1300.0]$ | Magnetic encoder disc resolution |
| `mass` | $0.238\text{ kg}$ | $[0.180, 0.320]\text{ kg}$ | Battery + chassis weight |

*Any parameter submitted outside these bounds is automatically clamped to the threshold with an explicit feedback note.*

---

### Tier 3: CLI Authority Overrides (Hidden Stress Tests)
Hidden test runs (Tests 2–6) inject non-negotiable stress perturbations via CLI arguments (`--imbalance`, `--slip`, `--seed`). In `physics_sim.py`, CLI arguments always take precedence over `sim_config.json`, ensuring students cannot bypass motor imbalance or slip challenges.

---

### Tier 4: Telemetry Log Cross-Validation & Hardware Constraints

When evaluating the Final Demo submission, the autograder cross-checks the submitted `sim_config.json` against the physical telemetry log (`run_log.jsonl`):

#### Hardware Generation Differences:
1. **2026 Boards (External SPI Flash):**
   * Features a 1 MB SPI NOR flash partition (`ZD25WQ80C`), providing **20–25 minutes** of continuous 25 Hz telemetry.
   * Enables deep multi-run analysis across full maze exploration and speed runs.
2. **Legacy 2025 Boards (Internal Flash Only):**
   * Falls back to a 64 KB internal MCU Flash partition (`0x08070000` to `0x0807FFFF`), providing **60–90 seconds** of logging.
   * **Validation Rule:** The parser inspects the `"board"` field in the `log_header` (`"board": "2025"` vs `"board": "2026"`). For 2025 boards, a ~60-second log is treated as a complete, valid sample.

#### Cross-Validation Metrics (SysID Verification):
* **Max Speed Check:** If `sim_config.json` specifies `max_speed = 0.50 m/s`, the physical telemetry log must show maximum sustained encoder velocities reaching at least $0.20\text{ m/s}$.
* **Dead-band Check:** If `sim_config.json` specifies `dead_band = 35 PWM`, the log must verify that the robot actually moves at low PWM commands.
* **Gyro Scale & Bias:** The stationary bias in the log header is cross-checked against the declared `imu.gyro_bias_std`.

---

## 3. Implementation Status
* **File:** `tools/autograder/SYSID_CONFIG_SECURITY_AND_BOUNDING.md`
* **Applicable Assignments:** Final Demo (`final_demo`), Milestone 2 (`milestone2_maze`), Milestone 1 (`milestone1_square`).
* **Integrity Gate:** Managed centrally via `grade_runner.py` and `physics_sim.py`.
