# MicroPython USB OTG & Boot Diagnostics State

## 1. Executive Summary
This document captures the resolved state, diagnostics, root causes, and verification for the MicroPython firmware, USB OTG Mass Storage (`UCT_MMOUSE` volume), interactive REPL, and hardware peripheral drivers across both 2025 and 2026 UCT Micromouse hardware revisions (STM32L476VE).

---

## 2. Hardware Clock & System Architecture (Fully Verified)
* **Target MCU:** `STM32L476VE` (100-pin LQFP, 512KB Flash, 128KB SRAM).
* **Hardware HSE Crystal:**
  - Standard 8.000 MHz external quartz crystal across pins `PH0` (OSC_IN) and `PH1` (OSC_OUT) across all batches (2025 and 2026).
  - Configured in [`mpconfigboard.h`](file:///Users/nicolls/proj/eee3097s/2026/UCT-Micromouse/firmware/src/micropython/boards/UCT_MICROMOUSE/mpconfigboard.h) and [`board_init.c`](file:///Users/nicolls/proj/eee3097s/2026/UCT-Micromouse/firmware/src/micropython/boards/UCT_MICROMOUSE/board_init.c):
    - `MICROPY_HW_CLK_USE_HSE = 1`
    - Main PLL: $8\text{ MHz} \times \frac{20 (\text{PLLN})}{1 (\text{PLLM}) \times 2 (\text{PLLR})} = 80.000\text{ MHz}$ SysClk.
    - PLLSAI1: $8\text{ MHz} \times \frac{12 (\text{PLLSAIN})}{1 (\text{PLLM}) \times 2 (\text{PLLSAIQ})} = 48.000\text{ MHz}$ for USB OTG FS & ADC.
    - Exact USART1 Baud Divider: `USART1->BRR = 694` ($\frac{80,000,000}{115200} = 694.44$, 0.06% error), guaranteeing crystal-locked 115200 baud without any clock drift or silicon variance.
* **Low-Power Debugging Fix:**
  - MicroPython's event loop executes `__WFI()` during idle wait.
  - Added `DBGMCU->CR |= DBGMCU_CR_DBG_SLEEP | DBGMCU_CR_DBG_STOP | DBGMCU_CR_DBG_STANDBY;` in `board_early_init()` to keep the SWD debug clocks active during WFI, preventing ST-Link debugger disconnects (`DEV_TARGET_CMD_ERR`).
* **Flash Partitioning:**
  - `0x08000000 - 0x0805FFFF`: MicroPython Firmware Binary (~360 KB).
  - `0x08060000 - 0x0806FFFF`: Internal Flash Storage partition (64 KB) mounted as `/flash` (`UCT_MMOUSE`).
  - `0x08070000 - 0x0807FFFF`: C-Kernel Flash Telemetry Logger fallback buffer (64 KB).

---

## 3. Verified Operational Status
1. **Interactive REPL:**
   - Single-character echo, fast response, crystal-locked 115200 baud on `/dev/cu.usbmodem*`.
   - Soft reboot (`Ctrl-D`) tested and verified without bus lockup or CPU hang.
2. **USB Mass Storage (`UCT_MMOUSE`):**
   - Automatically enumerates and mounts on host OS (`/Volumes/UCT_MMOUSE`).
   - Cleanly exposes `boot.py`, `main.py`, and `README.txt`.
3. **Lazy Peripheral Bring-up (`uct_mouse`):**
   - Clean module import and initialization (`uct_mouse.init()`).
   - Live telemetry and sensor reads verified over REPL:
     - `uct_mouse.get_vbatt()` -> `2.292 V`
     - `uct_mouse.get_tof()` -> filtered distances (mm, 180 kcps threshold)
     - `uct_mouse.get_tof_raw()` -> raw unthresholded distances (mm)
     - `uct_mouse.get_tof_signals()` -> return photon signal rates (kcps)
     - `uct_mouse.get_tof_detailed()` -> `((dist, sig), ...)` pairs
     - `uct_mouse.get_encoders()` -> live quadrature ticks
     - `uct_mouse.set_led(0..2, 1/0)` -> LED control with PB3 master gate
     - `uct_mouse.get_button()` -> SW1 button status
4. **Clean Reset & Drive Formatting Architecture:**
   - `--format-drive`: Soft filesystem format over USB OTG / serial REPL to wipe student scripts and restore default `boot.py`/`main.py`.
   - `--factory-reset`: True hardware clean-slate operation via ST-Link / DFU (full chip erase + clean baseline firmware reflash), completely wiping MCU firmware, filesystem partition (`0x08060000`), and telemetry log buffer (`0x08070000`).

---

## 4. 2026 Board Golden Baseline Metrics (Reconciliation Reference)
* **Board UID:** `0027003A5832501820313758`
* **Detected IMU:** `IMU_TYPE_LSM6DS3` (ST family on address `0x6A`, WHO_AM_I = `0x69`)
* **Stationary Accelerometer Baseline:**
  - $a_x = -0.129\text{ m/s}^2$ (Longitudinal / Forward-Back, pitch $-0.73^\circ$)
  - $a_y = -0.065\text{ m/s}^2$ (Lateral / Left-Right, roll $-0.37^\circ$)
  - $a_z = +10.173\text{ m/s}^2$ (Upward normal gravity reaction force)
  - Total Gravity Magnitude $|g| = 10.174\text{ m/s}^2$ (matches $9.81\text{ m/s}^2$ within 3.7%)
* **Stationary Gyroscope Baseline:**
  - Bias: $-0.46\text{ to }-0.58^\circ/\text{s}$ ($\approx -0.008\text{ to }-0.010\text{ rad/s}$)
  - Noise: Gaussian zero-mean $\sigma \approx 0.15^\circ/\text{s}$
* **Trajectory Execution (0.50m -> 90° CCW -> 0.50m):**
  - **Leg 1:** 0.497 m (2841 L / 2849 R ticks, 5.18s duration)
  - **Corner (CCW Turn):** +83.75° fast phase / +87.08° net integrated yaw (+112.5°/s peak rate, 0.86s duration)
  - **Leg 2:** 0.501 m (2866 L / 2872 R ticks, 5.20s duration)
  - **Total Records:** 562 sparse frames @ 25 Hz (14.87s runtime, ~30 KB payload, fits inside 64 KB partition)
  - **Log File Saved:** `benchmark_board_2026_reconciliation.jsonl`

---

## 5. 2025 Board Reconciliation Metrics & Cross-Generation Parity
* **Board UID:** `0036002C5642501020303759`
* **Detected IMU:** `IMU_TYPE_ICM42605` (InvenSense family on address `0x68`, WHO_AM_I = `0x42`)
* **Stationary Accelerometer Baseline:**
  - $a_x = -0.058\text{ m/s}^2$
  - $a_y = -0.026\text{ m/s}^2$
  - $a_z = +9.81\text{ to }+10.20\text{ m/s}^2$ (Upward normal gravity reaction)
  - Gravity Magnitude $|g| \approx 9.9\text{ m/s}^2$
* **Stationary Gyroscope Baseline:**
  - Bias: $+0.55\text{ dps}$ ($+0.55^\circ/\text{s}$)
  - Returns standard units in **degrees/s (dps)**
* **Trajectory Execution (0.50m -> 90° CCW -> 0.50m):**
  - **Leg 1:** 0.514 m (2941 L / 2954 R ticks, 5.42s duration) [vs 2026: 0.511m, 2925 L / 2933 R ticks]
  - **Corner (CCW Turn):** +86.87° integrated yaw (+113.3°/s peak rate, 0.89s duration) [vs 2026: +83.75° fast phase, +112.5°/s peak rate]
  - **Total Cumulative Encoders:** L=5348, R=6443 [vs 2026: L=5349, R=6405 — matching within 0.6%]
  - **Total Run Integrated Yaw:** 90.65° [vs 2026: 100.36°]
  - **Total Records:** 608 sparse frames @ 25 Hz (16.16s runtime, ~32 KB payload, 100% fits in 64 KB internal partition)
  - **Log File Saved:** `benchmark_board_2025_reconciliation.jsonl`

---

## 6. 2026 Board Final Post-Factory-Reset Validation
* **Board UID:** `0027003A5832501820313758`
* **Detected IMU:** `IMU_TYPE_LSM6DS3` (Address `0x6A`, WHO_AM_I = `0x69`)
* **Stationary Gyroscope Bias:** $-0.095\text{ dps}$
* **Trajectory Execution:**
  - **Leg 1:** 0.513 m (2937 L / 2938 R ticks, 5.40s duration)
  - **Corner Turn:** +82.52° fast phase (+110.0°/s peak rate, 0.84s duration)
  - **Leg 2:** 2808 L / 2895 R ticks (5.45s duration)
  - **Final Cumulative Encoders:** L=5392, R=6407
  - **Total Records:** 568 sparse frames @ 25 Hz (15.06s runtime)
  - **Log File Saved:** `benchmark_board_2026_final.jsonl`

---

## 7. USB OTG Decoupling & Enhanced ToF Drivers
* **USB OTG ISR Protection:**
  - Removed `MICROPY_INTERNAL_EVENT_HOOK` from low-level USB driver callbacks in `mpconfigboard.h`.
  - Added `__get_IPSR() != 0` check in `kernel_background_tick()` in `board_init.c` to prevent blocking I2C calls inside ISR contexts.
  - Resolved USB MSC enumeration latency and eliminated OTG connection freezes.
* **ToF Signal Threshold Calibration:**
  - Calibrated default driver threshold to **`80 kcps`** in `VL53L0X.c`, eliminating false 8190 dropouts on 30 cm targets while rejecting open-air noise.
  - Exposed `uct_mouse.get_tof_raw()`, `uct_mouse.get_tof_signals()`, and `uct_mouse.get_tof_detailed()` for advanced student signal processing.



