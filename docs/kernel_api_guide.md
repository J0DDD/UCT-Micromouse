# Micromouse Python API Reference Guide

This document defines the high-level Python API (`uct_mouse` module) used by students for both desktop simulation testing and physical STM32 hardware execution.

---

## 1. The `uct_mouse` Python Module

Students interact with the hardware and simulation environment strictly through the built-in `uct_mouse` library.

### API Methods Reference

| Method | Parameters | Return Value | Description |
|---|---|---|---|
| `init` | `fast_sim=None` *(bool)* | `int` | Initializes connection to either the virtual simulation testbed (PC) or the physical hardware (STM32). |
| `set_motors` | `left_pwm` *(int)*, `right_pwm` *(int)* | `None` | Sets raw motor speeds. Speeds range from `-100` (full reverse) to `100` (full forward). |
| `get_tof` | None | `(left, front_left, center, front_right, right)` *(tuple of ints)* | Returns current VL53L0X distance readings in millimeters (0–8190 mm) with standard noise filtering ($\ge 180\text{ kcps}$). `8190` represents out-of-range or invalid signal. |
| `get_tof_raw` | None | `(left, front_left, center, front_right, right)` *(tuple of ints)* | Returns raw, un-thresholded millimeter distance readings directly from the ASIC DSP registers. |
| `get_tof_signals` | None | `(left, front_left, center, front_right, right)` *(tuple of ints)* | Returns return photon signal rates in `kcps` (kilo-counts per second). Higher values indicate stronger reflection / closer proximity. |
| `get_tof_detailed` | None | `((l_dist, l_sig), (fl_dist, fl_sig), (c_dist, c_sig), (fr_dist, fr_sig), (r_dist, r_sig))` | Returns combined distance (mm) and return signal rate (kcps) pairs for all 5 active ToF sensors. |
| `get_encoders` | None | `(left, right)` *(tuple of ints)* | Returns total accumulated quadrature encoder ticks. |
| `get_gyro` | None | `float` | Returns current yaw gyro rate/angle (relative degrees/second rotation around Z-axis). |
| `get_vbatt` | None | `float` | Returns current battery supply voltage in Volts. |
| `delay_ms` | `ms` *(int)* | `None` | Delays execution. **CRITICAL:** On physical hardware, sensor/display updates are paced inside this call; control loops must call this to update values. |
| `set_polarity` | `left` *(int)*, `right` *(int)* | `None` | Normalizes physical motor wiring. Pass `1` (normal) or `-1` (reversed) to mathematically match your chassis. |
| `set_encoder_polarity` | `left` *(int)*, `right` *(int)* | `None` | Normalizes physical encoder direction. Pass `1` (normal) or `-1` (reversed) to mathematically match your chassis encoders. |
| `get_line_sensors`| None | `(fl, fr, sl, sr)` *(tuple of ints)* | Returns raw ADC readings for Front-Left, Front-Right, Side-Left, and Side-Right photodetector line sensors. |
| `get_telemetry` | None | `(ax, ay, az, gx, gy, gz, lenc, renc, current, battery_pct)` *(tuple)* | Returns full 6-DOF IMU data (ax/ay/az in m/s², gx/gy/gz in rad/s), encoders, battery current (mA), and battery life (%). |
| `get_ticks_ms` | None | `int` | Returns monotonic elapsed time in milliseconds. Commensurate between physical STM32 hardware (`HAL_GetTick()`) and simulation physics clocks. |
| `display_text` | `row` *(int, 1–4)*, `text` *(str)* | `None` | Writes custom text (up to 18 characters) to one of the 4 blue OLED rows. Passing an empty string `""` or `None` restores that row's default telemetry. |
| `clear_display` | None | `None` | Restores all 4 blue OLED rows back to default live telemetry streaming. |

---

## 2. Dynamic OLED Display Modes

The SSD1306 128x64 OLED display has 5 text rows (using standard 7x10 font):
* **Row 0 ($y=0$):** Reserved yellow header area, permanently displaying the platform header (`MicroPython`).
* **Row 1 ($y=16$):** Default `CMD: [left_pwm] [right_pwm]`
* **Row 2 ($y=28$):** Default TOF sensor telemetry (e.g. `TOF: [W] [N] [E]`)
* **Row 3 ($y=40$):** Default `BAT: [voltage]V [pct]% [current]mA`
* **Row 4 ($y=52$):** Default `WDG: [safety cutoff / ms]`

### Custom User Text & Independent Fallback

Students can override any of the 4 blue rows (1 to 4) independently using:

```python
import uct_mouse

uct_mouse.init()

# Display custom state on Row 1 and turn count on Row 4
uct_mouse.display_text(1, "State: EXPLORE")
uct_mouse.display_text(4, "Turns: 3")

# Rows 2 (TOF) and 3 (BAT) continue streaming live hardware telemetry!

# Restore Row 1 back to default motor telemetry:
uct_mouse.display_text(1, "")

# Or restore all rows back to default telemetry:
uct_mouse.clear_display()
```

### Dynamic TOF Telemetry Formatting (Row 2)

When Row 2 is in default telemetry mode, the C-Kernel automatically manages the TOF display configuration based on connected hardware:

* **3-Sensor Combination (N, W, E):** If only the Left, Centre, and Right TOF sensors are connected, the display shows:
  `W:[W_val] N:[N_val] E:[E_val]`
* **3-Sensor Combination (N, NW, NE):** If only the Front-Left, Centre, and Front-Right TOF sensors are connected, the display shows:
  `NW:[FL_val] N:[C_val] NE:[FR_val]`
* **All values are right-aligned to a 4-character fixed-width field** (`%4u`) to prevent horizontal layout shifting.

---

## 3. Time-of-Flight (ToF) Architecture & Sampling Dynamics

Understanding the dual-rate sampling architecture between the physical sensor silicon and userland Python is essential for designing wall-following and obstacle-filtering algorithms:

### A. Dual-Rate Architecture (Sensor Conversion vs Python Query)

1. **Hardware Measurement Timing Budget (~50 Hz / ~20 ms per sensor):**
   * Each STMicroelectronics VL53L0X Time-of-Flight sensor operates in hardware **Continuous Back-to-Back Ranging Mode** (`startContinuous(0)`).
   * In continuous back-to-back mode, the sensor ASIC starts a new laser measurement cycle immediately upon completing the previous one, operating at a physical conversion rate of **~50 Hz (a new completed measurement every ~15–20 ms)**.
   * During the physical photon emission and SPAD integration cycle, the sensor's hardware ranging interrupt flag (`RESULT_INTERRUPT_STATUS & 0x07`) remains unasserted.

2. **C-Kernel Polling & Shadow Register (100 Hz):**
   * The microcontroller's base C-Kernel runs an asynchronous background tick loop at **100 Hz (every 10 ms)**.
   * On every 10 ms tick, the kernel checks whether each sensor's hardware ranging flag has asserted.
   * When a physical measurement finishes (~every 1–2 ticks), the kernel reads the distance and photon signal strength via I2C and writes them to the internal shadow state structure (`current_state.tof_l`, etc.).
   * If a conversion is still in progress on that specific tick, the kernel retains the **latest valid cached measurement** in memory with zero blocking delay.

3. **Zero-Overhead Userland Access (`uct_mouse.get_tof()`):**
   * Calling `uct_mouse.get_tof()`, `get_tof_raw()`, `get_tof_signals()`, or `get_tof_detailed()` in Python reads directly from the C-Kernel shadow state in RAM (< 1 µs execution time).
   * In a 10 ms (100 Hz) control loop, you will receive approximately **50 genuine physical updates per second per sensor** (each reading holds for ~1 to 2 ticks).

### B. Recommendations for Student Signal Processing & Filtering

* **Expected Update Frequency:** Plan filters (Kalman, complementary, or low-pass) for an effective physical sample rate of **~50 Hz per sensor** (new data every ~20 ms).
* **Detecting Fresh Samples:** To run filter prediction/update steps strictly on fresh samples, compare incoming readings against the previous tick or monitor timestamp deltas with `uct_mouse.get_ticks_ms()`.
* **Out-of-Range & Noise Handling:** Open air or absorption surfaces return `8190` mm (the out-of-range sentinel). `uct_mouse.get_tof()` automatically suppresses ambient SPAD optical noise below 150 kcps; for raw, unthresholded data, use `uct_mouse.get_tof_raw()` and `uct_mouse.get_tof_signals()`.

