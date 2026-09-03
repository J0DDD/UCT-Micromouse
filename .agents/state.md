# Session State Log - UCT Micromouse

**Last Updated:** September 3, 2026  
**Target Hardware:** STM32L476VE (UCT Micromouse Chassis)  
**Active Submodule:** `external/MicroMouseTemplate`  

---

## 1. Summary of Completed Fixes

### User LED Mapping & Isolation (Resolved)
* **Pin Mapping Verified:** 
  * `LED0` = **`PC13`**
  * `LED1` = **`PA4`**
  * `LED2` = **`PA5`**
  * Master Gating Pin: **`PB3` (`CTRL_LEDS`)** (Active HIGH, must be driven `HIGH` to enable power to all LEDs).
* **Flash Driver Isolation:** Removed all debug/dirty-cache activity toggles on `PC13` from [`firmware/src/micropython/boards/UCT_MICROMOUSE/bdev.c`](file:///Users/nicolls/proj/eee3097s/2026/UCT-Micromouse/firmware/src/micropython/boards/UCT_MICROMOUSE/bdev.c). `LED0` is now 100% dedicated to userland application control (`uct_mouse.set_led(0, ...)`).
* **Aligned Across Codebase:** Synced in [`.agents/AGENTS.md`](file:///Users/nicolls/proj/eee3097s/2026/UCT-Micromouse/.agents/AGENTS.md), [`mpconfigboard.h`](file:///Users/nicolls/proj/eee3097s/2026/UCT-Micromouse/firmware/src/micropython/boards/UCT_MICROMOUSE/mpconfigboard.h), [`uct_mouse_mpy.c`](file:///Users/nicolls/proj/eee3097s/2026/UCT-Micromouse/firmware/src/micropython/boards/UCT_MICROMOUSE/uct_mouse_mpy.c), and [`board_init.c`](file:///Users/nicolls/proj/eee3097s/2026/UCT-Micromouse/firmware/src/micropython/boards/UCT_MICROMOUSE/board_init.c).

### TOF Sensor Pipeline & Low-Latency 50 Hz Operation (Resolved)
* **Unpopulated Sensor Skipping:** Added instant check `if (!TOF_result->initialized) { TOF_result->Distance = 8190; return; }` at top of `getVL53L0()`. Skips the 6 unconnected sensors (`FL`, `FR`, `MB_B`, `MB_F`, `MB_FL`, `MB_FR`), eliminating 30–60 ms of blocking I2C timeouts per tick.
* **Non-Blocking Range Checks:** `readRangeContinuousMillimeters()` checks `(readReg(RESULT_INTERRUPT_STATUS) & 0x07) == 0`. If a conversion is in progress, it returns `65535` immediately without busy-waiting; `getVL53L0()` retains the last valid sample without stalling the VM.
* **Correct Continuous Initialization Order:** In `initVL53L0()`, reordered sequence to:
  1. `setAddress_VL53L0X(tof->Address)` (assign unique address first)
  2. `setMeasurementTimingBudget(20000)` (20 ms / 50 Hz timing budget on new address)
  3. `startContinuous(0)` (start back-to-back continuous ranging on target address)
  *(Starting continuous ranging before changing the address previously stalled the sensor timing engine).*
* **Open-Air Noise Rejection:** Ambient SPAD counts in open air produce false ~30 mm distance readings with low photon signal amplitude (`Signal < 100` / 0.78 MCPS). Distance is filtered by:
  ```c
  if (distanceStr.Signal >= 100 && distance > 20 && distance < 2000) {
      TOF_result->Distance = distance;
  } else {
      TOF_result->Distance = 8190; // Clean open air / out of range
  }
  ```

### I2C Bus & OLED Display Stability (Resolved)
* **Eliminated Reset Loop in `micromouse_kernel.c`:** Transient NACK error codes from sensor reads previously triggered `restartI2C(&hi2c2)` and `SSD1306_Init()` every 100 ms display update, causing OLED flickering and continuous TOF resets.
* **Safe Recovery:** Hardware bus reset is now only triggered if `hi2c2.State == HAL_I2C_STATE_BUSY` for >50 consecutive ticks (>500 ms continuous hang). Transient error codes in `READY` state are safely cleared.

---

## 2. Recent Git Commits

### Main Repository (`UCT-Micromouse`)
* `cc9634b` - `fix(leds): align LED pin mapping to LED0=PC13, LED1=PA4, LED2=PA5 across AGENTS.md and firmware`
* `01d5fdd` - `fix(leds): remove flash disk cache activity toggles on PC13 (LED0)`
* `d1ca334` - `fix(tof): eliminate latency and reject open-air false short distances`
* `590a52b` - `fix(tof): resolve I2C reset loop on OLED and filter open air via photon return rate`
* `f5cfe0e` - `fix(tof): correct continuous mode initialization order`

### Submodule (`MicroMouseTemplate`)
* `faf024e` - `perf(tof): skip uninitialized sensors, make continuous reads non-blocking, and enable 50Hz timing budget`
* `bdafa57` - `fix(tof): use direct 12-byte burst read and enforce rangeStatus==0 to reject open-air noise`
* `3243ec0` - `fix(tof): filter open-air noise via return signal amplitude (Signal >= 100)`
* `25ee3a2` - `fix(tof): set address before continuous mode to prevent startup timing stall`

---

## 3. Current State & Next Steps for Next Session
1. **Milestone 0 Physical Verification:**
   * Run [`python/tests/milestone0_wall_follow.py`](file:///Users/nicolls/proj/eee3097s/2026/UCT-Micromouse/python/tests/milestone0_wall_follow.py) on the physical mouse.
   * Verify LED thresholds (<200 mm) for Left (LED0), Center (LED1), and Right (LED2).
   * Press SW1 (User Button) to test closed-loop wall following and front collision cutoff (<150 mm).
2. **PID & Velocity Tuning:** Fine-tune side error proportional gain (`corr = error * 0.4`) and baseline motor PWMs (`85`) in `milestone0_wall_follow.py` based on physical track behavior.
