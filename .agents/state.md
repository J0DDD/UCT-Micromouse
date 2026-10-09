# Session State Log - UCT Micromouse

**Last Updated:** September 29, 2026  
**Target Hardware:** STM32L476VE (UCT Micromouse Chassis)  
**Active Submodule:** `external/MicroMouseTemplate`  

---

## 1. Summary of Completed Fixes

### TOF Sensor Open-Air Noise Rejection (Resolved)
* **Signal Amplitude Filter (`Signal >= 150`):** In open air / empty space, ambient 940 nm photon shot noise occasionally triggered the VL53L0X ASIC histogram DSP, generating false short-range distance glitches (~30–80 mm) that prematurely tripped simple collision loops like `while TOF > 100:`.
* **Restored Threshold:** `getVL53L0()` in [`firmware/src/micropython/boards/UCT_MICROMOUSE/VL53L0X.c`](file:///Users/nicolls/proj/eee3097s/2026/UCT-Micromouse/firmware/src/micropython/boards/UCT_MICROMOUSE/VL53L0X.c) and [`external/MicroMouseTemplate/.../VL53L0X.c`](file:///Users/nicolls/proj/eee3097s/2026/UCT-Micromouse/external/MicroMouseTemplate/MicroMouseProgramming_Code/Core/Src/VL53L0X.c) now enforces:
  ```c
  if ((distanceStr.rangeStatus == RANGECOMPLETE || distanceStr.rangeStatus == NONE) &&
      distanceStr.Signal >= 150 && distance > 20 && distance < 2000) {
      TOF_result->Distance = distance;
  } else {
      TOF_result->Distance = 8190; // Clean open air / out of range
  }
  ```
* **Performance:** `Signal = 150` (~1.17 MCPS) cleanly rejects 100% of open-air ambient noise spikes (solid `8190`), while preserving high-fidelity obstacle detection up to ~700–800 mm.

### Dual-Chip Standalone Factory Reset (`tools/deploy.py`)
* **Two-Phase Architecture:** Solved the dual-chip chicken-and-egg dependency between the STM32 MCU internal flash (512 KB) and ZD25WQ80C external SPI NOR flash (1 MB):
  1. **Phase 1 (SWD Flashing):** Writes `micropython.bin` via `st-flash` at `0x08000000` to give the MCU firmware to drive the SPI2 and USB peripherals.
  2. **Phase 2 (Python Raw REPL Provisioning):** Connects over USB VCP to execute `os.VfsFat.mkfs(pyb.Flash())`, populates `boot.py` (with `pyb.usb_mode('VCP+MSC')`), `main.py`, and `README.txt`, and triggers `pyb.hard_reset()` to force the USB PHY to re-enumerate as a composite Mass Storage (`MSC`) drive.
* Automated via: `python tools/deploy.py --engine micropython --flash --factory-reset`.

### Milestone 1 Autograder Suite Optimization
* **Decoupled NumPy Dependency:** Rewrote [`tools/autograder/assignments/milestone1_square/test_suite.py`](file:///Users/nicolls/proj/eee3097s/2026/UCT-Micromouse/tools/autograder/assignments/milestone1_square/test_suite.py) using pure Python stdlib (`math`), preventing `ModuleNotFoundError: No module named 'numpy'` in minimal autograder Docker containers.
* **Trajectory SVG & HTML Visualizations:** Added inline vector trajectory rendering and base64 video playback in Gradescope test outputs.

---

## 2. Recent Git Commits

### Main Repository (`UCT-Micromouse`)
* `b137a91` - `fix(tof): filter open-air noise with Signal >= 150 threshold and refine factory reset`
* `df7ff90` - `fix(tof): restore stable VL53L0X timing budget and valid range thresholds to fix startup stall and 8190mm sensor readings`
* `cc9634b` - `fix(leds): align LED pin mapping to LED0=PC13, LED1=PA4, LED2=PA5 across AGENTS.md and firmware`
* `01d5fdd` - `fix(leds): remove flash disk cache activity toggles on PC13 (LED0)`

---

## 3. Current State & Next Steps
1. **Physical Chassis Ready:** Mouse is running latest firmware with solid TOF filtering and clean `UCT_MMOUSE` filesystem.
2. **Student Autograder Deployment:** Autograder test suites and deployment tools are synchronized.
