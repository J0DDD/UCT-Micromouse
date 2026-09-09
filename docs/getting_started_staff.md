# Staff & TA Reference Guide

This guide is for convenors, tutors, and TAs managing the UCT Micromouse lab and autograder setup.

---

## 🛠️ Step 1: Lab Board Initialization & Factory Reset

When issuing a board to a student, or if a student corrupts their board's state, reset it to the clean, default configuration:

1. Connect the processor board to your computer via the ST-LINK programmer USB port.
2. Run the factory reset script from the repository root:
   ```bash
   python tools/factory_reset.py
   ```
3. This script will:
   * Wipe the external SPI flash chip.
   * Wipe the internal MCU flash.
   * Flash the latest MicroPython binary (`firmware/binaries/micropython.bin`).
   * Boot the board, auto-format the filesystem partition, and populate the default `boot.py`, `main.py`, and `README.txt` files (omitting legacy `pybcdc.inf`).

---

## 📐 Step 2: Running and Configuring the Gradescope Autograder

The autograder runs student scripts in head-to-head loopback simulation.

### Running Gradescope locally
To test the grading engine locally on your machine:
```bash
python tools/autograder/grade_runner.py
```
This runs the simulation tests headlessly and writes the GradeScope-compatible JSON output to `tools/autograder/results.json`.

### Re-generating Zip Bundles for Students
If you update tests or simulator configuration, rebuild the assignment autograder ZIP bundles:
* ZIP bundles for D2L/Gradescope uploads are located under `workspace/deploy/`.
* Students upload these files to Gradescope to verify their code.

---

## 🛠️ Step 3: Hardware Diagnostics & Debugging

If a student's board behaves erratically or fails to connect:

1. **JEDEC ID Scan:** If you suspect an SPI flash communication issue, run the diagnostic script:
   ```bash
   python tools/deploy.py --flash
   ```
2. **Rebuild Firmware From Source:**
   To rebuild the custom MicroPython interpreter target yourself:
   ```bash
   make -C external/micropython/ports/stm32 BOARD=UCT_MICROMOUSE
   ```
   Copy the newly compiled `firmware.bin` to `firmware/binaries/micropython.bin` to push to the repository.
