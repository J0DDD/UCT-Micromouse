# UCT Micromouse 2026 — Session Primer & Continuity Guide

> **Course:** University of Cape Town (UCT) Micromouse Design Project (EEE3097S / EEE3098S / EEE3099S)  
> **Academic Year:** 2026 Rollout  
> **Course Convenor:** Associate Professor Fred Nicolls  
> **Primer Last Updated:** 2026-10-18  
> **Master Handbook:** `docs/EEE3097_8_9S_Course_Handbook_2026.md` (Single source of truth for students)

---

## 1. Executive Status Overview

### A. Submission 3 (GA3 Design Report 2) — **COMPLETED & DEPLOYED**
* **Status:** 100% evaluated, calibrated, and synchronized live to Gradescope.
* **Gradescope Assignment:** Course ID `1337806` | Assignment ID `8652928` (`Submission 3: GA3 Design Report 2`).
* **Cohort Metrics:**
  * Active evaluated cohort: **139 students** (145 Gradescope submissions).
  * Mean: **14.18 / 20.0 (70.9%)** | Median: **14.00 / 20.0 (70.0%)** | Std Dev: **1.33 marks**.
  * Top 5 Range: **17.5 to 19.0 / 20.0 (87.5% to 95.0%)**.
  * Pass Rate ($\ge 50\%$): **99.3% (138 / 139)** | First-Class Rate ($\ge 75\%$): **23.7% (33 / 139)**.
* **Master Gradebook Single Source of Truth (SSOT):**
  * `workspace/sub3_marking/OFFICIAL_GRADES_MASTER.json` (45 audit transactions, 100% HMAC-SHA256 locked).
  * Frozen snapshot: `workspace/sub3_marking/OFFICIAL_GRADES_MASTER-snap18oct26-final.json`.
  * Offline CSV export: `workspace/sub3_marking/gradescope_final_upload.csv` (139 locked records exported).
* **Live Sync Status:** 139 Synced, 0 Drifted, 0 Missing.
* **Class Announcement:** Approved single-paragraph announcement drafted in `workspace/sub3_marking/SESSION_HANDOFF_AND_RESUMPTION_GUIDE.md`.
* **Appeals Protocol:** Formal regrade requests must be handled via `grade_guard.py` transaction commits; never edit `OFFICIAL_GRADES_MASTER.json` manually.

### B. Course Hardware & Firmware Platform — **STABLE & PROVEN**
* **Base Architecture:** Three-tier architecture (Tier 1 C-Kernel $\leftrightarrow$ Tier 2 Abstraction $\leftrightarrow$ Tier 3 Userland Python/Simulink).
* **Clock System:** 8.000 MHz external quartz resonator (HSE) on `PH0`/`PH1` generating 80.000 MHz SysClk and 48.000 MHz USB clock across all 2025/2026 boards.
* **Dual-IMU Support:** Dynamic runtime auto-detection of both newer ICM-42605 (`0x68`) and legacy LSM6DS3 (`0x6A`) on I2C2, normalizing gyroscope to $\text{rad/s}$ and accelerometer to $\text{m/s}^2$.
* **Display Compatibility:** Dynamic I2C address scanning for SSD1306 OLED displays (`0x78` or `0x7A`).
* **Encoders:** Delta encoding protocol (`+lenc`, `+renc`) and full sync dump (`"sync": 1`) over JSON register proxy.
* **Telemetry:** Automatic 25 Hz JSON logging to SPI flash (1 MB on 2026 boards) or internal flash (64 KB on 2025 boards), with cryptographic UID and code verification hash.
* **Flashing Tool:** Unified deployment script `python tools/deploy.py`.

### C. Simulation & Autograding Parity — **OPERATIONAL**
* **Autograder Socket Server:** TCP/IP Local Loopback (`localhost:8000`) streaming identical JSON frames as physical hardware.
* **Polymorphic Execution:** Student scripts run against physical hardware via C-Kernel bindings or desktop simulation via `uct_mouse` mock loopback without modifying a single line of student code.
* **Fast Sim Support:** Seamless switching between real-time paced simulation (100 Hz) and headless fast simulation (`fast_sim=True`) for autograding and RL.

---

## 2. Immediate Continuity: What to Do Next

When resuming in a new session:

1. **Submission 3 Maintenance & Regrade Requests:**
   * If students submit regrade appeals via Gradescope or email:
     1. Inspect the student folder: `workspace/sub3_marking/students/<student_dir>/`.
     2. Review `submission_summary.md`, `evaluation.json`, and raw submission.
     3. If a mark adjustment is warranted, commit via `grade_guard.py` with Convenor motivation.
     4. Push update to Gradescope using `python workspace/sub3_marking/tools/sync_sub3_gradescope.py push <SID>`.
     5. Regenerate offline CSV with `python workspace/sub3_marking/export_gradescope.py`.

2. **Milestone 1 & 2 Physical Maze Trials:**
   * Verify student submissions for Milestone 1 (Square Trajectory) and Milestone 2 (Maze Solving).
   * Benchmarking scripts and reference traces are located in root: `benchmark_board_2026_final.jsonl`, `maze_controller.jsonl`.
   * Reference solutions in `solutions/`: `milestone1_square.py`, `milestone2_maze.py`.

3. **Documentation Updates (if required):**
   * Edit `docs/EEE3097_8_9S_Course_Handbook_2026.md` directly.
   * Remember rule: Markdown lists must have a blank line before initiation.
   * Do NOT compile PDF/HTML locally; GitHub Pages syncs live automatically.

---

## 3. Strict Operating Invariants & Rules

1. **Python Interpreter Rule:**
   * **Always use:** `/opt/local/bin/python` (invoked simply as `python` in zsh, Python 3.13).
   * **Never use:** `/opt/homebrew/bin/python3` (Python 3.14 lacks numpy, scipy, pygame, pyserial).

2. **Never Propose `cd` Commands:**
   * Always execute shell tools using the explicit `Cwd` parameter.

3. **Master Ledger Protection:**
   * `workspace/sub3_marking/OFFICIAL_GRADES_MASTER.json` must remain pristine.
   * Only write to it via `grade_guard.py` transactions.
   * Current snapshot: `workspace/sub3_marking/OFFICIAL_GRADES_MASTER-snap18oct26-final.json`.

4. **Hardware Deployment:**
   * Always deploy firmware via `python tools/deploy.py`.

---

## 4. Key Repository Map

```
/Users/nicolls/proj/eee3097s/2026/UCT-Micromouse/
├── SESSION_PRIMER.md                             # <-- THIS FILE (Session continuity primer)
├── docs/
│   └── EEE3097_8_9S_Course_Handbook_2026.md      # Single master course handbook
├── tools/
│   └── deploy.py                                 # Central flashing tool (MicroPython/PikaScript/Simulink)
├── firmware/                                     # C-Kernel firmware source
├── python/
│   ├── uct_mouse.py                              # Desktop mock & hardware interface library
│   └── maze_sim.py                               # 2D maze physics simulator & autograder engine
├── solutions/
│   ├── milestone1_square.py                      # Reference Milestone 1 solution
│   └── milestone2_maze.py                        # Reference Milestone 2 solution
└── workspace/
    └── sub3_marking/                             # Submission 3 evaluation environment
        ├── OFFICIAL_GRADES_MASTER.json           # Master SSOT Ledger (139 locked records)
        ├── OFFICIAL_GRADES_MASTER-snap18oct26-final.json # Snapshot backup
        ├── gradescope_final_upload.csv           # Final offline Gradescope export
        ├── export_gradescope.py                  # Script to export locked grades to CSV
        ├── grade_guard.py                        # Cryptographic HMAC & transaction ledger
        ├── SESSION_HANDOFF_AND_RESUMPTION_GUIDE.md # Detailed marking handoff & rubrics
        ├── tools/
        │   ├── sync_sub3_gradescope.py           # Closed-loop Gradescope sync engine
        │   ├── gradescope_cookies.json           # Active session cookies
        │   └── gradescope_rubrics.json           # Catalog of rubric item IDs
        └── students/                             # 141 individual student dossiers & code
```

---

## 5. Ready-to-Use Command Cheatsheet

```zsh
# Verify live Gradescope sync status
python workspace/sub3_marking/tools/sync_sub3_gradescope.py status

# Regenerate official Gradescope export CSV
python workspace/sub3_marking/export_gradescope.py

# Query a specific student in master ledger
python -c "
import json
with open('workspace/sub3_marking/OFFICIAL_GRADES_MASTER.json') as f:
    master = json.load(f)
sid = '1967687'  # Katherine Vineall
print(json.dumps(master['students'][sid], indent=2))
"

# Flash MicroPython firmware to board via ST-Link
python tools/deploy.py --target micropython --flash

# Run desktop simulation test
python solutions/milestone1_square.py
```
