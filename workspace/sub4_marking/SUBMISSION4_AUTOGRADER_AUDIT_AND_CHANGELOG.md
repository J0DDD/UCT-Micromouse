# Submission 4 (Final Maze Solver) Autograder Audit, Specification & Changelog

**Document Purpose:** This document serves as the master record, mathematical specification, visual animation guide, and persistent repair changelog for the Submission 4 (Milestone 2 / Final Maze Solver) Gradescope Autograder. All standalone artifacts and deployment packages required to recreate, restore, or modify this exact grading engine are preserved directly in `workspace/sub4_marking/`.

---

## 1. Quick Restoration & Deployment Reference

* **Gradescope Deployment Archive:** [`workspace/sub4_marking/autograder_deployment_backup/milestone2_maze_autograder.zip`](file:///Users/nicolls/proj/eee3097s/2026/UCT-Micromouse/workspace/sub4_marking/autograder_deployment_backup/milestone2_maze_autograder.zip) (synced with `workspace/deploy/milestone2_maze_autograder.zip`).
* **Standalone Unpacked Source:** [`workspace/sub4_marking/autograder_deployment_backup/source/`](file:///Users/nicolls/proj/eee3097s/2026/UCT-Micromouse/workspace/sub4_marking/autograder_deployment_backup/source/).
* **Single Rebuild Command:** To recompile the autograder ZIP bundles from repository source files at any time:

  ```bash
  python tools/autograder/build_zip.py
  ```

* **Local Verification Command (Single Student):**

  ```bash
  python tools/autograder/grade_runner.py --assignment milestone2_maze --submission workspace/sub4_marking/submissions/<STUDENT_FOLDER>
  ```

---

## 2. Test Cases & Multi-Pillar Stress Matrix

The evaluation suite executes **5 simulation trials** across a 60-point scale (converted from a 100-point normalized test score), evaluating algorithmic depth, sensor fusion, motion profiling, and arena scalability:

| Test # | Test Name | Arena | Weight | Seed | Imbalance | Traction Slip | Visibility | Educational & Engineering Objective |
| :---: | :--- | :---: | :---: | :---: | :---: | :---: | :---: | :--- |
| **1** | **Baseline Maze Navigation** | $4 \times 6$ | **20%** (12.0 pts) | 42 | $+4.0\%$ | $2.0\%$ | **Visible** | Standard benchmark maze topology; verifies fundamental 4-phase mission execution and pirouette. |
| **2** | **Asymmetric Corridors & Motor Bias** | $4 \times 6$ | **20%** (12.0 pts) | 43 | $+6.0\%$ | $2.0\%$ | **Visible** | Evaluates single-wall referencing and IMU yaw hold when parallel side walls are absent. |
| **3** | **Complex Non-Convex Alternate Path** | $4 \times 6$ | **20%** (12.0 pts) | 44 | $-6.0\%$ | $2.0\%$ | **Visible** | Tests global shortest-path calculation (Floodfill/A*) vs naive exploratory path replay. |
| **4** | **Dynamic Traction Slip & Corridor Merging** | $4 \times 6$ | **20%** (12.0 pts) | 45 | $+4.0\%$ | $4.0\%$ | **Visible** | Rewards straight-corridor segment merging and elevated acceleration profiles ($R \ge 1.35\times$). |
| **5** | **Championship Scalability Arena** | $8 \times 8$ | **20%** (12.0 pts) | 48 | $+5.0\%$ | $3.0\%$ | **Visible** | Evaluates scalable graph search and dynamic dimension querying (`uct_mouse.get_maze_dimensions()`). |

$$\text{Final Total Score} = \left( 0.20 \times \sum_{k=1}^5 S_k \right) \times \frac{60}{100}$$

---

## 3. Calibrated 4-Phase Scoring Rubric (100.00 Points / Trial)

Each simulation trial evaluates the robot's physical trajectory and state machine across 4 distinct operational phases:

| Operational Phase | Score | Evaluation Criteria & Kinematic Physics |
| :--- | :---: | :--- |
| **Phase 1: Target Room Discovery** | **30.0 pts** | Awarded when the robot successfully enters the $2 \times 2$ target room ($R \le 0.28\text{ m}$ of target center $(c_x \times 0.2\text{m}, c_y \times 0.2\text{m})$). Partial credit scaled linearly on closest Euclidean approach if target is not reached. |
| **Phase 2: Target Recognition Pirouette** | **20.0 pts** | Robot must perform an on-the-spot $360^\circ$ yaw spin ($\ge 5.5\text{ rad}$) while remaining strictly inside the target room with linear displacement $\le 0.12\text{ m}$. |
| **Phase 3: Autonomous Return to Start** | **20.0 pts** | After discovering the target room, the robot must autonomously navigate back to the start cell $(0,0)$ ($d_{\text{start}} \le 0.15\text{ m}$). |
| **Phase 4: High-Speed Sprint** | **20.0 pts** | Following its return to start, the robot must sprint back to the target room using its mapped optimal path. |
| **Speed & Dynamic Optimization Bonus** | **10.0 pts** | Evaluates genuine corridor merging and trajectory optimization:<br>• **Absolute Sprint Velocity (5.0 pts):** Full marks for average sprint speed $\bar{v}_{\text{sprint}} \ge 0.30\text{ m/s}$ ($0\text{ pts}$ at $\le 0.15\text{ m/s}$).<br>• **Sprint Speedup Factor (5.0 pts):** Full marks for achieving $\ge 1.35\times$ speedup during sprint compared to exploration ($0\text{ pts}$ for $\le 1.05\times$). |
| **Timeout Penalty** | **-10.0 pts** | Applied if the run exceeds the time limit ($120.0\text{ s}$ for $4 \times 6$; $240.0\text{ s}$ for $8 \times 8$). |
| **Total Test Trial Score** | **100.0 pts** | Clamped to $[0, 100]\%$ and normalized to 60.0 course points. |

---

## 4. SVG Vector Trajectory Animation Engine

The autograder generates native inline SMIL-animated SVG vector replay graphics embedded directly into Gradescope student feedback:

* **Dynamic Multi-Scale Geometry & Solid Walls:**
  * Automatically scales viewBox and coordinate systems for both $4 \times 6$ ($0.8\text{m} \times 1.2\text{m}$) and $8 \times 8$ ($1.6\text{m} \times 1.6\text{m}$) arenas.
  * Solid white/light-gray barrier planks (`#e2e8f0`, width `3.5px`, rounded caps) rendering all outer perimeter and internal DFS walls.
  * Circular lattice junction posts (`#cbd5e1`) at every cell intersection.
  * Faint dashed grid lines (`#232d3d`) highlighting open navigable corridors.
* **Zone Indicators:**
  * **Start Cell:** Cyan highlighted cell at $(0,0)$.
  * **Target Room:** Glowing emerald green $2 \times 2$ central plaza.
* **Trajectory Trail:** Golden navigation path trace (`#ffd166`).
* **Vehicle Scale & Detail:** Authentic **1:1 physical scale** representation of the UCT Micromouse ($105\text{ mm} \times 96\text{ mm}$):
  * Charcoal-black chassis base with dark green solder-mask PCB.
  * Gold power/signal traces and silkscreen labels.
  * $65\text{ mm}$ polyurethane wheels with yellow reduction gearboxes.
  * STM32 central microcontroller and bright cyan $0.96"$ OLED display.
  * 3x forward/side infrared ToF sensor emitter lenses with red active indicator dots.
* **Kinematic Heading & Physical Time Fidelity (`keyTimes`):**
  * Trajectory samples record $(x, y, \theta, t_{\text{sim}})$.
  * SVG dual transforms (`<animateTransform type="translate">` and `<animateTransform type="rotate">`) bind directly to continuous unwrapped orientation angles $\theta(t)$ and normalized timestamps $t_k / T_{\text{total}}$.
  * Accurately reproduces fast sprints, deceleration into turns, in-place $360^\circ$ victory pirouettes, and stationary pauses at $(0,0)$.

---

## 4.1 Polymorphic Execution of `get_maze_dimensions()` & `set_maze_dimensions()`

The maze dimension API operates polymorphically across all three execution targets:

1. **Physical Silicon Hardware (STM32 / MicroPython / PikaScript):**
   * Default returns `(4, 6)` as a native 2-tuple `(rows, cols)` (representing the standard $4 \times 6$ lab wooden arena).
   * For the physical Championship Tournament, calling `uct_mouse.set_maze_dimensions(8, 8)` directly updates static internal C variables in the MicroPython C module (`uct_mouse_mpy.c`) and PikaScript interface, causing `get_maze_dimensions()` to return `(8, 8)` with zero overhead.
   * Coordinate-agnostic exploration algorithms (e.g. `self.walls = {}` dictionary of `(x,y)` coordinates) function identically on $4 \times 6$, $8 \times 8$, or $16 \times 16$ arenas without hardcoded limits.
2. **Desktop PC Simulation (Student Laptop):**
   * Automatically queries the connected simulator process over the JSON loopback stream (`{"rows": R, "cols": C}`).
   * Reflects the configured geometry in `simulation_config.json` (or defaults to `(4, 6)`).
3. **Gradescope Autograder Container:**
   * In Tests 1–4, dynamically delivers `(4, 6)`.
   * In Test 5 (Championship Scalability Arena), dynamically delivers `(8, 8)` upon socket handshake, allowing modular student solvers to allocate or scale their exploration graphs without recompilation.

---

## 5. Audit Log & Student Complaint / Revision Changelog

Use this section to record all modifications, student dispute reviews, rubric calibrations, or parameter fixes. This provides an audit trail for regrading without requiring git excavation:

| Date | Affected Student / Cohort | Issue Reported / Observed | Root Cause Analysis | Remediation / Fix Applied | Re-graded Score Impact |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **2026-10-09** | **Initial Release** | Deployment initialization for Submission 4 / Milestone 2. | Standardized grading baseline established with SVG vector playback and 4-phase evaluation. | Deployment bundle compiled and archived in `workspace/sub4_marking/autograder_deployment_backup/`. | Baseline established. |
| **2026-10-09** | **Cohort-Wide** | Internal maze walls missing from SVG visual animation. | `generate_trajectory_svg()` only drew outer boundaries and dashed grid lines; wall segments were omitted. | Updated `physics_sim.py` to record `"walls"` line segments in `trajectory.json`; upgraded `grade_runner.py` to render solid planks (`#e2e8f0`) and corner pegs (`#cbd5e1`). | Pure visual enhancement ($0$ point change). |
| **2026-10-09** | **Cohort-Wide** | $2 \times 2$ target room location was fixed at $(4, 2)$ across all seeds. | `generate_random_maze` hardcoded `(4, 2)` instead of selecting candidate coordinates. | Implemented seed-based candidate selection across valid $2 \times 2$ bounds (excluding $(0,0)$ start). | Enforces generalized exploration across different target positions. |
| **2026-10-09** | **Thabelo Tshiuda (1961628)** | $360^\circ$ pirouette not rotating visually in SVG replay; Phase 4 sprint cut off after 3s pause. | 1. `animateMotion rotate="auto"` only rotates along $(x,y)$ path tangents (which are undefined/zero during an in-place spin).<br>2. Simulator auto-stop watchdog triggered at $\ge 3.0\text{ s}$ stationary, cutting off the 3s pause before Phase 4. | 1. Upgraded SVG animator to dual `<animateTransform>` tracking true unwrapped yaw $\theta(t)$.<br>2. Increased stationary auto-stop watchdog from $3.0\text{ s} \rightarrow 6.0\text{ s}$ in `physics_sim.py`. | Enables full Phase 4 sprint execution for students adhering to the 3.0s pause. |
| **2026-10-09** | **Cohort-Wide** | Static total time ($\le 35\text{s}$) unfair to seeds with distant target rooms; did not enforce sprint trajectory optimization. | Static metric rewarded close target room luck over true motion control; single-cell crawling earned unmerited bonus. | Replaced static total time with **Dual-Factor Speed Bonus**: 5 pts for absolute velocity ($\bar{v}_{\text{sprint}} \ge 0.30\text{ m/s}$) + 5 pts for speedup ratio ($R = \bar{v}_{\text{sprint}} / \bar{v}_{\text{explore}} \ge 1.35\times$). | Enforces straight corridor merging and elevated velocity profiles. |
| **2026-10-09** | **Cohort-Wide** | Need for post-submission excellence stress testing to establish a fair mark spread against AI prompt-hacks. | Standard 4x6 runs alone do not test algorithmic graph scalability or single-wall referencing in open corridors. | Upgraded to 5-pillar test suite including Test 5 ($8 \times 8$ Championship Arena); introduced `uct_mouse.get_maze_dimensions()` API. | Establishes a natural, bell-curved grade distribution rewarding modular engineering. |

---

## 6. Directory Structure & File Map

```text
workspace/sub4_marking/
├── autograder_deployment_backup/
│   ├── milestone2_maze_autograder.zip   # Ready-to-upload Gradescope ZIP
│   └── source/                          # Standalone uncompressed autograder source tree
│       ├── setup.sh                     # Ubuntu Gradescope container setup
│       ├── run_autograder               # Execution wrapper
│       ├── grade_runner.py              # Test harness, rubric & SVG animator
│       ├── physics_sim.py               # 2D kinematic, sensor & wall simulation
│       ├── uct_mouse.py                 # PC mock interface
│       ├── micromouse.py                # Network protocol client
│       ├── simulink_wrapper.c           # Simulink C bridge
│       └── assignments/
│           └── milestone2_maze/
│               └── test_suite.py        # Submission 4 test suite & rubric
├── submissions/                         # Downloaded student submissions
│   └── 1961628_Thabelo_Tshiuda/         # Sample submission files & Gradescope reports
└── SUBMISSION4_AUTOGRADER_AUDIT_AND_CHANGELOG.md  # This document
```
