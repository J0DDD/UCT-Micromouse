# Milestone 1 (Square) Autograder Deployment Record & Restoration Guide

**Document Purpose:** This document records the complete specification, test parameters, scoring mathematics, and restoration procedure for the Milestone 1 (1m × 1m Square) Gradescope Autograder. All artifacts required to recreate, modify, or redeploy this exact autograder are archived in this directory.

---

## 1. Fast Restoration & Deployment Reference

* **Deployment ZIP Archive:** [`workspace/sub1_marking/autograder_deployment_backup/milestone1_square_autograder.zip`](file:///Users/nicolls/proj/eee3097s/2026/UCT-Micromouse/workspace/sub1_marking/autograder_deployment_backup/milestone1_square_autograder.zip) (also in `workspace/deploy/milestone1_square_autograder.zip`).
* **Gradescope Assignment Target:** Course `1337806`, Assignment `8312193` (Milestone 1: 1m × 1m Square).
* **Single Rebuild Command:** To recompile the autograder bundle from repository sources at any time:
  ```bash
  python tools/autograder/build_zip.py
  ```
* **Local Test Command (Single Student):**
  ```bash
  python tools/autograder/grade_runner.py --assignment milestone1_square --submission workspace/sub1_marking/submissions/<STUDENT_FOLDER>
  ```

---

## 2. Test Cases & Perturbation Matrix

The evaluation suite executes **6 simulation trials** across a 60-point integer scale (converted from a 100-point normalized test score):

| Test # | Test Name | Weight | Imbalance | Slip Coeff | Gyro Bias | Visibility | Purpose |
| :---: | :--- | :---: | :---: | :---: | :---: | :---: | :--- |
| **1** | **Public Baseline Run** | **25%** (15.0 pts) | $+4.8\%$ | $4.0\%$ | $-0.15^\circ/\text{s}$ | **Visible** | Benchmark run with deadband tolerances (same standard seen by students during submission). |
| **2** | **Hidden Positive Imbalance** | **15%** (9.0 pts) | $+12.0\%$ | $6.0\%$ | $-0.25^\circ/\text{s}$ | After Due Date | Tests closed-loop heading hold against strong right-wheel bias. |
| **3** | **Hidden Negative Imbalance** | **15%** (9.0 pts) | $-12.0\%$ | $6.0\%$ | $+0.25^\circ/\text{s}$ | After Due Date | Tests closed-loop heading hold against strong left-wheel bias. |
| **4** | **Hidden Traction Slip & Spin** | **15%** (9.0 pts) | $+6.0\%$ | $12.0\%$ | $-0.18^\circ/\text{s}$ | After Due Date | Tests distance odometry under low surface traction and wheel slip. |
| **5** | **Hidden Dynamic Turn Skid** | **15%** (9.0 pts) | $-12.0\%$ | $10.0\%$ | $+0.32^\circ/\text{s}$ | After Due Date | Tests gyroscope angle integration during rotational deceleration. |
| **6** | **Hidden Compound Perturbation**| **15%** (9.0 pts) | $+12.0\%$ | $12.0\%$ | $-0.35^\circ/\text{s}$ | After Due Date | Multi-axis stress test combining maximum imbalance and maximum slip. |

$$\text{Total Score} = \left( 0.25 \times S_1 + 0.15 \times \sum_{k=2}^6 S_k \right) \times \frac{60}{100}$$

---

## 3. Calibrated Scoring Rubric (100.00 Points / Trial)

Each test trial evaluates the recorded trajectory against 4 physical criteria:

| Evaluation Criterion | Score Allocation | Scoring Dynamics |
| :--- | :---: | :--- |
| **Leg Trajectory (Length & Straightness)** | **40.00 pts** | 4 legs × 10.0 pts each ($6.0\text{ pts}$ length error, $4.0\text{ pts}$ cross-track straightness). |
| **Corner Right-Angleness ($90^\circ$ Turns)** | **30.00 pts** | 4 corners × 7.50 pts each ($90.0^\circ$ angular change from gyroscope integration). |
| **Circuit Closure & Parking Accuracy** | **25.00 pts** | Euclidean return offset to origin $(0,0)$ upon completing the 4th side. |
| **Run Speed & Efficiency** | **5.00 pts** | Full marks for completing the 4m perimeter under $30.0\text{ s}$ ($0\text{ pts}$ at $\ge 50\text{ s}$). |
| **Empty Arena Collision Bonus** | **0.00 pts** | **Eliminated.** Removed unearned passive marks; transferred into Circuit Closure. |
| **Total Test Score** | **100.00 pts** | Integer percentage score for each trial. |

---

## 4. Key Algorithmic Guarantees

### A. Turn-Bounded Leg Segmentation
* Straight legs are segmented cleanly using detected corner turns ($\omega > 18.0^\circ/\text{s}$).
* **Startup Immunity:** Sensor calibration routines (e.g. 50-sample gyro drift loops) or initial ramp-ups before movement never create spurious $2\text{ cm}$ "mini-legs".
* **Oscillation Immunity:** Intermediate speed dips or control loop hesitations during a leg remain grouped within that leg, guaranteeing exact 4-leg alignment.

### B. Turn 4 & Forward Rollout Immunity
* **Zero Forward Motion Requirement:** Turn 4 is detected purely from rotational kinematics ($\omega > 18.0^\circ/\text{s}$) at the completion of Leg 4. Students are never required to nudge forward to trigger corner detection.
* **Rollout Protection:** The return offset is searched across all trajectory samples from $D_{\text{cum}} \ge 2.50\text{ m}$ through completion. Extra coasting or forward movement past the origin incurs **$0$ penalty** and does not distort the evaluated parking accuracy.

### C. DOMPurify-Safe Inline Vector Animation
* Video files and base64 GIFs are stripped by Gradescope's DOMPurify sanitizer.
* The autograder embeds a native SMIL-animated SVG vector model directly into the feedback report.
* Renders the authentic UCT Micromouse in **true 1:1 physical scale** ($105\text{ mm} \times 96\text{ mm}$, $65\text{ mm}$ wheels, dark green PCB, gold traces, OLED module, and 3x red ToF lasers) smoothly replaying the student's exact run.

---

## 5. Active Cohort Baseline Statistics

Gradescope cohort performance under this calibrated rubric:

* **Minimum:** $0.00\%$
* **Median:** $79.39\%$
* **Mean:** $76.46\%$
* **Maximum:** $96.58\%$
* **Standard Deviation:** $13.83\%$

---

## 6. Directory Backup Contents

The folder [`workspace/sub1_marking/autograder_deployment_backup/`](file:///Users/nicolls/proj/eee3097s/2026/UCT-Micromouse/workspace/sub1_marking/autograder_deployment_backup/) contains:

* `milestone1_square_autograder.zip`: Pre-built, ready-to-upload ZIP for Gradescope.
* `source/`: Unzipped, standalone source files matching the ZIP contents (`test_suite.py`, `grade_runner.py`, `physics_sim.py`, `uct_mouse.py`, `simulink_wrapper.c`, `setup.sh`, `run_autograder`).
