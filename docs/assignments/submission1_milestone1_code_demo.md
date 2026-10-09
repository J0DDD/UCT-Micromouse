# EEE3097/8/9S Micromouse 2026: Submission 1
## Milestone 1 Code & Demo (25% of Course Mark)

---

### 1. Objective
Design and implement a closed-loop controller that guides your Micromouse robot to traverse a perfect **1.0m x 1.0m square** on the floor and stop autonomously. Your design must use closed-loop feedback (such as gyroscope yaw heading alignment and quadrature encoder distances) to remain robust against traction loss, slip, and motor asymmetries.

#### **Specific Learning Objectives:**
* Configure and read quadrature wheel encoders to track linear distance.
* Interface with the MPU6050 IMU gyroscope and integrate angular rate to track heading.
* Formulate a Proportional (P) or PID controller to synchronize wheel velocities and steer the mouse differentially.
* Implement a finite state machine (FSM) to transition between straight trajectories and spin-in-place turns.
* Design safety guards to handle motor saturation and battery voltage sag.

---

### 2. Step-by-Step Implementation Guide

#### **Step 1: Sensor Verification & Calibration**
* Place the mouse on a flat surface. Write a test script to read raw encoder ticks and gyro yaw rate.
* Verify that pushing the mouse forward increases both encoder counts symmetrically. 
* Rotate the mouse 360° manually. Verify that your gyro integration math returns exactly 360° (or $2\pi$ radians). Adjust your gyroscope scale factor parameters if you see scaling errors.

#### **Step 2: Dual-Motor Speed Synchronization**
* Implement a controller that adjusts left and right motor PWM signals dynamically so both wheels rotate at the same linear velocity. 
* Test this by running the mouse straight on the floor. If it veers to one side, tune your feedback gains to minimize the drift.

#### **Step 3: Closed-Loop Heading Correction (Steering)**
* Add your integrated gyroscope yaw angle to your straight-run feedback loop.
* Set your target yaw heading to 0°. If the mouse drifts or is physically bumped, your controller must adjust the wheel speeds differentially (steering) to return to 0°.

#### **Step 4: Spin-in-Place Turning**
* Implement a precise 90° left (anticlockwise) turn state.
* The mouse should spin in place (left wheel reverse, right wheel forward) until the integrated gyroscope yaw angle matches exactly +90° relative to the heading before the turn.
* Ensure your turn controller includes a settling window to prevent overshoot and oscillations.

#### **Step 5: Finite State Machine Integration**
* Combine these states into a sequential state machine:
  $$\text{DRIVE\_FORWARD\_1 (1.0m)} \rightarrow \text{TURN\_LEFT\_1 (90}^\circ\text{)} \rightarrow \dots \rightarrow \text{TURN\_LEFT\_4 (90}^\circ\text{)} \rightarrow \text{STOP}$$
* The mouse must execute **4 forward drives and 4 turns**, coming to a complete, autonomous stop at the end of the 4th turn. This ensures the robot ends its run at the **exact same position and orientation** ($0^\circ$) as it started.
* Once the square is completed, the mouse must remain completely stopped for **at least 3 seconds** to clearly indicate the completion of the task on video and in the telemetry log.

---

### 3. Deliverables (Gradescope Submission)
To make submission simple and prevent errors, a packaging tool is provided. Run the following command from your repository root:
```bash
python tools/package_submission.py --task milestone1 --src workspace/task1_square/
```
This script will perform dynamic diagnostic checks:
* **Local Syntax Guard:** The script runs a local syntax compile check. If your Python code has indentation or formatting errors, the packager will abort and pinpoint the error line so you can fix it before zipping.
* **Auto-Generated Package:** It automatically packages all code, subdirectories, models, and telemetry logs into a single **`submission_milestone1.zip`** in your project root.

Upload this ZIP file AND your **`run_video.mp4`** separately to Gradescope (Gradescope allows you to drag-and-drop both files into the submission portal together).

The submission consists of:
1. **Your ZIP Package (`submission_milestone1.zip`):**
   * **Your Controller Code:** Automatically compiled and zipped from your workspace directory (includes `main.py` and any subfolders/libraries recursively).
   * **Physical Telemetry Log (`run_log.jsonl`):** Automatically detected by the packager tool from your project directory (no need to copy it manually).
2. **Your Physical Run Video (`run_video.mp4`):**
   * Uploaded as a **separate file** alongside your ZIP.

#### **Testing the Autograder & Robustness Stress Tests Offline (Locally)**
You are highly encouraged to test your algorithm against the grading suite and physical perturbation stress tests locally on your laptop before uploading to Gradescope.

1. **Multi-Track Robustness Suite (Recommended):**
   Evaluate your controller across 6 physical tracks with motor gain imbalances ($\pm 12\%$) and surface slip ($10\%$):
   ```bash
   python tools/test_robustness.py workspace/task1_square/main.py
   ```
   To test with completely randomized perturbation ranges:
   ```bash
   python tools/test_robustness.py workspace/task1_square/main.py --randomize
   ```

2. **Official Autograder Replicator:**
   To run the full multi-test evaluation suite locally, run this command from the repository root:
   ```bash
   python tools/autograder/grade_runner.py
   ```
   This script runs the local simulator backend, automatically detects and executes your code from **`workspace/task1_square/`**, runs it through all 3 test scenarios (including the hidden runs), and outputs the resulting score sheet directly to your terminal.
   
   *Note: If you want to run the autograder on a different folder, you can override the source folder using the `--submission` flag:*
   ```bash
   python tools/autograder/grade_runner.py --submission path/to/your/folder
   ```

---

### 4. Video Requirements & Academic Honesty Declaration
To verify that your physical run is authentic and represents your own work, the video must strictly adhere to the following sequence:
1. **Student Card Close-up:** The video **MUST start with a clear, readable close-up of your physical Student Card** for at least 3 seconds. In doing so, you declare that the submission represents your own work.
2. **Setup:** Pan the camera to show the mouse positioned at the starting corner of the 1.0m x 1.0m grid.
3. **Traversal:** Capture the complete run without cuts. The mouse must drive 1.0m straight, turn 90° left (anticlockwise), and repeat this 4 times to form a square, coming to an autonomous stop.

---

### 5. Code & Log Correlation Verification (Anti-Cheat Check)
The autograder uses two fields in your log's header line to verify authenticity:
```json
{"log_header":1,"uid":"066AFF514885864967083830","hash":4017325881}
```
* **Hardware ID Check:** The `"uid"` field represents your microcontroller's unique device ID. While this is not registered in advance, the course convenors check the submitted logs for duplicate UIDs. Submitting logs with identical UIDs under different student accounts indicates shared files/hardware runs and will trigger a plagiarism audit.
* **Code Match Check:** The `"hash"` field is a 32-bit FNV-1a checksum computed in hardware based on the code loaded into the mouse. The autograder will compile your submitted script/binary locally and verify that the resulting checksum matches the hash inside the log. **Mismatched hashes will result in an immediate submission rejection.**

---

### 6. Evaluation Criteria & Grading Rubric
Your Gradescope submission is evaluated across three parts:

#### Part A: Autograded Co-Simulation Trajectory (60% of Milestone Mark)
The autograder executes your controller script/binary in the virtual simulation testbed across **three separate runs** featuring static perturbations (motor imbalances, slip coefficients, and IMU calibration offsets) alongside dynamic transient noise (starting wheel slips and turn deceleration overshoots). 

The runs are configured as follows:
1. **Test 1: Public Baseline Run (40% Weight - Score visible immediately)**
   * *Perturbations:* Moderate motor imbalance (`0.05`) and moderate running slip (`0.04`).
   * *Purpose:* Verifies basic controller trajectory execution and provides quick feedback.
   * *Local Replication Command:*
     ```bash
     python tools/physics_sim.py --imbalance 0.05 --slip 0.04 --seed 42
     ```
2. **Test 2: Hidden Asymmetry Stress-Test (30% Weight - Score hidden until after due date)**
   * *Perturbations:* Heavy motor imbalance (`0.12`), representing a chassis where one motor has significantly lower gain/torque.
   * *Purpose:* Verifies that your controller actively corrects steer-heading using closed-loop gyro feedback, rather than relying on hardcoded motor matching constants.
   * *Local Replication Command:*
     ```bash
     python tools/physics_sim.py --imbalance 0.12 --slip 0.02 --seed 43
     ```
3. **Test 3: Hidden Starting/Turning Slip Run (30% Weight - Score hidden until after due date)**
   * *Perturbations:* High slip coefficient (`0.10`) combined with active starting wheel-spin slip and turn deceleration settling slip.
   * *Purpose:* Verifies that your FSM transitions and steering controller do not drift or miss turn target angles due to transient traction losses.
   * *Local Replication Command:*
     ```bash
     python tools/physics_sim.py --imbalance 0.04 --slip 0.10 --seed 44
     ```

Each simulation run is scored out of **100 points** using the following breakdown:
* **Leg Segments (30 points max):** 7.5 pts per leg (East, North, West, South). Split equally between length accuracy ($\pm 5\text{ cm}$ tolerance) and straightness ($2\text{ cm}$ lateral deviation margin).
* **Corner Turn Angles (30 points max):** 7.5 pts per corner. Evaluates orientation change accuracy relative to $90.0^\circ$ ($\pm 3^\circ$ tolerance).
* **Parking Accuracy (20 points max):** Distance between final resting spot and start point. Full points if $\le 3\text{ cm}$, scaling to 0 at $\ge 25\text{ cm}$.
* **Run Speed Efficiency (10 points max):** Full points if time $\le 20\text{s}$, scaling to 0 at $\ge 40\text{s}$.
* **Safety Bonus (10 points):** Flat bonus if the run completes without colliding with virtual walls.
* **Applied Penalties:**
  * **Timeout ($-10$ points):** Applied if the controller fails to stop within the 45-second limit.
  * *Collision Note:* Contacting a wall halts the simulation immediately, naturally capping your score based only on the corners completed prior to the crash. No additional numerical collision penalties are subtracted.

The final Part A score is the weighted average of these three runs.

#### Part B: Physical Run Verification (30% of Milestone Mark)
Tutors will evaluate your submitted physical demonstration video (`run_video.mp4`) and verify hardware control performance. Marks are awarded for smooth acceleration/deceleration transitions, correct turn alignments, and successful autonomous execution on real floor surfaces without manual intervention or drifting out-of-bounds.

#### Part C: Submission Compliance (10% of Milestone Mark)
Evaluated by tutors on instruction compliance:
* **All Files Included (5%):** Correct zipping of source code workspace and valid FNV-1a checksum matched physical telemetry log file (`run_log.jsonl`).
* **Student Card Close-up (5%):** The physical demo video begins with a clear, readable 3-second close-up of your Student Card.

---

> [!NOTE]
> **Grading Adaptation Policy:** The grading thresholds, coefficients, and parameters detailed above serve as baseline targets. Course staff reserve the right to adjust or tailor specific parameters post-submission to ensure final grades are highly representative of design performance and ECSA attribute tracking.
