import json
import os

def evaluate_log(log_path):
    feedback = []
    feedback.append("=== Milestone 0 Hardware Verification Results ===")

    if not log_path or not os.path.exists(log_path):
        return 0.0, "Submission Error: Could not find 'run_log.jsonl' in your submission folder. Please ensure you extracted the log from your physical mouse using 'python tools/dump_logs.py' and uploaded it directly."

    try:
        with open(log_path, "r") as f:
            lines = f.readlines()
    except Exception as e:
        return 0.0, f"Submission Error: Failed to read 'run_log.jsonl': {e}"

    if not lines:
        return 0.0, "Submission Error: The uploaded 'run_log.jsonl' is completely empty."

    # Parse header (first line)
    try:
        header = json.loads(lines[0].strip())
    except Exception as e:
        return 0.0, f"Verification Error: Failed to parse header (Line 1) as JSON: {e}"

    if "log_header" not in header or "uid" not in header:
        return 0.0, "Verification Error: Log header (Line 1) is missing required verification fields ('log_header' and 'uid'). Make sure you upload an authentic run log generated directly by the mouse."

    uid = header.get("uid", "UNKNOWN")
    code_hash = header.get("hash", 0)
    feedback.append(f"[Registration] Device UID  : {uid}")
    feedback.append(f"[Registration] Code Hash   : {code_hash}")

    # Check for active readings in telemetry records
    records_count = 0
    motor_actuated = False
    encoders_changed = False
    tof_updated = False
    gyro_active = False

    for i in range(1, len(lines)):
        line_clean = lines[i].strip()
        if not line_clean:
            continue
        try:
            record = json.loads(line_clean)
        except Exception:
            continue # skip corrupt lines

        records_count += 1
        
        # Check motor commands
        if "l" in record or "r" in record:
            motor_actuated = True

        # Track encoder change
        if "le" in record or "re" in record:
            encoders_changed = True
            
        # Track ToF center readings
        if "tc" in record:
            tof_updated = True

        # Track Gyro readings
        if "g" in record:
            gyro_active = True

    feedback.append(f"Parsed {records_count} active sparse telemetry ticks.")

    score = 0.0
    checklist = []
    
    # Validation checks
    if records_count >= 5:
        checklist.append("[PASSED] Telemetry Log Length: Log contains sufficient telemetry frames.")
        score += 20.0
    else:
        checklist.append("[FAILED] Telemetry Log Length: Log must contain at least 5 telemetry frames.")

    if motor_actuated:
        checklist.append("[PASSED] Motor Actuation Check: Active PWM duty commands detected.")
        score += 20.0
    else:
        checklist.append("[FAILED] Motor Actuation Check: No motor duty commands (l/r keys) found in the log.")

    if encoders_changed:
        checklist.append("[PASSED] Quadrature Encoder Check: Active wheel movement ticks detected.")
        score += 20.0
    else:
        checklist.append("[FAILED] Quadrature Encoder Check: No wheel encoder updates detected. Ensure wheels rotate.")

    if tof_updated:
        checklist.append("[PASSED] Time-of-Flight Check: Active range readings detected.")
        score += 20.0
    else:
        checklist.append("[FAILED] Time-of-Flight Check: No Time-of-Flight readings detected. Check sensor soldering/power.")

    if gyro_active:
        checklist.append("[PASSED] Gyroscope Check: Yaw rate sensor updates detected.")
        score += 20.0
    else:
        checklist.append("[FAILED] Gyroscope Check: No Gyroscope readings detected. Check IMU soldering/power.")

    feedback.append("\n=== Hardware Diagnostics Checklist ===")
    feedback.extend(checklist)

    feedback.append("\n--------------------------------------------------")
    if score >= 100.0:
        feedback.append("RESULT: Milestone 0 Build Verification SUCCESSFUL!")
        feedback.append("Your hardware has been registered, and all subsystems are verified working.")
    else:
        feedback.append(f"RESULT: Milestone 0 FAILED (Score: {score}/100)")
        feedback.append("Please check the failed diagnostics items, verify your hardware connections, and re-submit a new log.")

    return score, "\n".join(feedback)
