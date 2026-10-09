#!/usr/bin/env python3
"""
UCT Micromouse Controller Robustness & Perturbation Test Suite
Runs student control scripts against multiple physical simulation tracks
(motor imbalances, surface slip, dynamic skid, and compound perturbations)
to evaluate closed-loop disturbance rejection and trajectory accuracy.
"""

import os
import sys
import subprocess
import time
import json
import signal
import argparse
import tempfile
import math

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
PHYSICS_SIM = os.path.join(REPO_ROOT, "tools", "physics_sim.py")

DEFAULT_M1_TESTS = [
    {"name": "1. Public Baseline Run", "map": "empty", "imbalance": 0.05, "slip": 0.03, "seed": 42},
    {"name": "2. Pos. Motor Imbalance (+12%)", "map": "empty", "imbalance": 0.12, "slip": 0.04, "seed": 43},
    {"name": "3. Neg. Motor Imbalance (-12%)", "map": "empty", "imbalance": -0.12, "slip": 0.04, "seed": 44},
    {"name": "4. High Traction Slip (10%)", "map": "empty", "imbalance": 0.06, "slip": 0.10, "seed": 45},
    {"name": "5. Turn Skid & Decel (-12%/8%)", "map": "empty", "imbalance": -0.12, "slip": 0.08, "seed": 46},
    {"name": "6. Compound Drift (+14%/10%)", "map": "empty", "imbalance": 0.14, "slip": 0.10, "seed": 47},
]

DEFAULT_M2_TESTS = [
    {"name": "1. Baseline Maze (Seed 42)", "map": "random", "imbalance": 0.08, "slip": 0.02, "seed": 42},
    {"name": "2. High Imbalance (+12%, Seed 101)", "map": "random", "imbalance": 0.12, "slip": 0.03, "seed": 101},
    {"name": "3. Neg. Imbalance (-12%, Seed 256)", "map": "random", "imbalance": -0.12, "slip": 0.03, "seed": 256},
    {"name": "4. High Slip (8%, Seed 512)", "map": "random", "imbalance": 0.06, "slip": 0.08, "seed": 512},
    {"name": "5. Alternate Maze (Seed 1024)", "map": "random", "imbalance": 0.08, "slip": 0.04, "seed": 1024},
    {"name": "6. Extreme Track (+14%/8%, Seed 2048)", "map": "random", "imbalance": 0.14, "slip": 0.08, "seed": 2048},
]

def unwrap_angles(thetas):
    if not thetas:
        return []
    res = [thetas[0]]
    for t in thetas[1:]:
        diff = t - res[-1]
        diff = (diff + math.pi) % (2.0 * math.pi) - math.pi
        res.append(res[-1] + diff)
    return res

def evaluate_m1_trajectory(data):
    if not data or "trajectory" not in data or len(data["trajectory"]) < 10:
        return {"score": 0.0, "crashed": True, "parking_err": 99.9, "runtime": 0.0, "notes": "No trajectory"}

    start_x = data.get("start_x", 0.0)
    start_y = data.get("start_y", 0.0)
    final_x = data.get("final_x", 0.0)
    final_y = data.get("final_y", 0.0)
    sim_time = data.get("time", 0.0)
    crashed = data.get("crashed", False)
    trajectory = data.get("trajectory", [])

    d_e = math.hypot(final_x - start_x, final_y - start_y)

    raw_thetas = [pt[2] for pt in trajectory]
    unwrapped = unwrap_angles(raw_thetas)
    final_angle_change = unwrapped[-1] - unwrapped[0]
    direction_sign = -1.0 if final_angle_change < -math.pi / 2.0 else 1.0
    angles = [(th - unwrapped[0]) * direction_sign for th in unwrapped]

    legs_points = {0: [], 1: [], 2: [], 3: []}
    turns_points = {0: [], 1: [], 2: [], 3: []}
    current_state = 0
    turn4_end_heading = None

    for idx, pt in enumerate(trajectory):
        tx, ty = pt[0], pt[1]
        th = angles[idx]
        raw_th = raw_thetas[idx]
        if current_state == 0:
            if th < math.radians(15.0): legs_points[0].append((tx, ty, raw_th))
            else: current_state = 1; turns_points[0].append((tx, ty, raw_th))
        elif current_state == 1:
            if th < math.radians(75.0): turns_points[0].append((tx, ty, raw_th))
            else: current_state = 2; legs_points[1].append((tx, ty, raw_th))
        elif current_state == 2:
            if th < math.radians(105.0): legs_points[1].append((tx, ty, raw_th))
            else: current_state = 3; turns_points[1].append((tx, ty, raw_th))
        elif current_state == 3:
            if th < math.radians(165.0): turns_points[1].append((tx, ty, raw_th))
            else: current_state = 4; legs_points[2].append((tx, ty, raw_th))
        elif current_state == 4:
            if th < math.radians(195.0): legs_points[2].append((tx, ty, raw_th))
            else: current_state = 5; turns_points[2].append((tx, ty, raw_th))
        elif current_state == 5:
            if th < math.radians(255.0): turns_points[2].append((tx, ty, raw_th))
            else: current_state = 6; legs_points[3].append((tx, ty, raw_th))
        elif current_state == 6:
            if th < math.radians(285.0): legs_points[3].append((tx, ty, raw_th))
            else: current_state = 7; turns_points[3].append((tx, ty, raw_th))
        elif current_state == 7:
            if th < math.radians(345.0): turns_points[3].append((tx, ty, raw_th))
            else: current_state = 8; turn4_end_heading = raw_th
        elif current_state == 8:
            turn4_end_heading = raw_th

    if turn4_end_heading is None and len(trajectory) > 0 and (current_state >= 7 or len(turns_points[3]) > 0):
        turn4_end_heading = raw_thetas[-1]

    # Score straight legs (35 pts max)
    leg_scores = []
    for i in range(4):
        pts = legs_points[i]
        if len(pts) < 2:
            leg_scores.append(0.0)
            continue
        x0, y0 = pts[0][0], pts[0][1]
        x1, y1 = pts[-1][0], pts[-1][1]
        line_len = math.hypot(x1 - x0, y1 - y0)
        len_error = abs(line_len - 1.0)
        len_sc = 4.375 if len_error <= 0.025 else max(0.0, 4.375 - (len_error - 0.025) / 0.095 * 4.375)
        
        max_dev = 0.0
        if line_len > 1e-3:
            for pt in pts:
                px, py = pt[0], pt[1]
                dev = abs((y1 - y0) * px - (x1 - x0) * py + x1 * y0 - y1 * x0) / line_len
                max_dev = max(max_dev, dev)
        dev_sc = 4.375 if max_dev <= 0.012 else max(0.0, 4.375 - (max_dev - 0.012) / 0.058 * 4.375)
        leg_scores.append(len_sc + dev_sc)

    # Score corner turns (35 pts max)
    def circular_mean(thetas):
        return math.atan2(sum(math.sin(th) for th in thetas), sum(math.cos(th) for th in thetas))

    leg_headings = {}
    target_headings = [0.0, math.pi / 2.0, math.pi, -math.pi / 2.0]
    for i in range(4):
        leg_headings[i] = circular_mean([pt[2] for pt in legs_points[i]]) if len(legs_points[i]) > 0 else target_headings[i]

    turn_scores = []
    for i in range(4):
        if len(legs_points[i]) < 3:
            turn_scores.append(0.0)
            continue
        h_start = leg_headings[i]
        if i < 3:
            if len(legs_points[i + 1]) < 3:
                turn_scores.append(0.0); continue
            h_end = leg_headings[i + 1]
        else:
            if turn4_end_heading is not None: h_end = turn4_end_heading
            elif turns_points[3]: h_end = turns_points[3][-1][2]
            else: turn_scores.append(0.0); continue
        
        turn_angle = (h_end - h_start + math.pi) % (2.0 * math.pi) - math.pi
        turn_deg = (math.degrees(turn_angle) + 360.0) % 360.0
        if direction_sign < 0: turn_deg = 360.0 - turn_deg
        turn_error = abs(turn_deg - 90.0)
        t_sc = 8.75 if turn_error <= 1.5 else max(0.0, 8.75 - (turn_error - 1.5) / 4.5 * 8.75)
        turn_scores.append(t_sc)

    # Parking score (20 pts max)
    parking_sc = 20.0 if d_e <= 0.018 else max(0.0, 20.0 - (d_e - 0.018) / 0.062 * 20.0)

    # Speed cadence (10 pts max)
    speed_sc = 10.0 if sim_time <= 18.0 else max(0.0, 10.0 - (sim_time - 18.0) / 14.0 * 10.0)

    total_score = sum(leg_scores) + sum(turn_scores) + parking_sc + speed_sc
    return {
        "score": round(total_score, 1),
        "crashed": crashed,
        "detail": f"Park: {d_e*100.0:.1f}cm",
        "runtime": round(sim_time, 1)
    }

def evaluate_m2_trajectory(data):
    if not data or "trajectory" not in data or len(data.get("trajectory", [])) < 5:
        return {"score": 0.0, "crashed": True, "detail": "No Trajectory", "runtime": 0.0}

    start_x = data.get("start_x", 0.10)
    start_y = data.get("start_y", 0.10)
    final_x = data.get("final_x", start_x)
    final_y = data.get("final_y", start_y)
    sim_time = data.get("time", 0.0)
    crashed = data.get("crashed", False)
    trajectory = data.get("trajectory", [])

    CELL_DIM = 0.20
    if "target_room" in data and data["target_room"]:
        cx, cy = data["target_room"]
        target_center_x = cx * CELL_DIM
        target_center_y = cy * CELL_DIM
    else:
        target_center_x = 0.80
        target_center_y = 0.40
    target_radius = 0.28

    entered_target = False
    target_entry_idx = -1
    min_dist_to_target = 999.0

    for idx, pt in enumerate(trajectory):
        d = math.hypot(pt[0] - target_center_x, pt[1] - target_center_y)
        if d < min_dist_to_target:
            min_dist_to_target = d
        if d <= target_radius and not entered_target:
            entered_target = True
            target_entry_idx = idx

    pirouette_detected = False
    if entered_target:
        for start_i in range(target_entry_idx, len(trajectory) - 5):
            if math.hypot(trajectory[start_i][0] - target_center_x, trajectory[start_i][1] - target_center_y) > target_radius:
                continue
            yaw_accum = 0.0
            spin_start_pt = trajectory[start_i]
            prev_theta = spin_start_pt[2]
            max_linear_disp = 0.0
            for end_i in range(start_i + 1, min(len(trajectory), start_i + 350)):
                pt = trajectory[end_i]
                d_theta = pt[2] - prev_theta
                while d_theta > math.pi: d_theta -= 2 * math.pi
                while d_theta < -math.pi: d_theta += 2 * math.pi
                yaw_accum += abs(d_theta)
                prev_theta = pt[2]
                disp = math.hypot(pt[0] - spin_start_pt[0], pt[1] - spin_start_pt[1])
                if disp > max_linear_disp:
                    max_linear_disp = disp
                if yaw_accum >= 5.5 and max_linear_disp <= 0.12:
                    pirouette_detected = True
                    break
            if pirouette_detected:
                break

    returned_to_start = False
    if entered_target and target_entry_idx >= 0:
        for pt in trajectory[target_entry_idx:]:
            d_start = math.hypot(pt[0] - start_x, pt[1] - start_y)
            if d_start <= 0.15:
                returned_to_start = True
                break

    sprint_completed = False
    if returned_to_start:
        final_dist_to_target = math.hypot(final_x - target_center_x, final_y - target_center_y)
        if final_dist_to_target <= target_radius:
            sprint_completed = True

    score = 0.0
    if entered_target: score += 30.0
    else: score += max(0.0, 30.0 * (1.0 - min_dist_to_target / 1.0))

    if pirouette_detected: score += 20.0
    if returned_to_start: score += 20.0
    if sprint_completed: score += 20.0

    if sprint_completed and sim_time <= 25.0: score += 10.0
    elif sprint_completed and sim_time < 90.0: score += max(0.0, 10.0 * (1.0 - (sim_time - 25.0) / 65.0))

    phases = f"T:{'✓' if entered_target else '✗'} P:{'✓' if pirouette_detected else '✗'} R:{'✓' if returned_to_start else '✗'} S:{'✓' if sprint_completed else '✗'}"

    return {
        "score": round(score, 1),
        "crashed": crashed,
        "detail": phases,
        "runtime": round(sim_time, 1)
    }

def run_single_test(script_path, test_cfg, task="milestone1", port=8000):
    traj_file = tempfile.mktemp(suffix=".json", prefix="traj_test_")
    env = os.environ.copy()
    env["PYTHONPATH"] = os.path.join(REPO_ROOT, "python") + ":" + env.get("PYTHONPATH", "")
    env["UCT_OFFLINE_MODE"] = "1"
    
    max_time = "90.0" if task in ("milestone2", "task2", "final_demo") else "45.0"
    sim_cmd = [
        sys.executable, PHYSICS_SIM,
        "--headless",
        "--port", str(port),
        "--map", test_cfg.get("map", "empty"),
        "--imbalance", str(test_cfg["imbalance"]),
        "--slip", str(test_cfg["slip"]),
        "--seed", str(test_cfg["seed"]),
        "--json-log", traj_file,
        "--max-time", max_time
    ]

    env["UCT_MICROMOUSE_PORT"] = str(port)

    sim_proc = subprocess.Popen(sim_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(0.4)

    student_proc = subprocess.Popen(
        [sys.executable, script_path],
        cwd=os.path.dirname(script_path) or REPO_ROOT,
        env=env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )

    timeout_limit = 50.0 if max_time == "45.0" else 100.0
    try:
        student_proc.wait(timeout=timeout_limit)
    except subprocess.TimeoutExpired:
        student_proc.kill()

    try:
        sim_proc.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        sim_proc.kill()

    time.sleep(0.2)

    data = None
    if os.path.exists(traj_file):
        try:
            with open(traj_file, "r") as f:
                data = json.load(f)
            os.remove(traj_file)
        except Exception:
            pass

    if task in ("milestone2", "task2", "final_demo"):
        return evaluate_m2_trajectory(data)
    else:
        return evaluate_m1_trajectory(data)

def main():
    parser = argparse.ArgumentParser(description="Test student micromouse controller against physical perturbation stress tests.")
    parser.add_argument("script_pos", nargs="?", default=None, help="Path to student controller script (positional).")
    parser.add_argument("--script", default=None, help="Path to student controller script (optional).")
    parser.add_argument("--task", choices=["milestone1", "milestone2", "task1", "task2"], default="auto", help="Task milestone type (default: auto-detect from script name).")
    parser.add_argument("--runs", type=int, default=6, help="Number of test runs to execute.")
    parser.add_argument("--randomize", action="store_true", help="Use randomized perturbations instead of standard suite.")
    args = parser.parse_args()

    chosen_script = args.script or args.script_pos or "python/milestone1_square.py"
    script_path = os.path.abspath(chosen_script)
    if not os.path.exists(script_path):
        print(f"❌ Error: Target script '{chosen_script}' not found.")
        sys.exit(1)

    # Auto-detect task
    task = args.task
    if task == "auto":
        script_lower = os.path.basename(chosen_script).lower()
        if "maze" in script_lower or "task2" in script_lower or "milestone2" in script_lower:
            task = "milestone2"
        else:
            task = "milestone1"

    print("=" * 85)
    print(f" 🐭 UCT MICROMOUSE CONTROLLER ROBUSTNESS & STRESS TEST SUITE ({task.upper()})")
    print(f" Target Script: {os.path.relpath(script_path, REPO_ROOT)}")
    print("=" * 85)

    base_tests = DEFAULT_M2_TESTS if task in ("milestone2", "task2") else DEFAULT_M1_TESTS
    tests = base_tests[:args.runs]
    if args.randomize:
        import random
        tests = []
        map_type = "random" if task in ("milestone2", "task2") else "empty"
        for i in range(args.runs):
            imb = round(random.uniform(-0.15, 0.15), 2)
            slp = round(random.uniform(0.02, 0.12), 2)
            sd = random.randint(100, 9999)
            tests.append({"name": f"Random Track {i+1} (Imb: {imb:+.0%}, Slip: {slp:.0%})", "map": map_type, "imbalance": imb, "slip": slp, "seed": sd})

    results = []
    detail_header = "Phases (T/P/R/S)" if task in ("milestone2", "task2") else "Parking Err"
    print(f"{'#':<3} | {'Track / Perturbation':<35} | {'Imbalance':<10} | {'Slip':<6} | {'Result':<10} | {detail_header:<15} | {'Score':<8}")
    print("-" * 85)

    for idx, t in enumerate(tests, 1):
        eval_res = run_single_test(script_path, t, task=task, port=8000)
        score = eval_res["score"]
        crashed = eval_res["crashed"]
        status_str = "💥 CRASH" if crashed else ("✅ PASS" if score >= 70.0 else "⚠️ DRIFT")
        detail_str = str(eval_res.get("detail", "N/A"))
        
        print(f"{idx:<3} | {t['name']:<35} | {t['imbalance']:+6.1%}     | {t['slip']:4.1%} | {status_str:<10} | {detail_str:<15} | {score:5.1f}%")
        results.append(score)

    avg_score = sum(results) / len(results) if results else 0.0
    pass_count = sum(1 for s in results if s >= 70.0)

    print("=" * 85)
    print(f" Summary: {pass_count}/{len(results)} Tracks Passed (>=70%) | Average Robustness Score: {avg_score:.1f}%")
    if avg_score >= 80.0:
        print(" 🏆 Outstanding Controller! High disturbance rejection across all tracks.")
    elif avg_score >= 60.0:
        print(" 👍 Solid Controller, but displays moderate drift under extreme perturbations.")
    else:
        print(" ⚠️ Controller Needs Tuning: Lacks closed-loop sensor error correction.")
    print("=" * 85)

if __name__ == "__main__":
    main()
