#!/usr/bin/env python3
import os
import sys
import time
import json
import csv
import subprocess
import tempfile
import statistics
import concurrent.futures

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
SUBMISSIONS_DIR = os.path.join(REPO_ROOT, "workspace", "sub1_marking", "submissions")
SCORES_CSV = os.path.join(REPO_ROOT, "workspace", "sub1_marking", "submission1_scores.csv")
CALIBRATED_CSV = os.path.join(REPO_ROOT, "workspace", "sub1_marking", "final_milestone1_calibrated_grades.csv")
SIM_SCRIPT = os.path.join(REPO_ROOT, "tools", "physics_sim.py")
TEST_SUITE_PY = os.path.join(REPO_ROOT, "tools", "autograder", "assignments", "milestone1_square", "test_suite.py")

sys.path.insert(0, os.path.dirname(TEST_SUITE_PY))
import test_suite

PYTHON_EXE = "/opt/local/bin/python3.13" if os.path.exists("/opt/local/bin/python3.13") else sys.executable

def find_primary_main(submission_dir):
    candidates = ["milestone1_square.py", "main.py", "milestone1.py", "square.py", "run_square.py"]
    for c in candidates:
        p = os.path.join(submission_dir, c)
        if os.path.exists(p):
            return p
    for root, dirs, files in os.walk(submission_dir):
        if "test" in root.lower() or "__pycache__" in root.lower():
            continue
        for c in candidates:
            if c in files:
                return os.path.join(root, c)
    return None

def evaluate_single_run_port(student_main, test_cfg, port=8000, seed=42):
    run_name = test_cfg[0]
    weight = test_cfg[1]
    imb_val = test_cfg[2]
    slip_val = test_cfg[3]
    
    with tempfile.TemporaryDirectory() as tmpdir:
        traj_json = os.path.join(tmpdir, "trajectory.json")
        sim_log = os.path.join(tmpdir, "sim.log")
        
        sim_env = os.environ.copy()
        sim_env["SDL_VIDEODRIVER"] = "dummy"
        sim_env["SDL_AUDIODRIVER"] = "dummy"
        sim_env["PYTHONUNBUFFERED"] = "1"
        
        sim_cmd = [
            PYTHON_EXE, "-u", SIM_SCRIPT,
            "--headless",
            "--map", "empty",
            "--port", str(port),
            "--imbalance", str(imb_val),
            "--slip", str(slip_val),
            "--json-log", traj_json,
            "--seed", str(seed)
        ]
        
        log_f = open(sim_log, "w")
        sim_proc = subprocess.Popen(sim_cmd, stdout=log_f, stderr=subprocess.STDOUT, env=sim_env)
        
        bound = False
        for _ in range(50):
            time.sleep(0.04)
            if os.path.exists(sim_log):
                with open(sim_log, "r") as lf:
                    content = lf.read()
                    if "Server listening" in content:
                        bound = True
                        break
        if not bound:
            sim_proc.terminate()
            try: sim_proc.wait(timeout=0.5)
            except Exception: sim_proc.kill()
            log_f.close()
            return 0.0, "Simulator bind failed"
            
        client_env = os.environ.copy()
        python_lib = os.path.join(REPO_ROOT, "python")
        existing_pp = client_env.get("PYTHONPATH", "")
        client_dir = os.path.dirname(student_main)
        client_env["PYTHONPATH"] = f"{python_lib}:{client_dir}:{existing_pp}"
        client_env["GRADESCOPE_AUTOGRADER"] = "1"
        client_env["UCT_MICROMOUSE_FAST_SIM"] = "1"
        client_env["UCT_MICROMOUSE_PORT"] = str(port)
        
        try:
            client_proc = subprocess.Popen(
                [PYTHON_EXE, "-u", student_main],
                cwd=client_dir,
                env=client_env,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            stdout, stderr = client_proc.communicate(timeout=15.0)
        except subprocess.TimeoutExpired:
            client_proc.terminate()
            try: client_proc.kill()
            except Exception: pass
            sim_proc.terminate()
            try: sim_proc.kill()
            except Exception: pass
            log_f.close()
            return 0.0, "Timed out (15s limit)"
        except Exception as e:
            sim_proc.terminate()
            try: sim_proc.kill()
            except Exception: pass
            log_f.close()
            return 0.0, f"Client error: {e}"
            
        try:
            sim_proc.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            sim_proc.terminate()
            try: sim_proc.kill()
            except Exception: pass
            
        log_f.close()
        
        is_hidden = test_cfg[4] if len(test_cfg) > 4 else False
        score, fb = test_suite.evaluate_run(traj_json, is_hidden=is_hidden)
        return score, fb

def evaluate_student(sdir, tests, worker_id=0):
    port = 8100 + worker_id
    full_sdir = os.path.abspath(os.path.join(SUBMISSIONS_DIR, sdir))
    student_main = find_primary_main(full_sdir)
    if not student_main:
        return sdir, 0.0, [0.0]*len(tests), "NO_MAIN"
    student_main = os.path.abspath(student_main)
        
    scores = []
    weighted_total = 0.0
    for idx, t_cfg in enumerate(tests):
        seed = 42 + idx
        s, fb = evaluate_single_run_port(student_main, t_cfg, port=port, seed=seed)
        scores.append(round(s, 2))
        weighted_total += s * t_cfg[1]
        
    return sdir, round(weighted_total, 2), scores, "OK"

def main():
    student_dirs = sorted([d for d in os.listdir(SUBMISSIONS_DIR) if os.path.isdir(os.path.join(SUBMISSIONS_DIR, d))])
    print(f"Total student submissions found: {len(student_dirs)}")
    
    # Load historical video marks to get overall course %
    historical_video = {}
    if os.path.exists(CALIBRATED_CSV):
        with open(CALIBRATED_CSV, "r", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            for row in reader:
                sid = row.get("OrgDefinedId", "").strip()
                v_pts = float(row.get("Practical Video & Compliance (40.0 pts)", 0.0))
                historical_video[sid] = v_pts
                
    candidate_suites = {
        "6-Test Calibrated Suite (25% Public, 5x15% Hidden)": [
            ("Test 1: Public Baseline Run", 0.25, 0.05, 0.03, False),
            ("Test 2: Hidden Positive Imbalance (+12%)", 0.15, 0.12, 0.06, True),
            ("Test 3: Hidden Negative Imbalance (-12%)", 0.15, -0.12, 0.06, True),
            ("Test 4: Hidden Traction Slip & Spin (12% Slip)", 0.15, 0.06, 0.12, True),
            ("Test 5: Hidden Turn Settling & Dynamic Skid", 0.15, -0.12, 0.10, True),
            ("Test 6: Hidden Compound Perturbation", 0.15, 0.12, 0.12, True)
        ],
        "5-Test Suite (20% Public, 4x20% Hidden)": [
            ("Test 1: Public Baseline Run", 0.20, 0.05, 0.03, False),
            ("Test 2: Hidden Positive Imbalance (+12%)", 0.20, 0.12, 0.06, True),
            ("Test 3: Hidden Negative Imbalance (-12%)", 0.20, -0.12, 0.06, True),
            ("Test 4: Hidden Traction Slip & Spin (12% Slip)", 0.20, 0.06, 0.12, True),
            ("Test 5: Hidden Compound Dynamic Perturbation", 0.20, -0.12, 0.10, True)
        ],
        "5-Test Suite (25% Public, 4x18.75% Hidden)": [
            ("Test 1: Public Baseline Run", 0.25, 0.05, 0.03, False),
            ("Test 2: Hidden Positive Imbalance (+12%)", 0.1875, 0.12, 0.06, True),
            ("Test 3: Hidden Negative Imbalance (-12%)", 0.1875, -0.12, 0.06, True),
            ("Test 4: Hidden Traction Slip & Spin (12% Slip)", 0.1875, 0.06, 0.12, True),
            ("Test 5: Hidden Compound Dynamic Perturbation", 0.1875, -0.12, 0.10, True)
        ]
    }
    
    # Test on a representative sample of 30 students first, then full cohort
    sample_size = int(sys.argv[1]) if len(sys.argv) > 1 else len(student_dirs)
    sample_dirs = student_dirs[:sample_size]
    print(f"Running evaluation on {len(sample_dirs)} submissions using 8 parallel workers...")
    
    for suite_name, test_runs in candidate_suites.items():
        print(f"\n=======================================================")
        print(f"Evaluating Suite: {suite_name}")
        print(f"=======================================================")
        t0 = time.time()
        
        all_results = []
        with concurrent.futures.ThreadPoolExecutor(max_workers=8) as executor:
            futures = {
                executor.submit(evaluate_student, sdir, test_runs, i % 8): sdir
                for i, sdir in enumerate(sample_dirs)
            }
            for fut in concurrent.futures.as_completed(futures):
                all_results.append(fut.result())
                
        dt = time.time() - t0
        auto_scores = [r[1] for r in all_results]
        mean_auto = statistics.mean(auto_scores)
        median_auto = statistics.median(auto_scores)
        stdev_auto = statistics.stdev(auto_scores) if len(auto_scores) > 1 else 0.0
        
        # Calculate overall percentage (Autograder 60% + Video 40%)
        overall_percentages = []
        for sdir, auto_score, _, _ in all_results:
            sid = sdir.split("_")[0]
            v_pts = historical_video.get(sid, 34.5) # default video ~86% (34.5/40)
            final_grade = (auto_score * 0.60) + v_pts
            overall_percentages.append(final_grade)
            
        mean_overall = statistics.mean(overall_percentages)
        
        print(f"Completed in {dt:.1f}s")
        print(f"Autograder Score (out of 100):")
        print(f"  Mean   : {mean_auto:.2f}%")
        print(f"  Median : {median_auto:.2f}%")
        print(f"  StdDev : {stdev_auto:.2f}%")
        print(f"  Min/Max: {min(auto_scores):.1f}% / {max(auto_scores):.1f}%")
        print(f"Combined Final Course Grade (out of 100%):")
        print(f"  Mean Final Grade: {mean_overall:.2f}%")

if __name__ == "__main__":
    main()
