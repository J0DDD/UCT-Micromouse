#!/usr/bin/env python3
import os
import sys
import subprocess
import time
import json
import signal
import glob
import importlib.util
import shutil
import tempfile

# 1. Path Resolution
if os.path.exists("/autograder"):
    SUBMISSION_DIR = "/autograder/submission"
    RESULTS_FILE = "/autograder/results/results.json"
    SOURCE_DIR = "/autograder/source"
    VIDEO_PATH = "/autograder/results/run.mp4"
    TRAJECTORY_JSON = os.path.join(tempfile.gettempdir(), "trajectory.json")
else:
    # Local mock mode
    base_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
    SUBMISSION_DIR = os.path.join(base_dir, "python")  # mock submission is the local python dir
    RESULTS_FILE = os.path.join(base_dir, "tools", "autograder", "results.json")
    SOURCE_DIR = os.path.join(base_dir, "tools", "autograder")
    VIDEO_PATH = os.path.join(base_dir, "tools", "autograder", "run.mp4")
    TRAJECTORY_JSON = os.path.join(base_dir, "tools", "autograder", "trajectory.json")

def write_results(score, feedback, test_name="Autograder Evaluation"):
    os.makedirs(os.path.dirname(RESULTS_FILE), exist_ok=True)
    results = {
        "score": score,
        "max_score": 100.0,
        "output": feedback,
        "visibility": "visible",
        "tests": [
            {
                "name": test_name,
                "score": score,
                "max_score": 100.0,
                "output": feedback,
                "visibility": "visible"
            }
        ]
    }
    with open(RESULTS_FILE, "w") as f:
        json.dump(results, f, indent=2)
    print(f"[Grader] Results written to {RESULTS_FILE} with score {score}")

def load_test_suite(assignment_name):
    suite_path = os.path.join(SOURCE_DIR, "assignments", assignment_name, "test_suite.py")
    if not os.path.exists(suite_path):
        # Local fallback if directory structured differently
        suite_path = os.path.join(os.path.dirname(__file__), "assignments", assignment_name, "test_suite.py")
        
    if not os.path.exists(suite_path):
        raise FileNotFoundError(f"Test suite not found at {suite_path}")
        
    spec = importlib.util.spec_from_file_location("test_suite", suite_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module

def main():
    print("=== UCT Micromouse Gradescope Autograder Runner ===")
    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
    
    global SUBMISSION_DIR, RESULTS_FILE
    import argparse
    parser = argparse.ArgumentParser(description="Gradescope Autograder Runner")
    parser.add_argument("--submission", type=str, default=None, help="Submission directory")
    parser.add_argument("--results", type=str, default=None, help="Results output file path")
    args, _ = parser.parse_known_args()
    
    if args.submission:
        SUBMISSION_DIR = os.path.abspath(args.submission)
        print(f"[Grader] Overridden SUBMISSION_DIR: {SUBMISSION_DIR}")
    
    if args.results:
        RESULTS_FILE = os.path.abspath(args.results)
        print(f"[Grader] Overridden RESULTS_FILE: {RESULTS_FILE}")
    
    # 2. Find active assignment config
    active_assignment_file = os.path.join(SOURCE_DIR, "active_assignment.txt")
    if not os.path.exists(active_assignment_file):
        # Local fallback
        active_assignment_file = os.path.join(os.path.dirname(__file__), "active_assignment.txt")
        
    if os.path.exists(active_assignment_file):
        with open(active_assignment_file, "r") as f:
            assignment_name = f.read().strip()
    else:
        assignment_name = "milestone1" # default fallback
        
    print(f"[Grader] Active assignment: {assignment_name}")

    # Dynamically resolve default submission folder to workspace folders if not overridden
    if not args.submission:
        assignment_workspaces = {
            "milestone1_square": os.path.join(repo_root, "workspace", "task1_square"),
            "milestone2_maze": os.path.join(repo_root, "workspace", "task2_maze"),
            "final_demo": os.path.join(repo_root, "workspace", "final_task")
        }
        fallback_dir = assignment_workspaces.get(assignment_name)
        if fallback_dir and os.path.exists(fallback_dir):
            SUBMISSION_DIR = fallback_dir
            print(f"[Grader] Default SUBMISSION_DIR resolved to active workspace: {SUBMISSION_DIR}")
        else:
            print(f"[Grader] Default SUBMISSION_DIR resolved to: {SUBMISSION_DIR}")
    
    try:
        test_suite = load_test_suite(assignment_name)
    except Exception as e:
        write_results(0.0, f"System Error: Failed to load test suite for assignment '{assignment_name}': {e}")
        return

    # Early exit for Milestone 0 (log submission only, no simulator needed)
    if assignment_name == "milestone0_verification":
        print("[Grader] Milestone 0 log evaluation track detected.")
        try:
            log_path = os.path.join(SUBMISSION_DIR, "run_log.jsonl")
            if not os.path.exists(log_path):
                # Scan for any .jsonl file in the submission folder as fallback
                candidates = glob.glob(os.path.join(SUBMISSION_DIR, "*.jsonl"))
                if candidates:
                    log_path = candidates[0]
            score, feedback = test_suite.evaluate_log(log_path)
            write_results(score, feedback, "Milestone 0 Evaluation")
        except Exception as e:
            write_results(0.0, f"Grading Error: Failed to evaluate log submission: {e}", "Milestone 0 Evaluation")
        return

    # 3. Detect submission track (Simulink vs Python)
    print(f"[Grader] Scanning submission directory: {SUBMISSION_DIR}")
    
    # Clean up any student-submitted uct_mouse.py or micromouse.py files to prevent shadowing of the grader's corrected libraries
    for root, dirs, files in os.walk(SUBMISSION_DIR):
        for f in files:
            if f in ["uct_mouse.py", "micromouse.py"]:
                target_path = os.path.join(root, f)
                print(f"[Grader] Cleaning up student-submitted framework override: {target_path}")
                try:
                    os.remove(target_path)
                except Exception as e:
                    print(f"[Grader] Warning: Failed to remove {target_path}: {e}")

    # Check for Simulink track by looking for any folder ending in _ert_rtw
    ert_dirs = []
    for root, dirs, files in os.walk(SUBMISSION_DIR):
        for d in dirs:
            if d.endswith("_ert_rtw"):
                ert_dirs.append(os.path.join(root, d))
                
    track = None
    model_dir = None
    model_name = None
    main_file = None
    
    if ert_dirs:
        # Prefer UCT_KDeploy_ert_rtw if multiple
        target_dir = None
        for d in ert_dirs:
            if os.path.basename(d) == "UCT_KDeploy_ert_rtw":
                target_dir = d
                break
        if not target_dir:
            target_dir = ert_dirs[0]
            
        track = "simulink"
        model_dir = target_dir
        model_name = os.path.basename(target_dir)[:-8]  # Strip '_ert_rtw'
        print(f"[Grader] Track detected: Simulink")
        print(f"[Grader] Found code generation folder: {model_dir}")
        print(f"[Grader] Model name: {model_name}")
    else:
        # Check for Python track by looking for <assignment_name>.py or main.py
        main_candidates = []
        target_name = f"{assignment_name}.py"
        
        for root, dirs, files in os.walk(SUBMISSION_DIR):
            if target_name in files:
                main_candidates.append(os.path.join(root, target_name))
            if "main.py" in files:
                main_candidates.append(os.path.join(root, "main.py"))
                
        if main_candidates:
            target_main = None
            
            # 1. Prioritize files named exactly <assignment_name>.py
            for p in main_candidates:
                if os.path.basename(p) == target_name:
                    target_main = p
                    break
                    
            # 2. Prioritize files in a folder named after the active assignment (e.g., milestone1/)
            if not target_main:
                for p in main_candidates:
                    path_parts = p.split(os.sep)
                    if assignment_name in path_parts or any(assignment_name in part for part in path_parts):
                        target_main = p
                        break
                        
            # 3. Prioritize files under python/ directory
            if not target_main:
                for p in main_candidates:
                    if "/python/" in p or p.endswith("python/main.py"):
                        target_main = p
                        break
                        
            # 4. Fallback
            if not target_main:
                target_main = main_candidates[0]
                
            track = "python"
            main_file = target_main
            print(f"[Grader] Track detected: Python")
            print(f"[Grader] Found entry point: {main_file}")
        else:
            write_results(0.0, f"Submission Error: Neither a Simulink code generation folder (*_ert_rtw), a Python entry point ({target_name}), nor a standard 'main.py' was found in your submission.")
            return

    # 4. Compilation if Simulink track
<<<<<<< HEAD
    client_bin = "simulink_client.exe"
=======
    client_bin = os.path.join(tempfile.gettempdir(), "simulink_client")
>>>>>>> ccec4d39a3e4fd3181170c9d5dd707616fc703bb
    if track == "simulink":
        print("[Grader] Compiling Simulink deployment code...")
        
        # Locate wrapper files in SOURCE_DIR
        pc_main = os.path.join(SOURCE_DIR, "PC_client_main.c")
        sim_wrapper = os.path.join(SOURCE_DIR, "simulink_wrapper.c")
        sim_header = os.path.join(SOURCE_DIR, "simulink_wrapper.h")
        
        # Verify wrapper files exist (if not in SOURCE_DIR, fallback to repo paths)
        repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
        if not os.path.exists(pc_main):
            pc_main = os.path.join(repo_root, "matlab", "simulink", "PC_client_main.c")
        if not os.path.exists(sim_wrapper):
            sim_wrapper = os.path.join(repo_root, "firmware", "src", "kernel", "src", "simulink_wrapper.c")
        if not os.path.exists(sim_header):
            sim_header = os.path.join(repo_root, "firmware", "src", "kernel", "inc", "simulink_wrapper.h")
            
        if not os.path.exists(pc_main) or not os.path.exists(sim_wrapper):
            write_results(0.0, "System Error: Missing standalone main client or simulink wrappers in autograder package.")
            return
            
        # Find all generated sources in model directory (exclude ert_main.c)
        model_sources = glob.glob(os.path.join(model_dir, "*.c"))
        model_sources = [f for f in model_sources if os.path.basename(f) != "ert_main.c"]
        
        all_sources = [pc_main, sim_wrapper] + model_sources
        
        # Choose compiler
        compiler = shutil.which("gcc") or shutil.which("clang")
        if not compiler:
            write_results(0.0, "System Error: No suitable C compiler (gcc or clang) found in the autograder environment.")
            return
            
        # Build compile command
        cmd = [
            compiler,
            "-O2",
            f"-DMODEL_NAME={model_name}",
            f"-I{model_dir}",
            f"-I{os.path.dirname(sim_header)}", # wrapper header folder
            f"-I{SOURCE_DIR}" # also include source dir
        ]
        cmd.extend(all_sources)
        cmd.extend(["-o", client_bin, "-lm"])
        
        print(f"[Grader] Compiler command: {' '.join(cmd)}")
        try:
            res = subprocess.run(cmd, capture_output=True, text=True, timeout=30.0)
            if res.returncode != 0:
                feedback = (
                    f"Compilation Error: Your Simulink generated C code failed to compile.\n\n"
                    f"--- Compiler Output (stdout) ---\n{res.stdout}\n\n"
                    f"--- Compiler Error (stderr) ---\n{res.stderr}"
                )
                write_results(0.0, feedback, "Compilation Check")
                return
            print("[Grader] Compilation succeeded.")
        except subprocess.TimeoutExpired:
            write_results(0.0, "Compilation Error: C compilation process timed out after 30 seconds.", "Compilation Check")
            return
        except Exception as e:
            write_results(0.0, f"Compilation Error: Failed to invoke compiler: {e}", "Compilation Check")
            return

    # 5. Execute Multi-Run Simulation Tests
    test_runs = getattr(test_suite, "TEST_RUNS", [("Standard Run", 1.0, 0.08, 0.08, False)])
    
    sim_script = os.path.join(SOURCE_DIR, "physics_sim.py")
    if not os.path.exists(sim_script):
        sim_script = os.path.join(repo_root, "tools", "physics_sim.py")
    if not os.path.exists(sim_script):
        sim_script = os.path.join(os.path.dirname(__file__), "..", "physics_sim.py")
    if not os.path.exists(sim_script):
        write_results(0.0, f"System Error: Simulator backend script 'physics_sim.py' not found (looked in {SOURCE_DIR}, {repo_root}/tools).")
        return
        
    total_score = 0.0
    gradescope_tests = []
    
    for idx, (run_name, weight, imb_val, slip_val, is_hidden) in enumerate(test_runs):
        print(f"\n[Grader] === Executing {run_name} (Weight: {weight*100:.0f}%, Imbalance: {imb_val}, Slip: {slip_val}) ===")
        
        sim_cmd = [
            sys.executable,
            "-u",
            sim_script,
            "--headless",
            "--map", getattr(test_suite, "MAP", "empty"),
            "--imbalance", str(imb_val),
            "--slip", str(slip_val),
            "--json-log", TRAJECTORY_JSON,
            "--video", VIDEO_PATH if idx == 0 else "", # only record video for first run
            "--max-time", str(getattr(test_suite, "TIME_LIMIT", 45.0)),
            "--seed", str(getattr(test_suite, "SEED", 42) + idx)
        ]
        
        # Clean up old trajectory file
        if os.path.exists(TRAJECTORY_JSON):
            try:
                os.remove(TRAJECTORY_JSON)
            except Exception:
                pass
                
<<<<<<< HEAD
        sim_log_path = "simulator_backend.log"
=======
        sim_log_path = os.path.join(tempfile.gettempdir(), "simulator_backend.log")
>>>>>>> ccec4d39a3e4fd3181170c9d5dd707616fc703bb
        if os.path.exists(sim_log_path):
            try:
                os.remove(sim_log_path)
            except Exception:
                pass
                
        try:
            sim_log_file = open(sim_log_path, "w")
            sim_proc = subprocess.Popen(
                sim_cmd,
                stdout=sim_log_file,
                stderr=sim_log_file,
                text=True
            )
            sim_log_file.close()
        except Exception as e:
            write_results(0.0, f"System Error: Failed to start simulation backend: {e}")
            return
            
        # Wait for simulator readiness (increased timeout to 20s for slow/cold container boot)
        simulator_ready = False
        exited_early = False
        start_wait = time.time()
        while time.time() - start_wait < 20.0:
            poll_status = sim_proc.poll()
            if poll_status is not None:
                exited_early = True
                break
            if os.path.exists(sim_log_path):
                try:
                    with open(sim_log_path, "r") as f:
                        log_content = f.read()
                        if "Waiting for student script to connect" in log_content:
                            simulator_ready = True
                            break
                except Exception:
                    pass
            time.sleep(0.1)
            
        if not simulator_ready:
            sim_proc.terminate()
            try:
                sim_proc.wait(timeout=2.0)
            except Exception:
                sim_proc.kill()
                
            # Fetch backend log output to show student/convenor what failed
            log_tail = ""
            if os.path.exists(sim_log_path):
                try:
                    with open(sim_log_path, "r") as f:
                        lines = f.readlines()
                        log_tail = "".join(lines[-15:])  # Grab last 15 lines of simulator logs
                except Exception as log_err:
                    log_tail = f"Could not read log file: {log_err}"
            
            if exited_early:
                write_results(
                    0.0,
                    f"System Error: Simulator backend exited early with code {poll_status}.\n\n"
                    f"--- Simulator Backend Log (Last 15 lines) ---\n{log_tail}"
                )
            else:
                write_results(
                    0.0,
                    f"System Error: Simulator failed to start or bind to port 8000 within 20s timeout.\n\n"
                    f"--- Simulator Backend Log (Last 15 lines) ---\n{log_tail}"
                )
            return
            
        # Run Student Client
        client_env = os.environ.copy()
        client_env["GRADESCOPE_AUTOGRADER"] = "1"
        
        if track == "python":
            client_cmd = [sys.executable, main_file]
            python_paths = [SOURCE_DIR, os.path.dirname(main_file), os.path.join(repo_root, "python")]
            if "PYTHONPATH" in os.environ:
                python_paths.append(os.environ["PYTHONPATH"])
            client_env["PYTHONPATH"] = os.path.pathsep.join(python_paths)
            client_cwd = os.path.dirname(main_file)
        else:
            client_cmd = [client_bin]
<<<<<<< HEAD
            client_cwd = "."
=======
            client_cwd = tempfile.gettempdir()
>>>>>>> ccec4d39a3e4fd3181170c9d5dd707616fc703bb
            
        try:
            client_proc = subprocess.Popen(
                client_cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                env=client_env,
                cwd=client_cwd,
                text=True
            )
        except Exception as e:
            sim_proc.send_signal(signal.SIGINT)
            try: sim_proc.wait(timeout=2.0)
            except Exception: sim_proc.kill()
            write_results(0.0, f"Execution Error: Failed to start student script/binary: {e}")
            return
            
        # Monitor
        time_limit = getattr(test_suite, "TIME_LIMIT", 45.0)
        max_duration = time_limit + 10.0
        start_time = time.time()
        client_exited = False
        timed_out = False
        
        while time.time() - start_time < max_duration:
            if not client_exited and client_proc.poll() is not None:
                client_exited = True
                time.sleep(1.5)
            if sim_proc.poll() is not None:
                break
            time.sleep(0.5)
        else:
            timed_out = True
            
        # Cleanup
        if client_proc.poll() is None:
            client_proc.terminate()
            try: client_proc.wait(timeout=2.0)
            except Exception: client_proc.kill()
            
        if sim_proc.poll() is None:
            sim_proc.send_signal(signal.SIGINT)
            try: sim_proc.wait(timeout=3.0)
            except Exception: sim_proc.kill()
            
        client_stdout, client_stderr = client_proc.communicate()
        
        sim_stdout = ""
        if os.path.exists(sim_log_path):
            try:
                with open(sim_log_path, "r") as f:
                    sim_stdout = f.read()
            except Exception:
                pass
                
        # Evaluate
        run_score = 0.0
        run_feedback = ""
        
        if not os.path.exists(TRAJECTORY_JSON) or os.path.getsize(TRAJECTORY_JSON) == 0:
            run_feedback = (
                f"Execution Error: No simulation trajectory was recorded.\n"
                f"Your script or binary did not connect to the simulator on port 8000.\n\n"
                f"--- Console Output (stdout) ---\n{client_stdout}\n\n"
                f"--- Error Output (stderr) ---\n{client_stderr}\n"
            )
        else:
            try:
                raw_score, run_feedback = test_suite.evaluate_run(TRAJECTORY_JSON)
                run_score = raw_score
            except Exception as e:
                run_feedback = f"System Error: Failed to evaluate simulation results: {e}"
                
        weighted_score = run_score * weight
        total_score += weighted_score
        
        run_visibility = "after_due_date" if is_hidden else "visible"
        
        run_report = [
            f"=== {run_name} ===",
            f"Weight: {weight*100:.0f}%",
            f"Raw Score: {run_score:.1f} / 100.0 pts",
            f"Weighted Score Contribution: {weighted_score:.1f} pts",
            f"Visibility: {run_visibility.replace('_', ' ').capitalize()}",
            "-" * 50,
            run_feedback,
            ""
        ]
        if client_stdout:
            run_report.append(f"--- Student Output (stdout) ---\n{client_stdout}\n")
        if client_stderr:
            run_report.append(f"--- Student Errors (stderr) ---\n{client_stderr}\n")
        if sim_stdout:
            run_report.append(f"--- Simulator Output ---\n{sim_stdout}\n")
            
        joined_run_report = "\n".join(run_report)
        
        gradescope_tests.append({
            "name": run_name,
            "score": round(weighted_score, 2),
            "max_score": round(weight * 100.0, 2),
            "output": joined_run_report,
            "visibility": run_visibility
        })
        
        print(f"[Grader] Completed {run_name}: Score {run_score}/100 (Weighted: {weighted_score})")

    # 6. Write Consolidated Results JSON File
    os.makedirs(os.path.dirname(RESULTS_FILE), exist_ok=True)
    final_score = round(total_score, 2)
    
    results = {
        "score": final_score,
        "max_score": 100.0,
        "output": f"=== Final Grade Summary ===\n  Milestone 1 Combined Score: {final_score:.2f} / 100.0 pts\n==========================",
        "visibility": "visible",
        "tests": gradescope_tests
    }
    
    with open(RESULTS_FILE, "w") as f:
        json.dump(results, f, indent=2)
        
    print(f"\n[Grader] All test runs finished. Final combined score: {final_score}% written to {RESULTS_FILE}")

if __name__ == "__main__":
    main()
