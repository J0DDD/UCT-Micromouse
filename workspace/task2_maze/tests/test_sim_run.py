"""Run main.py against the physics simulator on one or more seeds and score
each run with the official milestone2 evaluator.

Usage:
  python tests/test_sim_run.py                      # the 5 grader test cases
  python tests/test_sim_run.py --seeds 1-20 --rows 4 --cols 6 --imb 0.06 --slip 0.04
  python tests/test_sim_run.py --seeds 42 --video run.mp4 --verbose
"""
import argparse
import importlib.util
import json
import os
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
TASK_DIR = os.path.dirname(HERE)
REPO = os.path.abspath(os.path.join(TASK_DIR, "..", ".."))
SIM = os.path.join(REPO, "tools", "physics_sim.py")
SUITE = os.path.join(REPO, "tools", "autograder", "assignments", "milestone2_maze", "test_suite.py")

GRADER_CASES = [  # (rows, cols, imbalance, slip, seed, max_time) from test_suite.TEST_RUNS
    (4, 6, 0.04, 0.02, 42, 120.0),
    (4, 6, 0.06, 0.02, 43, 120.0),
    (4, 6, -0.06, 0.02, 44, 120.0),
    (4, 6, 0.04, 0.04, 45, 120.0),
    (8, 8, 0.05, 0.03, 48, 240.0),
]


def load_suite():
    spec = importlib.util.spec_from_file_location("m2_suite", SUITE)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def parse_seeds(text):
    seeds = []
    for part in text.split(","):
        if "-" in part:
            a, b = part.split("-")
            seeds.extend(range(int(a), int(b) + 1))
        else:
            seeds.append(int(part))
    return seeds


def wait_ready(sim, timeout=20.0):
    """Block until the simulator prints that it is listening (like grade_runner),
    then keep draining its output so it can never block on a full pipe."""
    import threading
    t0 = time.time()
    for line in sim.stdout:
        if "Waiting for student script" in line or time.time() - t0 > timeout:
            break
    threading.Thread(target=lambda: [None for _ in sim.stdout], daemon=True).start()


def start_sim(cmd):
    sim = subprocess.Popen([cmd[0], "-u"] + cmd[1:], stdout=subprocess.PIPE,
                           stderr=subprocess.STDOUT, text=True)
    wait_ready(sim)
    return sim


def start_run(case, port, config, video, verbose):
    rows, cols, imb, slip, seed, max_time = case
    traj = os.path.join(tempfile.gettempdir(), "m2_traj_%d.json" % port)
    if os.path.exists(traj):
        os.remove(traj)
    cmd = [sys.executable, SIM, "--headless", "--map", "random", "--port", str(port),
           "--imbalance", str(imb), "--slip", str(slip), "--seed", str(seed),
           "--rows", str(rows), "--cols", str(cols), "--max-time", str(max_time),
           "--json-log", traj]
    if config:
        cmd += ["--config", config]
    if video:
        cmd += ["--video", video]
    sim = start_sim(cmd)
    env = os.environ.copy()
    env["GRADESCOPE_AUTOGRADER"] = "1"
    env["UCT_MICROMOUSE_PORT"] = str(port)
    if verbose:
        env["MAZE_VERBOSE"] = "1"
    out = None if verbose else subprocess.DEVNULL
    client = subprocess.Popen([sys.executable, os.path.join(TASK_DIR, "main.py")],
                              cwd=TASK_DIR, env=env, stdout=out, stderr=subprocess.PIPE, text=True)
    return {"case": case, "sim": sim, "client": client, "traj": traj, "t0": time.time()}


def finish_run(run, suite, timeout=200.0):
    try:
        _, err = run["client"].communicate(timeout=timeout)
    except subprocess.TimeoutExpired:
        run["client"].kill()
        err = "client timeout"
    try:
        run["sim"].wait(timeout=15)
    except subprocess.TimeoutExpired:
        run["sim"].terminate()
        run["sim"].wait(timeout=5)
    if not os.path.exists(run["traj"]):
        return 0.0, "no trajectory written\n" + (err or "")[-1500:]
    score, fb = suite.evaluate_run(run["traj"])
    if err and "Traceback" in err:
        fb += "\nCLIENT ERROR:\n" + err[-1500:]
    return score, fb


def summarize(fb):
    keys = ("Simulation Time", "Collision", "[Phase", "[Bonus")
    lines = [l.strip() for l in fb.splitlines() if any(k in l for k in keys)]
    return "\n    ".join(lines)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", default=None)
    ap.add_argument("--rows", type=int, default=4)
    ap.add_argument("--cols", type=int, default=6)
    ap.add_argument("--imb", type=float, default=0.06)
    ap.add_argument("--slip", type=float, default=0.04)
    ap.add_argument("--config", default=None, help="sim config (default: simulator default)")
    ap.add_argument("--parallel", type=int, default=4)
    ap.add_argument("--video", default=None)
    ap.add_argument("--verbose", action="store_true")
    ap.add_argument("--full", action="store_true", help="print full feedback")
    args = ap.parse_args()

    if args.seeds is None:
        cases = GRADER_CASES
    else:
        t = 240.0 if args.rows >= 8 else 120.0
        cases = [(args.rows, args.cols, args.imb, args.slip, s, t) for s in parse_seeds(args.seeds)]

    suite = load_suite()
    results = []
    pending = list(cases)
    port = 8100
    while pending:
        batch = []
        for _ in range(min(args.parallel, len(pending))):
            batch.append(start_run(pending.pop(0), port, args.config, args.video, args.verbose))
            port += 1
        for run in batch:
            score, fb = finish_run(run, suite)
            results.append((run["case"], score))
            rows, cols, imb, slip, seed, _ = run["case"]
            print("%dx%d seed %-5d imb %+.2f slip %.2f -> %5.1f" % (rows, cols, seed, imb, slip, score))
            print("    " + (fb if args.full else summarize(fb)))
    mean = sum(s for _, s in results) / max(1, len(results))
    print("\nMEAN SCORE: %.1f over %d runs (min %.1f)" % (mean, len(results), min(s for _, s in results)))


if __name__ == "__main__":
    main()
