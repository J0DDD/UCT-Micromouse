"""Motion-control test bench: runs probe_motion.py in the empty arena and
reports turn overshoot / settling and straight-line heading / lateral /
stopping error, measured on the simulator's TRUE pose.

Usage: python tests/test_motion.py [--fast] [--seed N] [--imb 0.06] [--slip 0.04]
       python tests/test_motion.py --map random --seed 42   (statistics of a full maze run)
"""
import argparse
import json
import math
import os
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
TASK_DIR = os.path.dirname(HERE)
REPO = os.path.abspath(os.path.join(TASK_DIR, "..", ".."))
SIM = os.path.join(REPO, "tools", "physics_sim.py")


def wait_ready(sim, timeout=20.0):
    """Block until the simulator prints that it is listening (like grade_runner),
    then keep draining its output so it can never block on a full pipe."""
    import threading
    t0 = time.time()
    for line in sim.stdout:
        if "Waiting for student script" in line or time.time() - t0 > timeout:
            break
    threading.Thread(target=lambda: [None for _ in sim.stdout], daemon=True).start()


def run(args, port):
    traj = os.path.join(tempfile.gettempdir(), "motion_probe_%d.jsonl" % port)
    if os.path.exists(traj):
        os.remove(traj)
    sim = subprocess.Popen([sys.executable, "-u", SIM, "--headless", "--map", args.map, "--port", str(port),
                            "--seed", str(args.seed), "--imbalance", str(args.imb), "--slip", str(args.slip),
                            "--max-time", "120", "--trajectory-log", traj],
                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    wait_ready(sim)
    env = dict(os.environ, GRADESCOPE_AUTOGRADER="1", UCT_MICROMOUSE_PORT=str(port))
    if args.fast:
        env["PROBE_FAST"] = "1"
    client = os.path.join(TASK_DIR, "main.py") if args.map == "random" else os.path.join(HERE, "probe_motion.py")
    subprocess.run([sys.executable, client], env=env, cwd=TASK_DIR,
                   stdout=subprocess.DEVNULL)
    sim.wait(timeout=30)
    return [json.loads(l) for l in open(traj)][1:]


def phases(recs):
    """Split into ('turn'|'drive'|'idle', [records]) by the PWM pattern."""
    out = []
    for r in recs:
        if r["pwm_l"] * r["pwm_r"] < 0:
            kind = "turn"
        elif r["pwm_l"] == 0 and r["pwm_r"] == 0:
            kind = "idle"
        else:
            kind = "drive"
        if out and out[-1][0] == kind:
            out[-1][1].append(r)
        else:
            out.append((kind, [r]))
    # merge short blips (e.g. a single same-sign tick inside a turn) into neighbours
    merged = []
    for kind, rs in out:
        if merged and len(rs) <= 3 and kind != "idle":
            merged[-1][1].extend(rs)
        else:
            merged.append((kind, rs))
    return merged


def unwrap(recs):
    th = []
    prev = None
    acc = 0.0
    for r in recs:
        t = r["theta"]
        if prev is not None:
            d = (t - prev + math.pi) % (2 * math.pi) - math.pi
            acc += d
        else:
            acc = t
        prev = t
        th.append(math.degrees(acc))
    return th


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--fast", action="store_true")
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--imb", type=float, default=0.06)
    ap.add_argument("--slip", type=float, default=0.04)
    ap.add_argument("--map", default="empty", help="empty = scripted probe; random = full main.py maze run")
    ap.add_argument("--quiet", action="store_true", help="summary lines only")
    args = ap.parse_args()
    recs = run(args, 8400 + args.seed % 50)
    th = unwrap(recs)
    for i, r in enumerate(recs):
        r["th"] = th[i]

    ph = phases(recs)
    turns = []
    drives = []
    for i, (kind, rs) in enumerate(ph):
        if kind == "turn":
            # settle: include following idle samples
            tail = ph[i + 1][1] if i + 1 < len(ph) and ph[i + 1][0] == "idle" else []
            final = (tail[-1] if tail else rs[-1])["th"]
            target = round(final / 90.0) * 90.0
            start = rs[0]["th"]
            direction = 1.0 if target > start else -1.0
            over = max(0.0, max(direction * (r["th"] - target) for r in rs + tail))
            reversals = 0
            for a, b in zip(rs, rs[1:]):
                if (a["pwm_r"] > 0) != (b["pwm_r"] > 0):
                    reversals += 1
            turns.append((rs[-1]["time"] - rs[0]["time"], over, final - target, reversals, target - round(start / 90.0) * 90.0))
        elif kind == "drive" and len(rs) > 20:
            nominal = round(rs[0]["th"] / 90.0) * 90.0
            herr = [r["th"] - nominal for r in rs]
            skip = int(0.3 / 0.01)
            steady = herr[skip:] or herr
            rms = math.sqrt(sum(e * e for e in steady) / len(steady))
            horiz = abs(math.cos(math.radians(nominal))) > 0.5
            lat = [((r["y"] if horiz else r["x"]) - 0.1) for r in rs]
            lat = [l - round(l / 0.2) * 0.2 for l in lat]
            tail = ph[i + 1][1] if i + 1 < len(ph) and ph[i + 1][0] == "idle" else rs
            end = tail[-1]
            along = (end["x"] if horiz else end["y"]) - 0.1
            stop_err = along - round(along / 0.2) * 0.2
            drives.append((rs[-1]["time"] - rs[0]["time"], max(abs(e) for e in herr), rms,
                           max(abs(l) for l in lat), stop_err))

    if not args.quiet:
        print("TURNS  (dur s | overshoot deg | final err deg | reversals | size)")
        for t in turns:
            print("  %5.2f | %5.2f | %+5.2f | %d | %+.0f" % t)
        print("DRIVES (dur s | peak head err | steady rms | max lateral mm | stop err mm)")
        for d in drives:
            print("  %5.2f | %5.2f | %5.2f | %5.1f | %+5.1f" % (d[0], d[1], d[2], d[3] * 1000, d[4] * 1000))
    if turns:
        print("turn mean dur %.2f s, mean overshoot %.2f, max |final err| %.2f, reversals %d" % (
            sum(t[0] for t in turns) / len(turns), sum(t[1] for t in turns) / len(turns),
            max(abs(t[2]) for t in turns), sum(t[3] for t in turns)))
    if drives:
        print("drive max peak head %.2f, mean rms %.2f, max lateral %.1f mm, max |stop err| %.1f mm" % (
            max(d[1] for d in drives), sum(d[2] for d in drives) / len(drives),
            max(d[3] for d in drives) * 1000, max(abs(d[4]) for d in drives) * 1000))
    print("total time %.1f s" % recs[-1]["time"])


if __name__ == "__main__":
    main()
