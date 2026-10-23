"""Run the calibrate.py routines against the simulator, where the true chassis
values are known (tools/simulation_config.json: 4400 ticks/m, half track
0.054 m, deadband 28 PWM, side ToF 0.040 m off-centre -> side_ref 0.060,
front ToF 0.065 m ahead -> front_stop 0.035, side ToFs 0.050 m ahead of the
axle), to check the routines before using them on the robot.

Usage: python tests/test_calibrate.py [routine ...]
       (default: sensors timing straight spin motors edge)
"""
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
TASK_DIR = os.path.dirname(HERE)
REPO = os.path.abspath(os.path.join(TASK_DIR, "..", ".."))
SIM = os.path.join(REPO, "tools", "physics_sim.py")
sys.path.insert(0, HERE)
from test_sim_run import wait_ready  # noqa: E402

# routine -> (map, seed, python call)
CASES = {
    "sensors": ("random", 42, "calibrate.sensors(seconds=3)"),
    "tof": ("random", 42, "calibrate.tof()"),
    "timing": ("empty", 1, "calibrate.timing()"),
    "straight": ("empty", 1, "calibrate.straight(5)"),
    "spin": ("empty", 1, "calibrate.spin(3)"),
    "motors": ("empty", 1, "calibrate.motors()"),
    # seed 1: cell (1,0) has no North wall, so the left wall ends at x = 0.2 m
    "edge": ("random", 1, "calibrate.edge()"),
}


def run(name, port):
    mp, seed, call = CASES[name]
    sim = subprocess.Popen([sys.executable, "-u", SIM, "--headless", "--map", mp, "--seed", str(seed),
                            "--imbalance", "0.04", "--slip", "0.02", "--port", str(port)],
                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    wait_ready(sim)
    env = dict(os.environ, GRADESCOPE_AUTOGRADER="1", UCT_MICROMOUSE_PORT=str(port))
    code = "import sys; sys.path.insert(0, %r); import calibrate; %s" % (TASK_DIR, call)
    out = subprocess.run([sys.executable, "-c", code], cwd=TASK_DIR, env=env,
                         capture_output=True, text=True, timeout=300)
    sim.terminate()
    lines = [l for l in out.stdout.splitlines() if not l.startswith("[")]
    print("=== %s ===" % name)
    print("\n".join(lines[-14:]))
    if out.returncode != 0:
        print(out.stderr[-1500:])


def main():
    names = sys.argv[1:] or list(CASES)
    for i, n in enumerate(names):
        run(n, 8500 + i)


if __name__ == "__main__":
    main()
