"""Motion-only client used by test_motion.py: a fixed script of straights and
pivots in the open arena (no maze logic), so the controllers can be tuned
against the simulator's true pose."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import uct_mouse                      # noqa: E402
from config import CELL_M, EXPLORE_SPEED, SPRINT_SPEED   # noqa: E402
from motion import Robot, cell_centre  # noqa: E402

SPEED = SPRINT_SPEED if os.environ.get("PROBE_FAST") == "1" else EXPLORE_SPEED


def main():
    uct_mouse.init()
    r = Robot()
    r.calibrate_gyro()
    # square of 3-cell sides, then single-cell steps with turns both ways
    x, y = 0, 0
    steps = ((1, 3), (0, 3), (3, 3), (2, 3), (1, 1), (0, 1), (1, 1), (2, 1), (1, 1), (0, 1))
    for d, n in steps:
        r.face(d)
        x += (0, 1, 0, -1)[d] * n
        y += (1, 0, -1, 0)[d] * n
        ex, ey = cell_centre(x, y)
        r.drive_to(ex, ey, d, SPEED)
    r.face(1)
    r.pirouette()
    uct_mouse.delay_ms(500)


if __name__ == "__main__":
    main()
