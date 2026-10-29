# =========================================================================
# UCT Micromouse - Task 2 end-of-run celebration (PHYSICAL ROBOT version)
# =========================================================================
# After a successful sprint the mouse stands still and the OLED scrolls
# "MAZE SOLVED!" across the screen while confetti falls, then shows the
# times: whole run, exploration, return and sprint.
#
# The OLED can only show text through the uct_mouse API: 4 rows of up to 18
# characters (rows 1-4).
# Preview without a run: calibrate.celebration()
# =========================================================================

import uct_mouse
from config import *

W = 18                      # characters per OLED row
H = 4                       # OLED rows
CONFETTI = "*+.o'"
BANNER_ROW = 1


def banner_text(times):
    if times.get("sprint") is not None:
        return "MAZE SOLVED! SPRINT %.1f S" % times["sprint"]
    return "MAZE SOLVED!"


def banner_frame(k, msg):
    """Frame k: msg scrolls across the second row, and one new piece of
    confetti appears per frame and falls a row per frame."""
    grid = [[" "] * W for _ in range(H)]
    for i in range(max(0, k - H + 1), k + 1):
        grid[k - i][(i * 7 + 3) % W] = CONFETTI[i % len(CONFETTI)]
    x = W - k                               # enters on the right, leaves on the left
    for i in range(len(msg)):
        if 0 <= x + i < W:
            grid[BANNER_ROW][x + i] = msg[i]
    return ["".join(r) for r in grid]


def play_banner(times):
    msg = banner_text(times)
    for k in range(W + len(msg) + 1):
        frame = banner_frame(k, msg)
        for row in range(H):
            uct_mouse.display_text(row + 1, frame[row])
        uct_mouse.delay_ms(BANNER_FRAME_MS)
    uct_mouse.delay_ms(400)


def times_screen(times):
    """Seconds per phase; a phase that is missing (None) shows as --."""
    labels = (("Total", "total"), ("Explore", "explore"), ("Return", "return"), ("Sprint", "sprint"))
    for row in range(H):
        name, key = labels[row]
        t = times.get(key)
        uct_mouse.display_text(row + 1, "%-8s%7.1f s" % (name, t) if t is not None
                               else "%-8s     -- s" % name)


def party(robot, times):
    """After the sprint: a short pause, the banner, then the times.
    times: {"total", "explore", "return", "sprint"} in seconds."""
    uct_mouse.display_text(1, "MAZE SOLVED!")
    robot.stop(CELEBRATE_DELAY_MS)          # a moment of stillness first
    play_banner(times)
    times_screen(times)
