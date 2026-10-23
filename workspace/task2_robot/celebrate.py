# =========================================================================
# UCT Micromouse - Task 2 end-of-run celebration (PHYSICAL ROBOT version)
# =========================================================================
# After a successful sprint: the mouse "twerks" (wiggles its rear left and
# right while bobbing back and forth) on the spot while the OLED plays a
# little text animation, then it squares back up and shows the run times.
#
# The OLED can only show text through the uct_mouse API: 4 rows of up to 18
# characters (rows 1-4). Each animation frame is 4 strings, one per row.
# =========================================================================

import uct_mouse
from config import *

HALF_PI = 1.5707963267948966

# Animation frames: 4 rows each, at most 18 characters per row
FRAMES = (
    ("  (\\_/)    YAY!",
     "  (^o^)/",
     "  /| |",
     "  _/ \\_   ~~~~"),
    ("   (\\_/)  YAY!",
     " \\(^o^)",
     "    | |\\",
     "   _/ \\_  ~~~~"),
    ("  (\\_/)   SOLVED",
     "  (>o<)",
     " </| |\\>",
     "  _/ \\_   *  *"),
    ("   (\\_/)  SOLVED",
     "   (^-^)",
     "  <| |>",
     "  _/ \\_    *  *"),
)


def show(frame):
    for row in range(4):
        uct_mouse.display_text(row + 1, frame[row])


def party(robot, sprint_s=None, total_s=None):
    """Twerk on the spot with the OLED animation, then face the original
    direction again and show the times (seconds, or None to skip)."""
    uct_mouse.display_text(1, "MAZE SOLVED!")
    robot.stop(CELEBRATE_DELAY_MS)                      # a moment of stillness first
    hold = round(robot.heading / HALF_PI) * HALF_PI     # the maze axis it faces
    ticks = max(1, CELEBRATE_BEAT_MS // LOOP_MS)
    frame = 0
    next_frame = uct_mouse.get_ticks_ms()
    for beat in range(CELEBRATE_BEATS):
        # the animation runs on its own clock, slower than the wiggles
        if uct_mouse.get_ticks_ms() >= next_frame:
            show(FRAMES[frame % len(FRAMES)])
            frame += 1
            next_frame += CELEBRATE_FRAME_MS
        # one beat: rear swings one way while the mouse bobs backwards, the
        # next beat the other way while it bobs forwards - nets out to zero
        sgn = 1.0 if beat % 2 == 0 else -1.0
        for _ in range(ticks):
            robot.tick(-sgn * TWERK_V, sgn * TWERK_W)
    robot.brake()
    robot.stop()
    robot.turn_to(hold)
    uct_mouse.display_text(1, "MAZE SOLVED!")
    uct_mouse.display_text(2, "Sprint %.1f s" % sprint_s if sprint_s is not None else "")
    uct_mouse.display_text(3, "Total  %.0f s" % total_s if total_s is not None else "")
    uct_mouse.display_text(4, "  (\\_/) (^o^)")
