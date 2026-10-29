# =========================================================================
# UCT Micromouse - Task 2 end-of-run celebration (PHYSICAL ROBOT version)
# =========================================================================
# After a successful sprint the mouse stands still and the OLED plays a short
# victory screen (~6 s): a little mouse runs to the cheese, eats it, then stars
# twinkle around MAZE SOLVED! and the mission time while the LEDs flash.
#
# The OLED can only show text through the uct_mouse API: 4 rows of up to 18
# characters (rows 1-4), refreshed ~10 times a second.
# Preview without a run: calibrate.celebration()
# =========================================================================

import uct_mouse
from config import *


def show(row, text):
    uct_mouse.display_text(row, text)


def leds(a, b):
    try:
        uct_mouse.set_led(0, a)
        uct_mouse.set_led(1, b)
    except Exception:
        pass                                # (no LEDs in the simulator)


def celebrate(secs):
    """Victory screen after the sprint (motors are already off, so it
    doesn't touch the run itself). The OLED has 4 rows of 18 characters and
    refreshes ~10 times a second. ~6 s:
      1. a little mouse <:3)~ runs across row 2 to the cheese [#]
      2. "nom nom!"
      3. twinkling stars around MAZE SOLVED! + the mission time, LEDs flashing
    The final screen stays until the next run (SW1)."""
    try:
        mouse = "<:3)~"
        cheese = "[#]"
        show(1, "")
        show(3, "")
        show(4, "")
        for x in range(18 - len(mouse), len(cheese), -1):      # 1. the run
            show(2, cheese + " " * (x - len(cheese)) + mouse)
            leds(x % 2, (x + 1) % 2)
            uct_mouse.delay_ms(110)
        for _ in range(3):                                      # 2. eating
            show(2, cheese + mouse + " nom nom!")
            uct_mouse.delay_ms(250)
            show(2, "[" + mouse + "   nom nom!")
            uct_mouse.delay_ms(250)
        stars = ("*  .   *  +  .  *",                           # 3. party
                 " .  *  +  . *  . ",
                 "+ .  *   .  +  * ",
                 " *  +  .  *  .  +")
        show(2, "  MAZE SOLVED!")
        show(4, "  time %.1f s" % secs)
        for i in range(24):
            show(1, stars[i % 4])
            show(3, stars[(i + 2) % 4])
            leds(i % 2, (i + 1) % 2)
            uct_mouse.delay_ms(200)
        show(1, "*  YOU DID IT!  *")
        show(3, "    <:3)~ [#]")
    finally:
        leds(0, 0)


def party(robot, times):
    """After the sprint: a short pause, then the victory screen with the
    whole mission time. times: {"total", "explore", "return", "sprint"} in s."""
    show(1, "MAZE SOLVED!")
    robot.stop(CELEBRATE_DELAY_MS)          # a moment of stillness first
    celebrate(times["total"])
