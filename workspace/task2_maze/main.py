# =========================================================================
# UCT Micromouse - Task 2: "Search, Celebrate & Sprint"
# =========================================================================
# 1. Explore from (0,0) with floodfill toward unresolved 2x2 candidates
#    until the target room is identified and entered.
# 2. 360 degree clockwise pirouette inside the room.
# 3. Return to (0,0) (floodfill, which also probes possible shortcuts),
#    face East and stop.
# 4. Pause 3 s, then sprint along the fastest known path into the room.
#
# Modules: config.py (constants), motion.py (control + estimation),
#          maze.py (map, room detection, planning).
# =========================================================================

import uct_mouse
from config import *
import maze as mz
from motion import Robot, cell_centre, setup_polarity, DIR_UX, DIR_UY


def log(msg):
    if VERBOSE:
        print(msg)


def maze_dimensions():
    try:
        dims = uct_mouse.get_maze_dimensions()
        rows = int(dims[0])
        cols = int(dims[1])
        if rows >= 2 and cols >= 2:
            return rows, cols
    except Exception:
        pass
    return DEFAULT_ROWS, DEFAULT_COLS


class MazeSolver:
    def __init__(self, robot, rows, cols):
        self.robot = robot
        self.map = mz.MazeMap(rows, cols, START_WALL_PRIOR)
        self.x = 0
        self.y = 0
        self.dir = mz.EAST

    # ------------------------------------------------------------------
    # Perception
    # ------------------------------------------------------------------
    def localise(self):
        """While stopped, pin the position estimate with every ToF that sees a
        mapped wall: the front reading fixes the along-track position, the
        side readings the lateral position (walls may be a cell or more away)."""
        r = self.robot
        d = self.dir
        checks = ((d, r.tof_c, r.front_stop), ((d + 3) % 4, r.tof_l, r.side_ref),
                  ((d + 1) % 4, r.tof_r, r.side_ref))
        for c in checks:
            if c[1] < LOCALISE_RANGE:
                wall = self.wall_along(self.x, self.y, c[0])
                if wall is not None:
                    r.localise_axis(c[0], wall, c[1], c[2])

    def record(self, left_wall, right_wall, front_from_centre, weight=1):
        """Record the walls of the current cell given side wall flags and the
        front reading as it would be seen from the cell centre. `weight` is
        how many votes the observation gets in the map."""
        r = self.robot
        ahead = front_from_centre - r.front_stop          # distance to the first wall ahead
        n_open = int(ahead / CELL_M + 0.5)
        seen = True
        if front_from_centre >= TOF_MAX_M - 0.1 or n_open > FRONT_LOOKAHEAD_CELLS:
            n_open = FRONT_LOOKAHEAD_CELLS
            seen = False
        if n_open < 0:
            n_open = 0
        self.map.record_view(self.x, self.y, self.dir, left_wall, right_wall, n_open, seen, weight)

    def observe(self):
        """Classify walls around the current cell (mouse stopped at centre),
        then re-localise against the updated map."""
        r = self.robot
        r.sample_still(5)
        log("stop (%d,%d) d%d: L=%.3f C=%.3f R=%.3f" % (self.x, self.y, self.dir, r.tof_l, r.tof_c, r.tof_r))
        self.record(r.left_wall(), r.right_wall(), r.tof_c, STILL_WEIGHT)
        self.localise()

    # ------------------------------------------------------------------
    # Movement in maze coordinates
    # ------------------------------------------------------------------
    def step_to(self, direction, n_cells, speed):
        """Pivot to face `direction`, then drive n_cells and stop at the centre."""
        r = self.robot
        log("(%d,%d) -> dir %d x%d  L=%.3f C=%.3f R=%.3f lat=%.3f" % (
            self.x, self.y, direction, n_cells, r.tof_l, r.tof_c, r.tof_r, r.lat))
        r.face(direction)
        self.dir = direction
        wall = self.wall_along(self.x, self.y, direction)
        self.x += mz.DX[direction] * n_cells
        self.y += mz.DY[direction] * n_cells
        ex, ey = cell_centre(self.x, self.y)
        r.drive_to(ex, ey, direction, speed, wall_along=wall)

    def wall_along(self, x, y, d):
        """Along-axis position of the first mapped wall ahead of cell (x, y) in
        direction d, or None if an unknown side comes first / nothing within
        reach of the front sensor."""
        cx = x
        cy = y
        for _ in range(4):
            s = self.map.get(cx, cy, d)
            if s == mz.WALL:
                bx, by = cell_centre(cx, cy)
                bx += 0.5 * CELL_M * DIR_UX[d]
                by += 0.5 * CELL_M * DIR_UY[d]
                return bx * DIR_UX[d] + by * DIR_UY[d]
            if s != mz.OPEN:
                return None
            cx += mz.DX[d]
            cy += mz.DY[d]
        return None

    def straight_run(self, d, goals):
        """How many cells to drive in direction d before stopping: keep going
        while the next cell is already mapped and floodfill would carry on
        straight through it (re-observing a mapped cell adds nothing)."""
        m = self.map
        n = 1
        cx = self.x + mz.DX[d]
        cy = self.y + mz.DY[d]
        while m.is_visited(cx, cy):
            if m.next_direction(cx, cy, d, goals, True) != d:
                break
            cx += mz.DX[d]
            cy += mz.DY[d]
            n += 1
        return n

    def navigate(self, goal_fn, done_fn, max_moves=400):
        """Floodfill (optimistic about unknown walls), re-planning after every
        cell. The mouse only stops where it must turn (or has arrived):
        straight on, it maps each new cell on the move and keeps driving.
        Returns True when done_fn() is satisfied."""
        moves = 0
        while moves < max_moves:
            self.observe()
            self.map.find_room()
            if done_fn():
                return True
            d = self.map.next_direction(self.x, self.y, self.dir, goal_fn(), True)
            if d is None:
                return False
            self.drive_exploring(d, goal_fn, done_fn)
            moves += 1
        return False

    def drive_exploring(self, d, goal_fn, done_fn):
        """Drive in direction d through as many cells as floodfill keeps going
        straight, mapping unvisited cells on the move.

        For each cell being entered, the side sensors (SIDE_SENSOR_X ahead of
        the axle) are sampled while they pass through the cell's middle, and
        the front sensor gives the wall ahead. DECIDE_BEFORE_CENTRE short of
        the cell centre the cell is recorded and floodfill is asked for the
        next move: straight on extends the drive by a cell without slowing,
        anything else lets the drive brake to a stop at this centre.
        """
        r = self.robot
        m = self.map
        r.face(d)
        self.dir = d
        ux = DIR_UX[d]
        uy = DIR_UY[d]
        st = [self.x + mz.DX[d], self.y + mz.DY[d], 0, 0, 0, 0.0, False]
        # st = [cell x, cell y, samples, left votes, right votes, front sum, decided]

        def on_tick():
            cx, cy = cell_centre(st[0], st[1])
            rel = (r.px - cx) * ux + (r.py - cy) * uy       # axle relative to the centre
            if SAMPLE_FROM <= rel <= SAMPLE_TO:
                st[2] += 1
                if r.left_wall():
                    st[3] += 1
                if r.right_wall():
                    st[4] += 1
                st[5] += r.tof_c + rel                       # reading as seen from the centre
            if st[6] or rel < -DECIDE_BEFORE_CENTRE:
                return None
            st[6] = True
            self.x = st[0]
            self.y = st[1]
            # on the robot the wide ToF beams misread openings while moving,
            # so an unvisited cell is left unmapped here and the drive stops
            # in it to look while stationary
            if MAP_ON_THE_MOVE and not m.is_visited(self.x, self.y) and st[2] >= 3:
                n = st[2]
                log("  moving (%d,%d) d%d: L wall %d/%d  R wall %d/%d  front %.3f" % (
                    self.x, self.y, d, st[3], n, st[4], n, st[5] / n))
                self.record(st[3] * 2 > n, st[4] * 2 > n, st[5] / n)
            m.find_room()
            if done_fn():
                return None
            nd = m.next_direction(self.x, self.y, d, goal_fn(), True)
            if nd != d or m.get(self.x, self.y, d) != mz.OPEN or not m.is_visited(self.x, self.y):
                return None                                  # stop at this centre
            st[0] += mz.DX[d]
            st[1] += mz.DY[d]
            st[2] = 0
            st[3] = 0
            st[4] = 0
            st[5] = 0.0
            st[6] = False
            nx, ny = cell_centre(st[0], st[1])
            return (nx, ny, self.wall_along(self.x, self.y, d))

        ex, ey = cell_centre(st[0], st[1])
        r.drive_to(ex, ey, d, EXPLORE_SPEED, wall_along=self.wall_along(self.x, self.y, d),
                   on_tick=on_tick)
        self.x = st[0]
        self.y = st[1]

    def follow_path(self, headings, v_max, accel, decel):
        """Drive a known path without stopping: straights are merged into one
        speed profile each, and every corner is a smooth quarter-circle turn
        from the boundary entering the turn cell to the boundary leaving it."""
        r = self.robot
        segs = mz.segments(headings)
        if not segs:
            return
        log("path %s" % (segs,))
        r.face(segs[0][0])
        self.dir = segs[0][0]
        r.sample_still(3)
        self.localise()
        half = CELL_M / 2.0
        for i in range(len(segs)):
            h = segs[i][0]
            n = segs[i][1]
            wall = self.wall_along(self.x, self.y, h)
            self.x += mz.DX[h] * n
            self.y += mz.DY[h] * n
            self.dir = h
            cx, cy = cell_centre(self.x, self.y)
            if i == len(segs) - 1:
                r.drive_to(cx, cy, h, v_max, 0.0, 0.0, accel, decel, wall)
                break
            # straight up to the entry boundary of the turn cell (may be zero
            # length when two turns follow each other), then the arc
            ex = cx - half * DIR_UX[h]
            ey = cy - half * DIR_UY[h]
            if (ex - r.px) * DIR_UX[h] + (ey - r.py) * DIR_UY[h] > 0.005:
                r.drive_to(ex, ey, h, v_max, ARC_SPEED, half, accel, decel, wall)
            r.arc(cx, cy, h, segs[i + 1][0], ARC_SPEED)

    # ------------------------------------------------------------------
    # Mission phases
    # ------------------------------------------------------------------
    def explore(self):
        m = self.map
        return self.navigate(m.exploration_goals,
                             lambda: m.room is not None and m.in_room(self.x, self.y))

    def return_to_start(self):
        """Head home. While a route through unexplored cells could still be
        shorter than the best known one, step through it with floodfill (this
        also improves the map for the sprint); once the known route is as
        short as any possible route, drive it smoothly."""
        m = self.map
        home = [(0, 0)]
        moves = 0
        while not (self.x == 0 and self.y == 0) and moves < 400:
            here = m.index(self.x, self.y)
            known = m.floodfill(home, False)[here]
            best = m.floodfill(home, True)[here]
            if known == best:
                path = m.fastest_path((self.x, self.y), self.dir, home, TURN_COST_CELLS)
                if path:
                    self.follow_path(path, RETURN_SPEED, ACCEL, DECEL)
                break
            d = m.next_direction(self.x, self.y, self.dir, home, True)
            if d is None:
                return False
            self.step_to(d, self.straight_run(d, home), EXPLORE_SPEED)
            self.observe()
            moves += 1
        self.robot.face(mz.EAST)
        self.dir = mz.EAST
        return self.x == 0 and self.y == 0

    def sprint(self):
        m = self.map
        headings = m.fastest_path((0, 0), mz.EAST, m.room_cells(), TURN_COST_CELLS)
        if headings is None:
            return False
        self.follow_path(headings, SPRINT_SPEED, SPRINT_ACCEL, SPRINT_DECEL)
        return True


def run_mission(rows, cols):
    """One complete four-phase run from the start cell."""
    robot = Robot()
    robot.calibrate_gyro()
    robot.calibrate_in_start_cell()
    solver = MazeSolver(robot, rows, cols)
    log("maze %dx%d side_ref=%.3f front_stop=%.3f" % (rows, cols, robot.side_ref, robot.front_stop))

    # Phase 1: explore until inside the identified target room
    uct_mouse.display_text(1, "EXPLORE")
    if not solver.explore():
        print("Exploration failed at (%d,%d) facing %d; its map:" % (solver.x, solver.y, solver.dir))
        print(solver.map.ascii(solver.x, solver.y))
        robot.stop()
        return

    # Phase 2: recognition pirouette
    uct_mouse.display_text(1, "PIROUETTE")
    robot.pirouette()
    robot.sample_still(5)
    solver.localise()

    # Phase 3: back to the start cell, facing East
    uct_mouse.display_text(1, "RETURN")
    if not solver.return_to_start():
        print("Return failed")
        robot.stop()
        return

    # Phase 4: pause (re-estimating gyro bias meanwhile), then sprint
    uct_mouse.display_text(1, "SPRINT")
    robot.calibrate_gyro(int((SPRINT_PAUSE_MS - 200) / LOOP_MS))
    solver.sprint()
    robot.stop()
    uct_mouse.display_text(1, "DONE")
    uct_mouse.delay_ms(500)


def wait_for_start():
    """SW1 (press and release), then a second to get the hand out of the way."""
    uct_mouse.display_text(1, "SW1: start")
    print("Press SW1 to start")
    while uct_mouse.get_button() == 0:
        uct_mouse.delay_ms(50)
    while uct_mouse.get_button() != 0:
        uct_mouse.delay_ms(50)
    uct_mouse.display_text(1, "RUNNING")
    uct_mouse.delay_ms(1000)


def main():
    if not uct_mouse.init():
        print("Initialization failed.")
        return
    setup_polarity()
    rows, cols = maze_dimensions()
    if not ON_HARDWARE:
        run_mission(rows, cols)
        return
    # On the robot: after a run, wait for SW1 again (no power-cycling needed)
    while True:
        wait_for_start()
        run_mission(rows, cols)
        uct_mouse.set_motors(0, 0)
        uct_mouse.display_text(1, "DONE")
        uct_mouse.delay_ms(1500)


if __name__ == "__main__":
    try:
        main()
    finally:
        uct_mouse.set_motors(0, 0)      # also on Ctrl-C or an error mid-run
