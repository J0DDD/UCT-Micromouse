"""Offline check of the mapping / planning logic (no physics, perfect motion).

Builds mazes with the simulator's own generator, "drives" a virtual mouse
cell-by-cell using the same observations the real sensors give (left, right,
front with look-ahead), and checks that exploration finds the real target
room, the return reaches (0,0), and the sprint path is valid and shortest.

Run:  python tests/test_maze_logic.py [n_seeds]
"""
import os
import sys
from collections import deque

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, os.path.join(HERE, "..", "..", "..", "tools"))

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
import physics_sim  # noqa: E402
import maze as mz   # noqa: E402

LOOKAHEAD = 2


def true_walls(rows, cols, seed):
    segs, target = physics_sim.generate_random_maze(rows, cols, 0.20, seed=seed)
    walls = set()
    for (x1, y1), (x2, y2) in segs:
        c1, r1 = round(x1 / 0.2), round(y1 / 0.2)
        if abs(x1 - x2) < 1e-9:      # vertical: West side of (c1, r1)
            walls.add((c1, r1, mz.WEST))
            walls.add((c1 - 1, r1, mz.EAST))
        else:                         # horizontal: South side of (c1, r1)
            walls.add((c1, r1, mz.SOUTH))
            walls.add((c1, r1 - 1, mz.NORTH))
    for x in range(cols):
        walls.add((x, 0, mz.SOUTH))
        walls.add((x, rows - 1, mz.NORTH))
    for y in range(rows):
        walls.add((0, y, mz.WEST))
        walls.add((cols - 1, y, mz.EAST))
    return walls, target


def observe(m, walls, x, y, h):
    left = (x, y, (h + 3) % 4) in walls
    right = (x, y, (h + 1) % 4) in walls
    n_open = 0
    cx, cy = x, y
    seen = False
    while n_open < LOOKAHEAD + 1:
        if (cx, cy, h) in walls:
            seen = True
            break
        n_open += 1
        cx += mz.DX[h]
        cy += mz.DY[h]
    if n_open > LOOKAHEAD:
        n_open, seen = LOOKAHEAD, False
    m.record_view(x, y, h, left, right, n_open, seen)


def true_shortest(walls, rows, cols, start, goals):
    q = deque([start])
    dist = {start: 0}
    while q:
        c = q.popleft()
        if c in goals:
            return dist[c]
        for d in range(4):
            if (c[0], c[1], d) not in walls:
                n = (c[0] + mz.DX[d], c[1] + mz.DY[d])
                if n not in dist:
                    dist[n] = dist[c] + 1
                    q.append(n)
    return None


def navigate(m, walls, x, y, h, goal_fn, done_fn, max_moves):
    moves = 0
    while moves < max_moves:
        observe(m, walls, x, y, h)
        m.find_room()
        if done_fn(x, y):
            return x, y, h, moves
        d = m.next_direction(x, y, h, goal_fn(), optimistic=True)
        if d is None:
            return x, y, h, -1
        h = d
        assert (x, y, h) not in walls, "drove into a wall"
        x += mz.DX[h]
        y += mz.DY[h]
        moves += 1
    return x, y, h, -1


def run(rows, cols, seed):
    walls, (tcx, tcy) = true_walls(rows, cols, seed)
    real_room = (tcx - 1, tcy - 1)
    m = mz.MazeMap(rows, cols)
    x, y, h, n_exp = navigate(m, walls, 0, 0, mz.EAST, m.exploration_goals,
                              lambda cx, cy: m.room is not None and m.in_room(cx, cy), 400)
    if n_exp < 0:
        return False, "exploration failed"
    if m.room != real_room:
        return False, "wrong room %s vs real %s" % (m.room, real_room)
    x, y, h, n_ret = navigate(m, walls, x, y, h, lambda: [(0, 0)],
                              lambda cx, cy: (cx, cy) == (0, 0), 400)
    if n_ret < 0:
        return False, "return failed"
    headings = m.fastest_path((0, 0), mz.EAST, m.room_cells(), 0.6)
    if headings is None:
        return False, "no sprint path"
    cx, cy = 0, 0
    for hd in headings:
        assert (cx, cy, hd) not in walls, "sprint path through a wall"
        cx += mz.DX[hd]
        cy += mz.DY[hd]
    if (cx, cy) not in m.room_cells():
        return False, "sprint does not end in room"
    best = true_shortest(walls, rows, cols, (0, 0), set(m.room_cells()))
    note = "explore=%d return=%d sprint=%d (true shortest %d, %d segs)" % (
        n_exp, n_ret, len(headings), best, len(mz.segments(headings)))
    return True, note


def main():
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 40
    fails = 0
    longer = 0
    for rows, cols in ((4, 6), (8, 8)):
        for seed in range(1, n + 1):
            ok, note = run(rows, cols, seed)
            if not ok:
                fails += 1
                print("FAIL %dx%d seed %d: %s" % (rows, cols, seed, note))
            elif "sprint=" in note:
                sp = int(note.split("sprint=")[1].split()[0])
                best = int(note.split("true shortest ")[1].split(",")[0])
                if sp > best:
                    longer += 1
    print("done: %d failures, %d sprints longer than the true shortest path, over %d mazes"
          % (fails, longer, 2 * n))


if __name__ == "__main__":
    main()
