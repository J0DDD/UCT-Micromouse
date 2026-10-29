# =========================================================================
# UCT Micromouse - Maze map, target-room detection and path planning
# =========================================================================
# Pure logic: no hardware calls, so it can be unit-tested offline.
#
# Coordinates: cell (x, y), x = column (East +), y = row (North +), start (0,0).
# Directions:  0 = North, 1 = East, 2 = South, 3 = West.
# Each cell side is UNKNOWN, WALL or OPEN; the shared side of two neighbouring
# cells is always written to both so the map stays consistent.
# =========================================================================

NORTH = 0
EAST = 1
SOUTH = 2
WEST = 3
DX = (0, 1, 0, -1)
DY = (1, 0, -1, 0)

UNKNOWN = 0
WALL = 1
OPEN = 2

INF = 30000
FIXED_SCORE = 100   # evidence score of sides fixed by the rules (start cell)
MAX_SCORE = 6       # cap on accumulated evidence, so a side can still be corrected


class MazeMap:
    def __init__(self, rows, cols, start_prior=1):
        """start_prior: evidence for the start cell's North wall and East
        opening, which the rules guarantee. FIXED_SCORE makes them permanent
        (physical maze); the default single vote lets observations overrule
        it (the simulator's generator sometimes removes that wall)."""
        self.rows = rows
        self.cols = cols
        self.n = rows * cols
        self.sides = bytearray(self.n * 4)
        # Evidence per side: + for wall sightings, - for open ones (stationary
        # observations count double). The state is the sign, so a single bad
        # reading cannot flip a side that earlier readings agreed on. Sides
        # fixed by the rules get a score no observation can overturn.
        self.score = [0] * (self.n * 4)
        self.visited = bytearray(self.n)
        self.dist = [INF] * self.n          # reusable floodfill buffer
        self.queue = [0] * self.n           # reusable BFS queue
        self.room = None                    # bottom-left (x, y) of the confirmed 2x2 room

        for x in range(cols):
            self._set_side(x, 0, SOUTH, WALL)
            self._set_side(x, rows - 1, NORTH, WALL)
        for y in range(rows):
            self._set_side(0, y, WEST, WALL)
            self._set_side(cols - 1, y, EAST, WALL)
        # Start cell is boxed in on three sides and opens East.
        self.set_wall(0, 0, NORTH, WALL, start_prior)
        self.set_wall(0, 0, EAST, OPEN, start_prior)

    # ------------------------------------------------------------------
    # Basic access
    # ------------------------------------------------------------------
    def inside(self, x, y):
        return 0 <= x < self.cols and 0 <= y < self.rows

    def index(self, x, y):
        return y * self.cols + x

    def get(self, x, y, d):
        return self.sides[(y * self.cols + x) * 4 + d]

    def _set_side(self, x, y, d, state):
        self.sides[(y * self.cols + x) * 4 + d] = state

    def set_wall(self, x, y, d, state, weight=1):
        """Add one observation of a cell side (shared with its neighbour):
        `weight` votes for WALL or OPEN. Outer boundary walls are fixed and
        never changed; other sides take the sign of their accumulated votes."""
        nx = x + DX[d]
        ny = y + DY[d]
        if not self.inside(nx, ny):
            return
        i = (y * self.cols + x) * 4 + d
        j = (ny * self.cols + nx) * 4 + (d + 2) % 4
        s = self.score[i]
        if -FIXED_SCORE < s < FIXED_SCORE:
            s += weight if state == WALL else -weight
            if weight < FIXED_SCORE:
                s = max(-MAX_SCORE, min(MAX_SCORE, s))
        new = WALL if s > 0 else (OPEN if s < 0 else UNKNOWN)
        self.score[i] = s
        self.score[j] = s
        self._set_side(x, y, d, new)
        self._set_side(nx, ny, (d + 2) % 4, new)

    def mark_visited(self, x, y):
        self.visited[self.index(x, y)] = 1

    def is_visited(self, x, y):
        return self.visited[self.index(x, y)] == 1

    def can_move(self, x, y, d, optimistic):
        s = self.get(x, y, d)
        if s == OPEN:
            return True
        return optimistic and s == UNKNOWN

    # ------------------------------------------------------------------
    # Sensor integration
    # ------------------------------------------------------------------
    def record_view(self, x, y, heading, left_wall, right_wall, front_open_cells, front_wall_seen,
                    weight=1):
        """Apply one observation taken at the centre of (x, y) facing `heading`.

        front_open_cells: number of consecutive open sides straight ahead.
        front_wall_seen:  True if a wall was detected after those open cells
                          (False when the reading was beyond trusted range).
        weight:           votes this observation carries (stationary > moving);
                          sides further ahead than this cell get a single vote.
        """
        self.mark_visited(x, y)
        self.set_wall(x, y, (heading + 3) % 4, WALL if left_wall else OPEN, weight)
        self.set_wall(x, y, (heading + 1) % 4, WALL if right_wall else OPEN, weight)
        cx = x
        cy = y
        w = weight
        for _ in range(front_open_cells):
            if not self.inside(cx + DX[heading], cy + DY[heading]):
                break
            self.set_wall(cx, cy, heading, OPEN, w)
            cx += DX[heading]
            cy += DY[heading]
            w = 1
        if front_wall_seen:
            self.set_wall(cx, cy, heading, WALL, w)

    # ------------------------------------------------------------------
    # Target room detection
    # ------------------------------------------------------------------
    def _room_internal_sides(self, bx, by):
        """The four internal partitions of the 2x2 block with bottom-left (bx, by)."""
        return (self.get(bx, by, EAST), self.get(bx, by + 1, EAST),
                self.get(bx, by, NORTH), self.get(bx + 1, by, NORTH))

    def room_possible(self, bx, by):
        if bx == 0 and by == 0:
            return False                    # the start cell is never part of the room
        for s in self._room_internal_sides(bx, by):
            if s == WALL:
                return False
        return True

    def room_confirmed(self, bx, by):
        for s in self._room_internal_sides(bx, by):
            if s != OPEN:
                return False
        return True

    def perimeter_status(self, bx, by):
        """(open, unknown) counts over the 8 outer sides of a 2x2 block."""
        sides = (self.get(bx, by, SOUTH), self.get(bx + 1, by, SOUTH),
                 self.get(bx, by + 1, NORTH), self.get(bx + 1, by + 1, NORTH),
                 self.get(bx, by, WEST), self.get(bx, by + 1, WEST),
                 self.get(bx + 1, by, EAST), self.get(bx + 1, by + 1, EAST))
        n_open = 0
        n_unknown = 0
        for s in sides:
            if s == OPEN:
                n_open += 1
            elif s == UNKNOWN:
                n_unknown += 1
        return n_open, n_unknown

    def find_room(self):
        """Decide which 2x2 open block is the target room.

        Open 2x2 "plazas" can also appear by chance where random walls were
        removed (common on 8x8 arenas), so a fully open block is accepted
        immediately only when its perimeter is fully known and it has at most
        one entrance (the generator never braids the real room's perimeter).
        If every candidate block has been resolved and none is that enclosed,
        the most enclosed fully open block is taken instead.
        Returns the bottom-left (x, y) of the accepted room, or None.
        """
        if self.room is not None:
            return self.room
        unresolved = False
        best = None
        best_open = INF
        for i in range((self.rows - 1) * (self.cols - 1)):
            bx = i % (self.cols - 1)
            by = i // (self.cols - 1)
            if not self.room_possible(bx, by):
                continue
            if not self.room_confirmed(bx, by):
                unresolved = True
                continue
            n_open, n_unknown = self.perimeter_status(bx, by)
            if n_unknown > 0:
                unresolved = True
            if n_open + n_unknown <= 1:
                self.room = (bx, by)
                return self.room
            if n_open < best_open:
                best_open = n_open
                best = (bx, by)
        if not unresolved and best is not None:
            self.room = best
        return self.room

    def in_room(self, x, y):
        if self.room is None:
            return False
        return self.room[0] <= x <= self.room[0] + 1 and self.room[1] <= y <= self.room[1] + 1

    def room_cells(self):
        bx, by = self.room
        return [(bx, by), (bx + 1, by), (bx, by + 1), (bx + 1, by + 1)]

    def exploration_goals(self):
        """Cells worth driving to while searching for the room.

        Once the room is accepted, its cells are the goal. Otherwise the goal
        is every unvisited cell of every still-possible 2x2 block (visiting a
        cell resolves all four of its sides), so floodfill heads for the
        nearest unresolved candidate. Falls back to any unvisited cell.
        """
        if self.room is not None:
            return self.room_cells()
        goals = []
        for i in range((self.rows - 1) * (self.cols - 1)):
            bx = i % (self.cols - 1)
            by = i // (self.cols - 1)
            if self.room_possible(bx, by):
                self._add_unvisited_block_cells(goals, bx, by)
        if not goals:
            for i in range(self.n):
                if not self.visited[i]:
                    goals.append((i % self.cols, i // self.cols))
        return goals

    def _add_unvisited_block_cells(self, goals, bx, by):
        for c in ((bx, by), (bx + 1, by), (bx, by + 1), (bx + 1, by + 1)):
            if not self.is_visited(c[0], c[1]) and c not in goals:
                goals.append(c)

    # ------------------------------------------------------------------
    # Floodfill (BFS distance from the goal set)
    # ------------------------------------------------------------------
    def floodfill(self, goals, optimistic):
        dist = self.dist
        for i in range(self.n):
            dist[i] = INF
        q = self.queue
        head = 0
        tail = 0
        for g in goals:
            gi = self.index(g[0], g[1])
            if dist[gi] == INF:
                dist[gi] = 0
                q[tail] = gi
                tail += 1
        while head < tail:
            ci = q[head]
            head += 1
            x = ci % self.cols
            y = ci // self.cols
            nd = dist[ci] + 1
            for d in range(4):
                if self.can_move(x, y, d, optimistic):
                    ni = ci + DX[d] + DY[d] * self.cols
                    if dist[ni] == INF:
                        dist[ni] = nd
                        q[tail] = ni
                        tail += 1
        return dist

    def next_direction(self, x, y, heading, goals, optimistic=True):
        """Floodfill step: the direction toward the lowest-distance neighbour.
        Ties prefer going straight, then turning, then reversing."""
        dist = self.floodfill(goals, optimistic)
        here = dist[self.index(x, y)]
        if here == INF:
            return None
        best = None
        best_score = INF
        for turn in (0, 1, 3, 2):
            d = (heading + turn) % 4
            if not self.can_move(x, y, d, optimistic):
                continue
            nd = dist[self.index(x + DX[d], y + DY[d])]
            score = nd * 4 + (0 if turn == 0 else (1 if turn != 2 else 2))
            if nd < here and score < best_score:
                best_score = score
                best = d
        return best

    # ------------------------------------------------------------------
    # Time-optimal known path (for the sprint)
    # ------------------------------------------------------------------
    def fastest_path(self, start, start_heading, goals, turn_cost):
        """Dijkstra over (cell, heading) states using only known-open sides.

        Moving one cell costs 1, each 90 degree turn costs `turn_cost`, so long
        straights are preferred over zig-zags of the same length. Returns the
        list of headings to drive (one entry per cell moved), or None.
        """
        # Costs are whole tenths of a cell: small integers need no RAM of
        # their own in MicroPython, whereas every float is a separate heap
        # object (~16 bytes each) - with floats a 9x9 maze ran out of memory.
        n_states = self.n * 4
        turn = int(turn_cost * 10.0 + 0.5)
        cost = [INF * 10] * n_states
        prev = [-1] * n_states
        done = bytearray(n_states)
        goal_idx = []
        for g in goals:
            goal_idx.append(self.index(g[0], g[1]))
        s0 = self.index(start[0], start[1]) * 4 + start_heading
        cost[s0] = 0
        end_state = -1
        for _ in range(n_states):
            s = self._cheapest_open_state(cost, done)
            if s < 0:
                break
            done[s] = 1
            ci = s // 4
            if ci in goal_idx:
                end_state = s
                break
            self._relax_state(s, cost, prev, done, turn)
        if end_state < 0:
            return None
        headings = []
        s = end_state
        while prev[s] >= 0:
            p = prev[s]
            if p // 4 != s // 4:
                headings.append(s % 4)
            s = p
        headings.reverse()
        return headings

    def _cheapest_open_state(self, cost, done):
        best = -1
        best_cost = INF * 10
        for s in range(len(cost)):
            if not done[s] and cost[s] < best_cost:
                best_cost = cost[s]
                best = s
        return best

    def _relax_state(self, s, cost, prev, done, turn_cost):
        ci = s // 4
        h = s % 4
        x = ci % self.cols
        y = ci // self.cols
        # turn on the spot (left or right)
        for t in (1, 3):
            ns = ci * 4 + (h + t) % 4
            c = cost[s] + turn_cost
            if not done[ns] and c < cost[ns]:
                cost[ns] = c
                prev[ns] = s
        # move forward one cell
        if self.get(x, y, h) == OPEN:
            ns = (ci + DX[h] + DY[h] * self.cols) * 4 + h
            c = cost[s] + 10
            if not done[ns] and c < cost[ns]:
                cost[ns] = c
                prev[ns] = s


    def ascii(self, mx=-1, my=-1):
        """Map as text, North at the top: '---' / '|' known wall, '   ' / ' '
        known open, '...' / ':' unknown, M = mouse, . = visited cell."""
        lines = []
        for y in range(self.rows - 1, -1, -1):
            top = ""
            mid = ""
            for x in range(self.cols):
                n = self.get(x, y, NORTH)
                top += "+" + ("---" if n == WALL else ("   " if n == OPEN else "..."))
                w = self.get(x, y, WEST)
                mark = " M " if (x == mx and y == my) else (" . " if self.is_visited(x, y) else "   ")
                mid += ("|" if w == WALL else (" " if w == OPEN else ":")) + mark
            e = self.get(self.cols - 1, y, EAST)
            lines.append(top + "+")
            lines.append(mid + ("|" if e == WALL else ":"))
        lines.append("+---" * self.cols + "+")
        return chr(10).join(lines)


def segments(headings):
    """Collapse a per-cell heading list into [(heading, n_cells), ...]."""
    segs = []
    for h in headings:
        if segs and segs[-1][0] == h:
            segs[-1] = (h, segs[-1][1] + 1)
        else:
            segs.append((h, 1))
    return segs
