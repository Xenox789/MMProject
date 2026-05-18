"""
maze.py – Python port of the C++ Maze namespace from maze.cpp / maze.h

Wall bit-flags and Direction enum mirror the firmware exactly so test
results translate 1-to-1 to the real hardware.
"""

from collections import deque
from typing import List, Optional, Tuple

# ---------------------------------------------------------------------------
# Constants (mirrors config.h defaults)
# ---------------------------------------------------------------------------
DEFAULT_MAZE_SIZE = 16
DEFAULT_GOAL = (7, 7, 8, 8)  # (x1, y1, x2, y2) – 2×2 centre block

# Wall bit-flags
WALL_NORTH = 0x01
WALL_EAST  = 0x02
WALL_SOUTH = 0x04
WALL_WEST  = 0x08

# Direction enum values
NORTH = 0
EAST  = 1
SOUTH = 2
WEST  = 3

_DIR_NAMES = {NORTH: "N", EAST: "E", SOUTH: "S", WEST: "W"}

INF = 0xFFFF


# ---------------------------------------------------------------------------
# Helper functions (mirrors firmware direction helpers)
# ---------------------------------------------------------------------------

def turn_right(d: int) -> int:
    return (d + 1) % 4

def turn_left(d: int) -> int:
    return (d + 3) % 4

def opposite(d: int) -> int:
    return (d + 2) % 4

def next_cell(x: int, y: int, d: int) -> Tuple[int, int]:
    if d == NORTH: return x, y + 1
    if d == EAST:  return x + 1, y
    if d == SOUTH: return x, y - 1
    if d == WEST:  return x - 1, y
    return x, y

_WALL_FLAG = [WALL_NORTH, WALL_EAST, WALL_SOUTH, WALL_WEST]


# ---------------------------------------------------------------------------
# Maze class
# ---------------------------------------------------------------------------

class Maze:
    """
    A mutable 16×16 (or N×N) maze supporting:
      - wall placement / removal
      - flood-fill from one or two goal regions
      - best-path extraction
      - simulated mouse exploration (sensor-based wall discovery)
    """

    def __init__(self, size: int = DEFAULT_MAZE_SIZE,
                 goal: Tuple[int, int, int, int] = DEFAULT_GOAL):
        self.size = size
        self.goal = goal          # (x1, y1, x2, y2)

        # walls[x][y] – bit-mask of known walls for cell (x, y)
        self._walls: List[List[int]] = [[0] * size for _ in range(size)]
        # flood[x][y] – BFS distance to goal; INF = unreachable
        self._flood: List[List[int]] = [[INF] * size for _ in range(size)]

        self._apply_border_walls()

    # ------------------------------------------------------------------
    # Initialisation helpers
    # ------------------------------------------------------------------

    def _apply_border_walls(self):
        N = self.size
        for i in range(N):
            self._walls[i][0]   |= WALL_SOUTH
            self._walls[i][N-1] |= WALL_NORTH
            self._walls[0][i]   |= WALL_WEST
            self._walls[N-1][i] |= WALL_EAST

    def reset_walls(self):
        """Clear all internal walls, keep border walls."""
        N = self.size
        for x in range(N):
            for y in range(N):
                self._walls[x][y] = 0
        self._apply_border_walls()
        self._flood = [[INF] * N for _ in range(N)]

    # ------------------------------------------------------------------
    # Wall management
    # ------------------------------------------------------------------

    def set_wall(self, x: int, y: int, d: int):
        """Place a wall on side d of cell (x, y) and the neighbour's mirrored wall."""
        N = self.size
        if not (0 <= x < N and 0 <= y < N):
            return
        self._walls[x][y] |= _WALL_FLAG[d]
        nx, ny = next_cell(x, y, d)
        if 0 <= nx < N and 0 <= ny < N:
            self._walls[nx][ny] |= _WALL_FLAG[opposite(d)]

    def clear_wall(self, x: int, y: int, d: int):
        """Remove a wall on side d of cell (x, y) and the neighbour's mirrored wall."""
        N = self.size
        if not (0 <= x < N and 0 <= y < N):
            return
        self._walls[x][y] &= ~_WALL_FLAG[d]
        nx, ny = next_cell(x, y, d)
        if 0 <= nx < N and 0 <= ny < N:
            self._walls[nx][ny] &= ~_WALL_FLAG[opposite(d)]

    def toggle_wall(self, x: int, y: int, d: int):
        if self.has_wall(x, y, d):
            self.clear_wall(x, y, d)
        else:
            self.set_wall(x, y, d)

    def has_wall(self, x: int, y: int, d: int) -> bool:
        N = self.size
        if not (0 <= x < N and 0 <= y < N):
            return True   # out-of-bounds counts as wall
        return bool(self._walls[x][y] & _WALL_FLAG[d])

    def get_walls_raw(self, x: int, y: int) -> int:
        return self._walls[x][y]

    # ------------------------------------------------------------------
    # Flood-fill (direct port of firmware BFS)
    # ------------------------------------------------------------------

    def flood_fill(self, goal: Optional[Tuple[int, int, int, int]] = None):
        """
        BFS flood from the goal region(s).  After this call every cell
        holds the shortest-path distance (in cells) to the nearest goal.
        """
        if goal is None:
            goal = self.goal
        gx1, gy1, gx2, gy2 = goal

        N = self.size
        flood = self._flood
        for x in range(N):
            for y in range(N):
                flood[x][y] = INF

        queue: deque[Tuple[int, int]] = deque()
        for gx in range(gx1, gx2 + 1):
            for gy in range(gy1, gy2 + 1):
                if 0 <= gx < N and 0 <= gy < N:
                    flood[gx][gy] = 0
                    queue.append((gx, gy))

        while queue:
            cx, cy = queue.popleft()
            new_dist = flood[cx][cy] + 1
            for d in range(4):
                if self.has_wall(cx, cy, d):
                    continue
                nx, ny = next_cell(cx, cy, d)
                if not (0 <= nx < N and 0 <= ny < N):
                    continue
                if new_dist < flood[nx][ny]:
                    flood[nx][ny] = new_dist
                    queue.append((nx, ny))

    def flood_fill_to_start(self):
        self.flood_fill((0, 0, 0, 0))

    def get_flood(self, x: int, y: int) -> int:
        N = self.size
        if not (0 <= x < N and 0 <= y < N):
            return INF
        return self._flood[x][y]

    # ------------------------------------------------------------------
    # Navigation helpers
    # ------------------------------------------------------------------

    def best_direction(self, x: int, y: int, heading: int) -> Optional[int]:
        """
        Return the best open cardinal direction to move from (x, y),
        or None if every neighbour is walled off.
        Mirrors firmware priority: forward > right > left > back.
        """
        check_order = [
            heading,
            turn_right(heading),
            turn_left(heading),
            opposite(heading),
        ]
        best_val = INF
        best_dir = None
        for d in check_order:
            if self.has_wall(x, y, d):
                continue
            nx, ny = next_cell(x, y, d)
            val = self.get_flood(nx, ny)
            if val < best_val:
                best_val = val
                best_dir = d
        return best_dir

    def best_path(self, start_x: int = 0, start_y: int = 0,
                  start_heading: int = NORTH
                  ) -> List[Tuple[int, int]]:
        """
        Greedily follow flood-fill values from start to goal.
        Returns a list of (x, y) cells in order (includes start + goal).
        Returns empty list if no path exists.
        """
        N = self.size
        x, y, heading = start_x, start_y, start_heading
        path = [(x, y)]
        visited = set()
        visited.add((x, y))

        gx1, gy1, gx2, gy2 = self.goal
        def at_goal(cx, cy):
            return gx1 <= cx <= gx2 and gy1 <= cy <= gy2

        while not at_goal(x, y):
            d = self.best_direction(x, y, heading)
            if d is None:
                break   # every neighbour is walled off
            if self.has_wall(x, y, d):
                break   # safety: should not happen after best_direction
            nx, ny = next_cell(x, y, d)
            if (nx, ny) in visited:
                break   # stuck in a loop
            if not (0 <= nx < N and 0 <= ny < N):
                break
            x, y = nx, ny
            heading = d
            path.append((x, y))
            visited.add((x, y))
            if len(path) > N * N:
                break   # safety guard

        return path if at_goal(x, y) else []

    # ------------------------------------------------------------------
    # Simulated exploration
    # ------------------------------------------------------------------

    def simulate_exploration(self, true_maze: "Maze",
                             start_x: int = 0, start_y: int = 0,
                             start_heading: int = NORTH
                             ) -> Tuple[List[Tuple[int, int]], int]:
        """
        Simulate a micromouse exploring *true_maze* from scratch.

        The mouse starts with no knowledge of internal walls (only borders),
        discovers walls by "sensing" the true maze at each step, re-runs
        flood fill, and navigates greedily – exactly as the firmware does.

        Returns:
            path          – list of (x, y) cells visited during exploration
            flood_calls   – number of flood-fill re-runs (higher = harder)
        """
        self.reset_walls()
        self.flood_fill()

        x, y, heading = start_x, start_y, start_heading
        path = [(x, y)]
        flood_calls = 0
        N = self.size
        gx1, gy1, gx2, gy2 = self.goal

        def at_goal(cx, cy):
            return gx1 <= cx <= gx2 and gy1 <= cy <= gy2

        max_steps = N * N * 4
        for _ in range(max_steps):
            if at_goal(x, y):
                break

            # ---- sense walls from true_maze ----
            walls_changed = False
            for d in range(4):
                if true_maze.has_wall(x, y, d) and not self.has_wall(x, y, d):
                    self.set_wall(x, y, d)
                    walls_changed = True

            if walls_changed:
                self.flood_fill()
                flood_calls += 1

            # ---- move ----
            d = self.best_direction(x, y, heading)
            if self.has_wall(x, y, d):
                break   # completely blocked (shouldn't happen after flood)
            nx, ny = next_cell(x, y, d)
            if not (0 <= nx < N and 0 <= ny < N):
                break
            x, y = nx, ny
            heading = d
            path.append((x, y))

        return path, flood_calls

    # ------------------------------------------------------------------
    # Serialisation – simple list of (x, y, dir) wall tuples
    # ------------------------------------------------------------------

    def export_walls(self) -> List[Tuple[int, int, int]]:
        """Return all non-border walls as (x, y, direction) tuples."""
        result = []
        N = self.size
        seen = set()
        for x in range(N):
            for y in range(N):
                for d in [NORTH, EAST]:   # only need two sides to avoid duplicates
                    if self.has_wall(x, y, d):
                        nx, ny = next_cell(x, y, d)
                        # Skip if this is a border wall
                        if 0 <= nx < N and 0 <= ny < N:
                            key = (min(x, nx), min(y, ny), max(x, nx), max(y, ny))
                            if key not in seen:
                                seen.add(key)
                                result.append((x, y, d))
        return result

    def import_walls(self, wall_list: List[Tuple[int, int, int]],
                     reset_first: bool = True):
        """Load walls from a list of (x, y, direction) tuples."""
        if reset_first:
            self.reset_walls()
        for x, y, d in wall_list:
            self.set_wall(x, y, d)

    # ------------------------------------------------------------------
    # ASCII debug print (same layout as firmware printMaze)
    # ------------------------------------------------------------------

    def print_maze(self, show_flood: bool = True, path: Optional[List[Tuple[int, int]]] = None):
        N = self.size
        path_set = set(path) if path else set()
        gx1, gy1, gx2, gy2 = self.goal

        print("=" * (N * 4 + 1))
        for y in range(N - 1, -1, -1):
            # North wall row
            row = "+"
            for x in range(N):
                row += "---+" if self.has_wall(x, y, NORTH) else "   +"
            print(row)
            # West walls + cell content
            row = ""
            for x in range(N):
                row += "|" if self.has_wall(x, y, WEST) else " "
                f = self._flood[x][y]
                if (x, y) in path_set:
                    row += " * "
                elif gx1 <= x <= gx2 and gy1 <= y <= gy2:
                    row += " G "
                elif show_flood and f < INF:
                    row += f"{f:3d}"
                else:
                    row += "   "
            row += "|"
            print(row)
        # Bottom border
        print("+" + "---+" * N)
