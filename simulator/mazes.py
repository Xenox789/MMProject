"""
mazes.py – Pre-built maze definitions for the MicroMouse simulator.

Each maze is stored as a list of (x, y, direction) wall tuples plus
optional metadata.  Use load_preset(name) to get a configured Maze
object immediately.

Coordinate system (same as firmware):
  (0,0) = bottom-left start cell
  X grows right, Y grows up
  NORTH = +Y, EAST = +X, SOUTH = -Y, WEST = -X
"""

from maze import Maze, NORTH, EAST, SOUTH, WEST

# ---------------------------------------------------------------------------
# Helper to quickly describe walls in a human-friendly way
# ---------------------------------------------------------------------------

def _n(x, y): return (x, y, NORTH)
def _e(x, y): return (x, y, EAST)
def _s(x, y): return (x, y, SOUTH)
def _w(x, y): return (x, y, WEST)


# ---------------------------------------------------------------------------
# Preset catalogue
# ---------------------------------------------------------------------------

PRESETS: dict = {}


def _register(name: str, description: str, size: int,
               goal: tuple, walls: list):
    PRESETS[name] = dict(
        description=description,
        size=size,
        goal=goal,
        walls=walls,
    )


# ── 1. Empty maze ──────────────────────────────────────────────────────────
_register(
    name="empty",
    description="No internal walls – trivial straight-line path to centre",
    size=16,
    goal=(7, 7, 8, 8),
    walls=[],
)


# ── 2. Simple corridor ─────────────────────────────────────────────────────
# A single winding corridor across an 8×8 sub-maze so it stays readable.
_register(
    name="corridor",
    description="Single winding corridor – tests basic flood-fill path-following",
    size=8,
    goal=(3, 3, 4, 4),
    walls=[
        # Vertical divider with one gap
        _e(3, 0), _e(3, 1), _e(3, 2),           # wall east of col 3, rows 0-2
        _e(3, 4), _e(3, 5), _e(3, 6), _e(3, 7), # wall east of col 3, rows 4-7
        # Horizontal shelf forcing a detour
        _n(0, 3), _n(1, 3), _n(2, 3),
        _n(4, 4), _n(5, 4), _n(6, 4), _n(7, 4),
    ],
)


# ── 3. Spiral maze ─────────────────────────────────────────────────────────
# Spiral path (all 8×8, origin = bottom-left):
#   Enter ring-1 from south at col 1 → travel east along row 1 →
#   Gap in NE corner lets you enter ring-2 heading north along col 6 →
#   Turn west along row 6 (top of ring-2) →
#   Gap in NW lets you go south along col 1 (ring-2 west wall already open at y=6) →
#   ... eventually through inner rings to the goal centre.
_register(
    name="spiral",
    description="Inward spiral – mouse enters south and unwinds to centre",
    size=8,
    goal=(3, 3, 4, 4),
    walls=[
        # ── Ring 1 (outermost internal) – gap at SW (no west wall on row 1) ──
        # South boundary: north side of row 1 for cols 2-6
        _n(2,1),_n(3,1),_n(4,1),_n(5,1),_n(6,1),
        # East boundary: east side of col 6 for rows 2-5
        _e(6,2),_e(6,3),_e(6,4),_e(6,5),
        # North boundary: south side of row 6 for cols 1-5  (= north of row 5)
        _n(1,5),_n(2,5),_n(3,5),_n(4,5),_n(5,5),
        # West boundary: west side of col 1 for rows 2-5  (gap at row 6 NW)
        _w(1,2),_w(1,3),_w(1,4),_w(1,5),

        # ── Ring 2 – gap at NE (enter from _e(6,6) side, i.e. col 7 row 6) ──
        # South boundary: north side of row 2 for cols 2-5
        _n(2,2),_n(3,2),_n(4,2),_n(5,2),
        # East boundary: east side of col 5 for rows 3-4
        _e(5,3),_e(5,4),
        # North boundary: south side of row 5 for cols 2-4
        _n(2,4),_n(3,4),_n(4,4),
        # West boundary: west side of col 2 for rows 3-4
        _w(2,3),_w(2,4),
    ],
)


# ── 4. Maze with dead ends ─────────────────────────────────────────────────
_register(
    name="dead_ends",
    description="Multiple dead ends – tests that flood-fill avoids them",
    size=8,
    goal=(3, 3, 4, 4),
    walls=[
        # Dead-end pocket at (1,6)-(2,7)
        _s(1,6), _e(1,6),
        # Dead-end corridor along the bottom
        _n(2,0),_n(3,0),_n(4,0),
        _e(4,0),
        # Vertical wall forcing a turn
        _e(3,2),_e(3,3),_e(3,4),
        # Horizontal cross wall
        _n(0,4),_n(1,4),_n(2,4),
        _n(5,3),_n(6,3),_n(7,3),
        # Dead pocket on right
        _w(6,5),_s(6,5),
    ],
)


# ── 5. Classic competition-style 16×16 ────────────────────────────────────
# A condensed approximation of a typical competition maze.
# Walls are defined for the full 16×16 grid.
_register(
    name="competition",
    description="Competition-style 16×16 maze with multiple routes to centre",
    size=16,
    goal=(7, 7, 8, 8),
    walls=[
        # ---- Row 0 (bottom) ----
        _e(0,0), _n(2,0), _n(4,0), _e(5,0), _n(7,0),
        _n(9,0), _e(10,0), _n(12,0), _e(13,0),
        # ---- Row 1 ----
        _n(0,1), _e(1,1), _n(3,1), _e(4,1), _n(6,1),
        _e(7,1), _n(9,1), _e(10,1), _n(12,1),
        # ---- Row 2 ----
        _e(0,2), _n(1,2), _e(2,2), _n(4,2),
        _e(5,2), _n(7,2), _e(8,2), _n(10,2),
        _e(11,2), _n(13,2), _e(14,2),
        # ---- Row 3 ----
        _n(0,3), _e(1,3), _n(2,3), _e(3,3),
        _n(5,3), _e(6,3), _n(8,3), _e(9,3),
        _n(11,3), _e(12,3), _n(14,3),
        # ---- Row 4 ----
        _e(0,4), _n(1,4), _e(2,4), _n(3,4),
        _e(4,4), _n(5,4), _e(6,4), _n(7,4),
        _e(8,4), _n(9,4), _e(10,4), _n(11,4),
        _e(12,4), _n(13,4), _e(14,4),
        # ---- Rows 5-6 leading to centre ----
        _n(1,5), _e(2,5), _n(4,5), _e(5,5),
        _n(7,5), _e(8,5), _n(10,5), _e(11,5), _n(13,5),
        _e(0,6), _n(2,6), _e(3,6), _n(5,6),
        _e(6,6), _n(9,6), _e(10,6), _n(12,6), _e(13,6),
        # ---- Centre area (goal = 7,7 – 8,8) ----
        # West wall of goal:  east side of col 6
        _e(6,7), _e(6,8),
        # East wall of goal:  east side of col 8
        _e(8,7), _e(8,8),
        # North wall of goal: north side of row 8
        _n(7,8), _n(8,8),
        # South side left open (entry from below, between cols 7-8)
        # ---- Upper half (mirror-ish) ----
        _e(0,8), _n(1,8), _e(2,8), _n(3,8),
        _e(4,8), _n(5,8), _e(8,8), _n(10,8),
        _e(11,8), _n(12,8), _e(13,8),
        _n(0,9), _e(1,9), _n(2,9), _e(3,9),
        _n(5,9), _e(6,9), _n(8,9), _e(9,9),
        _n(11,9), _e(12,9), _n(14,9),
        _e(0,10), _n(1,10), _e(2,10), _n(4,10),
        _e(5,10), _n(7,10), _e(8,10), _n(10,10),
        _e(11,10), _n(13,10), _e(14,10),
        _n(0,11), _e(1,11), _n(3,11),
        _e(4,11), _n(6,11), _e(7,11),
        _n(9,11), _e(10,11), _n(12,11),
        _e(0,12), _n(2,12), _n(4,12),
        _e(5,12), _n(7,12), _e(8,12),
        _n(10,12), _e(11,12), _n(13,12),
        _n(0,13), _e(1,13), _n(3,13),
        _e(6,13), _n(8,13), _e(9,13),
        _n(12,13), _e(13,13),
        _e(0,14), _n(2,14), _e(3,14),
        _n(5,14), _e(6,14), _n(8,14),
        _e(9,14), _n(11,14), _e(12,14), _n(14,14),
    ],
)


# ── 6. Island maze ─────────────────────────────────────────────────────────
# Several isolated wall blocks create multiple equivalent routes.
_register(
    name="islands",
    description="Wall islands with multiple equally-optimal paths",
    size=8,
    goal=(3, 3, 4, 4),
    walls=[
        # Island 1 (bottom-left area)
        _n(1,1), _e(1,1), _s(2,2), _w(2,2),
        # Island 2 (bottom-right)
        _n(5,1), _w(5,1), _s(6,2), _e(6,2),
        # Island 3 (top-left)
        _s(1,5), _e(1,5), _n(2,4), _w(2,4),
        # Island 4 (top-right)
        _s(5,5), _w(5,5), _n(6,4), _e(6,4),
    ],
)


# ── 7. Unsolvable maze (for robustness testing) ────────────────────────────
_register(
    name="unsolvable",
    description="Goal is completely walled off – flood-fill returns INF",
    size=8,
    goal=(3, 3, 4, 4),
    walls=[
        # Completely surround the 3,3 – 4,4 goal region
        _n(3,4),_n(4,4),          # north side of goal
        _s(3,3),_s(4,3),          # south side of goal
        _w(3,3),_w(3,4),          # west side of goal
        _e(4,3),_e(4,4),          # east side of goal
    ],
)


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def load_preset(name: str) -> Maze:
    """Return a fully initialised Maze for the named preset."""
    if name not in PRESETS:
        raise KeyError(f"Unknown preset '{name}'. Available: {list(PRESETS)}")
    p = PRESETS[name]
    m = Maze(size=p["size"], goal=p["goal"])
    m.import_walls(p["walls"], reset_first=True)
    return m


def list_presets() -> None:
    """Print all registered presets."""
    print(f"{'Name':<15} {'Size':<6} {'Goal':<20} Description")
    print("-" * 70)
    for name, p in PRESETS.items():
        print(f"{name:<15} {p['size']:<6} {str(p['goal']):<20} {p['description']}")
