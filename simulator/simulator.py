# -*- coding: utf-8 -*-
"""
simulator.py - Interactive MicroMouse maze sandbox (pygame)

Controls
--------
  Left-click wall  Toggle wall at that edge
  Right-click cell Set start position

  F                Flood-fill the current maze (show gradient)
  P                Show best path from start
  X                Start two-phase run:
                     Phase 1 EXPLORE - mouse discovers walls live,
                              bounces goal↔start until a full clean
                              round-trip finds no new walls
                     Phase 2 SPEEDRUN - mouse follows the optimal path
                              on the full learned map at full speed
                     (press X again during EXPLORE to force speedrun now)
  R                Reset all internal walls (keep borders)
  C                Clear overlays

  1-7              Load preset maze
  G                Set goal to 2x2 block (then click a cell)
  + / -            Zoom in / out
  Arrows           Pan grid
  Space            Pause / resume animation
  ESC / Q          Quit

  H                Toggle this help overlay
"""

import sys
import time
import pygame

from maze  import Maze, NORTH, EAST, SOUTH, WEST, INF, next_cell
from mazes import load_preset, PRESETS

# ---------------------------------------------------------------------------
# Display constants
# ---------------------------------------------------------------------------
WINDOW_W     = 1060
WINDOW_H     = 800
SIDEBAR_W    = 230
FPS          = 60

DEFAULT_CELL_PX = 36
MIN_CELL_PX     = 12
MAX_CELL_PX     = 72

WALL_THICK   = 3
BORDER_THICK = 5
WALL_HIT_ZONE = 0.22

# Palette
C_BG            = (15,  15,  25)
C_GRID          = (40,  40,  60)
C_CELL_BG       = (24,  24,  40)
C_WALL          = (220, 220, 235)
C_BORDER_WALL   = (255, 255, 255)
C_UNKNOWN_WALL  = (70,  70, 100)
C_GOAL_FILL     = (70,  25, 110)
C_GOAL_BORDER   = (190,  90, 255)
C_PATH          = (0,  220, 130)
C_PATH_DOT      = (0,  255, 100)
C_EXPLORE_TRAIL = (255, 160,  40)
C_MOUSE_EXPLORE = (255, 210,  40)
C_MOUSE_SPEED   = (80,  230, 255)
C_START_BG      = (35,  70, 130)
C_SIDEBAR_BG    = (20,  20,  36)
C_TEXT          = (210, 210, 230)
C_DIM           = (100, 100, 130)
C_ACCENT        = (80,  160, 255)
C_WARNING       = (255, 130,  50)
C_GOOD          = (55,  225, 105)
C_HELP_BG       = (8,    8,  18, 210)

_FLOOD_GRADIENT = [
    (10,  20, 180),
    (0,  160, 220),
    (0,  205, 115),
    (225, 215,  0),
    (225,  45, 25),
]

def _lerp_colour(t):
    t = max(0.0, min(1.0, t))
    n   = len(_FLOOD_GRADIENT) - 1
    idx = t * n
    lo  = int(idx)
    hi  = min(lo + 1, n)
    f   = idx - lo
    r = int(_FLOOD_GRADIENT[lo][0] * (1-f) + _FLOOD_GRADIENT[hi][0] * f)
    g = int(_FLOOD_GRADIENT[lo][1] * (1-f) + _FLOOD_GRADIENT[hi][1] * f)
    b = int(_FLOOD_GRADIENT[lo][2] * (1-f) + _FLOOD_GRADIENT[hi][2] * f)
    return (r, g, b)


# ---------------------------------------------------------------------------
# Simulation phases
# ---------------------------------------------------------------------------
class Phase:
    IDLE     = "idle"
    FLOODED  = "flooded"
    PATH     = "path"
    EXPLORE  = "explore"
    SPEEDRUN = "speedrun"
    SET_GOAL = "set_goal"


# ---------------------------------------------------------------------------
# Main application
# ---------------------------------------------------------------------------
class MazeSimulator:

    def __init__(self):
        pygame.init()
        pygame.display.set_caption("MicroMouse - Maze Simulator")
        self.screen = pygame.display.set_mode((WINDOW_W, WINDOW_H), pygame.RESIZABLE)
        self.clock  = pygame.time.Clock()

        self.font_sm = pygame.font.SysFont("consolas", 12)
        self.font_md = pygame.font.SysFont("consolas", 14)
        self.font_lg = pygame.font.SysFont("consolas", 17, bold=True)
        self.font_xl = pygame.font.SysFont("consolas", 21, bold=True)

        self.cell_px  = DEFAULT_CELL_PX
        self.offset_x = SIDEBAR_W + 16
        self.offset_y = 16

        self.maze          = Maze()
        self.preset_name   = "(empty)"
        self.phase         = Phase.IDLE
        self.start_x       = 0
        self.start_y       = 0
        self.start_heading = NORTH
        self.path          = []
        self.hovered_wall  = None
        self.show_flood_nums = True
        self.show_help     = False
        self.status_msg    = "Ready - press H for help."
        self._pending_goal_click = False
        self._paused       = False

        # Live-run state
        self._known_maze          = None
        self._mouse_x             = 0
        self._mouse_y             = 0
        self._mouse_head          = NORTH
        self._explore_trail       = []   # full history of every cell visited
        self._speedrun_path       = []
        self._run_step            = 0
        self._step_timer          = 0.0
        self.explore_speed        = 0.10
        self.speedrun_speed       = 0.06
        self._explore_flood_calls = 0
        self._explore_total_steps = 0
        # Round-trip exploration state
        self._explore_target_goal = True   # True = heading to goal, False = heading to start
        self._new_walls_this_leg  = False  # any new walls found since last waypoint?
        self._clean_legs          = 0      # consecutive legs with zero new walls
        self._goal_found          = False  # mouse has physically reached the true goal

    # ------------------------------------------------------------------
    # Grid helpers
    # ------------------------------------------------------------------

    def _grid_origin(self):
        return (self.offset_x, self.offset_y)

    def _maze_to_screen(self, x, y):
        ox, oy = self._grid_origin()
        N = self.maze.size
        return (ox + x * self.cell_px, oy + (N - 1 - y) * self.cell_px)

    def _screen_to_maze(self, mx, my):
        ox, oy = self._grid_origin()
        N = self.maze.size
        raw_x = (mx - ox) / self.cell_px
        raw_y = (my - oy) / self.cell_px
        return int(raw_x), N - 1 - int(raw_y), raw_x % 1, raw_y % 1

    def _wall_under_cursor(self, mx, my):
        cx, cy, rx, ry = self._screen_to_maze(mx, my)
        N = self.maze.size
        z = WALL_HIT_ZONE
        cands = []
        if ry < z       and 0 <= cx < N and 0 <= cy < N:    cands.append((cx, cy,   NORTH))
        if ry > 1-z     and 0 <= cx < N and 0 <= cy-1 < N:  cands.append((cx, cy-1, NORTH))
        if rx < z       and 0 <= cx < N and 0 <= cy < N:    cands.append((cx, cy,   WEST))
        if rx > 1-z     and 0 <= cx+1<N and 0 <= cy < N:    cands.append((cx, cy,   EAST))
        return cands[-1] if cands else None

    def _is_border(self, x, y, d):
        N = self.maze.size
        return ((d == NORTH and y == N-1) or (d == SOUTH and y == 0) or
                (d == EAST  and x == N-1) or (d == WEST  and x == 0))

    # ------------------------------------------------------------------
    # Preset / view helpers
    # ------------------------------------------------------------------

    def load_preset(self, name):
        self.maze        = load_preset(name)
        self.preset_name = name
        self._reset_run_state()
        self.phase  = Phase.IDLE
        self.path   = []
        self.start_x = 0
        self.start_y = 0
        self.status_msg = f"Loaded preset: '{name}'"
        self._fit_maze()

    def _fit_maze(self):
        aw = self.screen.get_width()  - SIDEBAR_W - 32
        ah = self.screen.get_height() - 32
        N  = self.maze.size
        self.cell_px = max(MIN_CELL_PX, min(MAX_CELL_PX, min(aw // N, ah // N)))

    def _reset_run_state(self):
        self._known_maze             = None
        self._explore_trail          = []
        self._speedrun_path          = []
        self._run_step               = 0
        self._explore_flood_calls    = 0
        self._explore_total_steps    = 0
        self._explore_target_goal    = True
        self._new_walls_this_leg     = False
        self._clean_legs             = 0
        self._goal_found             = False
        self._visited                = set()
        self._explore_temp_target    = None  # (x,y) when using frontier target
        self._paused                 = False

    # ------------------------------------------------------------------
    # Static actions (F / P)
    # ------------------------------------------------------------------

    def do_flood_fill(self):
        self.maze.flood_fill()
        self.phase = Phase.FLOODED
        v = self.maze.get_flood(self.start_x, self.start_y)
        self.status_msg = (f"Flood fill  |  start->goal: "
                           f"{v if v < INF else 'unreachable'} cells")

    def do_best_path(self):
        self.maze.flood_fill()
        self.path = self.maze.best_path(self.start_x, self.start_y,
                                        self.start_heading)
        self.phase = Phase.PATH if self.path else Phase.FLOODED
        self.status_msg = (f"Best path: {len(self.path)-1} steps" if self.path
                           else "No path to goal - goal may be walled off.")

    # ------------------------------------------------------------------
    # Two-phase run  (X key)
    # ------------------------------------------------------------------

    def do_start_run(self):
        """Phase 1: exploration from scratch (round-trip until fully mapped)."""
        self._reset_run_state()
        self.path = []

        # The mouse does NOT know the true goal location.
        # Target the maze center by default (standard competition convention).
        N = self.maze.size
        c = N // 2
        center_goal = (c-1, c-1, c, c) if N % 2 == 0 else (c, c, c, c)
        km = Maze(size=N, goal=center_goal)
        km.flood_fill()   # flood toward center
        self._known_maze = km

        self._mouse_x             = self.start_x
        self._mouse_y             = self.start_y
        self._mouse_head          = self.start_heading
        self._explore_trail       = [(self._mouse_x, self._mouse_y)]
        self._visited             = {(self.start_x, self.start_y)}
        self._explore_temp_target = None
        self._explore_target_goal = True    # first leg: go to (assumed) goal
        self._new_walls_this_leg  = False
        self._clean_legs          = 0
        self._step_timer          = time.time()
        self.phase                = Phase.EXPLORE
        self.status_msg = "Phase 1: EXPLORE  |  searching for goal...  | [Space] pause  [X] speedrun now"

    def _at_target(self, x, y):
        """True when the mouse has reached the current exploration target."""
        if self._explore_target_goal:
            if self._goal_found:
                gx1, gy1, gx2, gy2 = self.maze.goal
                return gx1 <= x <= gx2 and gy1 <= y <= gy2
            elif self._explore_temp_target is not None:
                return (x, y) == self._explore_temp_target
            else:
                # Still heading toward assumed center
                gx1, gy1, gx2, gy2 = self._known_maze.goal
                return gx1 <= x <= gx2 and gy1 <= y <= gy2
        else:
            return x == self.start_x and y == self.start_y

    def _pick_frontier_target(self):
        """
        Find the closest unvisited reachable cell in known_maze and aim for it.
        Returns True if a new target was set, False if all reachable cells visited.
        """
        km = self._known_maze
        N  = km.size
        best_cell = None
        best_dist = -1
        mx, my = self._mouse_x, self._mouse_y
        for x in range(N):
            for y in range(N):
                if (x, y) not in self._visited and km.get_flood(x, y) < INF:
                    # Manhattan distance from current position as tiebreaker
                    d = abs(x - mx) + abs(y - my)
                    if d > best_dist:
                        best_dist = d
                        best_cell = (x, y)
        if best_cell is None:
            return False
        bx, by = best_cell
        self._explore_temp_target = best_cell
        km.goal = (bx, by, bx, by)
        km.flood_fill()
        self._explore_flood_calls += 1
        return True

    def _tick_explore(self):
        km = self._known_maze
        x, y, h = self._mouse_x, self._mouse_y, self._mouse_head
        N = self.maze.size

        # ── Mark cell as visited ──
        self._visited.add((x, y))

        # ── Detect the true goal by physical arrival (sensor: "am I in the goal?") ──
        if not self._goal_found:
            gx1, gy1, gx2, gy2 = self.maze.goal
            if gx1 <= x <= gx2 and gy1 <= y <= gy2:
                self._goal_found        = True
                self._explore_temp_target = None
                km.goal = self.maze.goal
                km.flood_fill()
                self._explore_flood_calls += 1
                self.status_msg = (f"Goal found at {self.maze.goal}!  "
                                   f"steps={self._explore_total_steps}")

        # ── Sense all four walls at current cell from the true maze ──
        walls_added = False
        for d in range(4):
            if self.maze.has_wall(x, y, d) and not km.has_wall(x, y, d):
                km.set_wall(x, y, d)
                walls_added = True

        if walls_added:
            # Mark that new walls were found during this leg
            self._new_walls_this_leg = True
            # Reflood toward the current target
            if self._explore_target_goal:
                km.flood_fill()
            else:
                km.flood_fill((self.start_x, self.start_y,
                               self.start_x, self.start_y))
            self._explore_flood_calls += 1

        # ── Reached the current waypoint? ──
        if self._at_target(x, y):
            if self._new_walls_this_leg:
                self._clean_legs = 0
            else:
                self._clean_legs += 1

            # ── Goal not yet found: pick a new frontier target ──
            if self._explore_target_goal and not self._goal_found:
                # Reached assumed center (or last frontier) but goal not there.
                # Pick a new unexplored region to visit.
                if self._pick_frontier_target():
                    self._new_walls_this_leg = False
                    self.status_msg = (f"Exploring frontier @ {self._explore_temp_target}  "
                                       f"| steps={self._explore_total_steps}")
                    return   # keep going outward, don't flip to return yet
                else:
                    # All reachable cells visited — maze fully explored, goal never found
                    # (shouldn't happen in a valid competition maze)
                    self._finish_explore()
                    return

            # Two consecutive clean legs = full clean round trip → done
            if self._clean_legs >= 2:
                self._finish_explore()
                return

            # Flip target and reflood toward new target
            self._explore_target_goal = not self._explore_target_goal
            self._new_walls_this_leg  = False
            if self._explore_target_goal:
                km.flood_fill()
                self.status_msg = (f"Exploring: heading to goal  "
                                   f"| steps={self._explore_total_steps} "
                                   f"floods={self._explore_flood_calls}")
            else:
                km.flood_fill((self.start_x, self.start_y,
                               self.start_x, self.start_y))
                self.status_msg = (f"Exploring: returning to start  "
                                   f"| steps={self._explore_total_steps} "
                                   f"floods={self._explore_flood_calls}")
            return

        # ── Move toward current target ──
        best = km.best_direction(x, y, h)
        if best is None or km.has_wall(x, y, best):
            # Completely stuck — finish anyway
            self._finish_explore()
            return

        nx, ny = next_cell(x, y, best)
        if not (0 <= nx < N and 0 <= ny < N):
            self._finish_explore()
            return

        self._mouse_x    = nx
        self._mouse_y    = ny
        self._mouse_head = best
        self._explore_trail.append((nx, ny))
        self._explore_total_steps += 1

    def _finish_explore(self):
        """Maze mapped — speedrun on the knowledge the mouse actually acquired."""
        km = self._known_maze
        # Ensure known_maze targets the true goal (may have been a temp target)
        if self._goal_found:
            km.goal = self.maze.goal
        km.flood_fill()
        self._explore_flood_calls += 1
        self._speedrun_path = km.best_path(
            self.start_x, self.start_y, self.start_heading)

        self._run_step   = 0
        self._step_timer = time.time()
        self.phase       = Phase.SPEEDRUN

        steps = len(self._speedrun_path) - 1 if self._speedrun_path else 0
        found_str = "goal found" if self._goal_found else "goal NOT found — path to center"
        self.status_msg = (
            f"Phase 2: SPEEDRUN  |  {found_str}  |  {steps} steps  "
            f"| explored {self._explore_total_steps} steps, "
            f"{self._explore_flood_calls} floods  | [Space] pause")

    def _tick_speedrun(self):
        if self._run_step < len(self._speedrun_path) - 1:
            self._run_step += 1
        else:
            sp = len(self._speedrun_path) - 1
            self.status_msg = (f"Run complete!  Optimal = {sp} steps  |  "
                               f"Explored in {self._explore_total_steps} steps  |  "
                               f"[X] restart  [C] clear")
            self._paused = True

    # ------------------------------------------------------------------
    # Drawing
    # ------------------------------------------------------------------

    def _draw_flood_overlay(self, surf, maze):
        N = maze.size
        vals = [maze.get_flood(x, y)
                for x in range(N) for y in range(N)
                if maze.get_flood(x, y) < INF]
        max_val = max(vals) if vals else 1
        for x in range(N):
            for y in range(N):
                v = maze.get_flood(x, y)
                c = (35,15,15) if v == INF else _lerp_colour(v / max_val)
                sx, sy = self._maze_to_screen(x, y)
                pygame.draw.rect(surf, c,
                                 (sx+1, sy+1, self.cell_px-2, self.cell_px-2))

    def _draw_flood_numbers(self, surf, maze):
        if self.cell_px < 24:
            return
        N = maze.size
        for x in range(N):
            for y in range(N):
                v = maze.get_flood(x, y)
                if v == INF:
                    continue
                sx, sy = self._maze_to_screen(x, y)
                t = self.font_sm.render(str(v), True, (255,255,255))
                surf.blit(t, t.get_rect(
                    center=(sx + self.cell_px//2, sy + self.cell_px//2)))

    def _draw_goal_highlight(self, surf):
        gx1, gy1, gx2, gy2 = self.maze.goal
        for gx in range(gx1, gx2+1):
            for gy in range(gy1, gy2+1):
                sx, sy = self._maze_to_screen(gx, gy)
                s = pygame.Surface((self.cell_px-2, self.cell_px-2), pygame.SRCALPHA)
                s.fill((*C_GOAL_FILL, 160))
                surf.blit(s, (sx+1, sy+1))

    def _draw_path_line(self, surf, path, colour, dot_colour=None, width=3):
        if not path:
            return
        cp  = self.cell_px
        pts = []
        for (x,y) in path:
            sx, sy = self._maze_to_screen(x, y)
            pts.append((sx+cp//2, sy+cp//2))
        if len(pts) > 1:
            pygame.draw.lines(surf, colour, False, pts, width)
        dc = dot_colour or colour
        for i in range(1, len(pts)-1):
            pygame.draw.circle(surf, dc, pts[i], max(2, cp//8))
        if pts:
            pygame.draw.circle(surf, C_ACCENT, pts[0], max(3, cp//6))

    def _draw_mouse(self, surf, x, y, heading, colour):
        cp = self.cell_px
        sx, sy = self._maze_to_screen(x, y)
        cx_ = sx + cp//2
        cy_ = sy + cp//2
        r   = max(5, cp//4)
        pygame.draw.circle(surf, colour, (cx_, cy_), r)
        pygame.draw.circle(surf, (255,255,255), (cx_, cy_), r, 2)
        # Direction indicator triangle (screen: y inverted)
        dx = [0, 1, 0, -1][heading]
        dy = [-1, 0, 1, 0][heading]
        tip_x = cx_ + dx*(r+3)
        tip_y = cy_ + dy*(r+3)
        pdx, pdy = -dy, dx
        hs = max(2, r//2)
        p1 = (tip_x, tip_y)
        p2 = (cx_+pdx*hs, cy_+pdy*hs)
        p3 = (cx_-pdx*hs, cy_-pdy*hs)
        pygame.draw.polygon(surf, (255,255,255), [p1,p2,p3])

    def _draw_walls_of_maze(self, surf, maze, colour, thickness):
        N  = maze.size
        cp = self.cell_px
        for x in range(N):
            for y in range(N):
                sx, sy = self._maze_to_screen(x, y)
                for d, (x1,y1,x2,y2) in [
                    (NORTH, (sx,    sy,    sx+cp, sy)),
                    (SOUTH, (sx,    sy+cp, sx+cp, sy+cp)),
                    (WEST,  (sx,    sy,    sx,    sy+cp)),
                    (EAST,  (sx+cp, sy,    sx+cp, sy+cp)),
                ]:
                    if maze.has_wall(x, y, d):
                        is_brd = self._is_border(x, y, d)
                        c = C_BORDER_WALL if is_brd else colour
                        t = BORDER_THICK  if is_brd else thickness
                        pygame.draw.line(surf, c, (x1,y1), (x2,y2), t)

    def _draw_unknown_walls(self, surf, true_maze, known_maze):
        N  = true_maze.size
        cp = self.cell_px
        for x in range(N):
            for y in range(N):
                sx, sy = self._maze_to_screen(x, y)
                for d, (x1,y1,x2,y2) in [
                    (NORTH, (sx,    sy,    sx+cp, sy)),
                    (SOUTH, (sx,    sy+cp, sx+cp, sy+cp)),
                    (WEST,  (sx,    sy,    sx,    sy+cp)),
                    (EAST,  (sx+cp, sy,    sx+cp, sy+cp)),
                ]:
                    if (true_maze.has_wall(x, y, d)
                            and not known_maze.has_wall(x, y, d)
                            and not self._is_border(x, y, d)):
                        pygame.draw.line(surf, C_UNKNOWN_WALL, (x1,y1), (x2,y2), 1)

    def _draw_grid(self, surf):
        N  = self.maze.size
        ox, oy = self._grid_origin()
        cp = self.cell_px

        pygame.draw.rect(surf, C_CELL_BG, (ox, oy, N*cp, N*cp))

        # Flood overlay
        if self.phase in (Phase.FLOODED, Phase.PATH, Phase.SPEEDRUN):
            self._draw_flood_overlay(surf, self.maze)
            if self.show_flood_nums:
                self._draw_flood_numbers(surf, self.maze)
        elif self.phase == Phase.EXPLORE and self._known_maze:
            self._draw_flood_overlay(surf, self._known_maze)
            if self.show_flood_nums:
                self._draw_flood_numbers(surf, self._known_maze)

        # Goal + start
        self._draw_goal_highlight(surf)
        sx, sy = self._maze_to_screen(self.start_x, self.start_y)
        pygame.draw.rect(surf, C_START_BG, (sx+1, sy+1, cp-2, cp-2))

        # Path overlays
        if self.phase == Phase.PATH and self.path:
            self._draw_path_line(surf, self.path, C_PATH, C_PATH_DOT)
        if self.phase == Phase.EXPLORE and self._explore_trail:
            self._draw_path_line(surf, self._explore_trail,
                                 C_EXPLORE_TRAIL, width=2)
        if self.phase == Phase.SPEEDRUN:
            self._draw_path_line(surf, self._explore_trail,
                                 (100,70,20), width=1)
            if self._speedrun_path:
                vis = self._speedrun_path[:self._run_step+1]
                self._draw_path_line(surf, vis, C_PATH, C_PATH_DOT, width=3)

        # Grid lines
        for i in range(N+1):
            pygame.draw.line(surf, C_GRID,
                             (ox, oy+i*cp), (ox+N*cp, oy+i*cp), 1)
            pygame.draw.line(surf, C_GRID,
                             (ox+i*cp, oy), (ox+i*cp, oy+N*cp), 1)

        # Walls
        if self.phase == Phase.EXPLORE and self._known_maze:
            self._draw_unknown_walls(surf, self.maze, self._known_maze)
            self._draw_walls_of_maze(surf, self._known_maze, C_WALL, WALL_THICK)
        else:
            self._draw_walls_of_maze(surf, self.maze, C_WALL, WALL_THICK)

        # Hovered wall highlight
        if self.hovered_wall:
            hx, hy, hd = self.hovered_wall
            hsx, hsy = self._maze_to_screen(hx, hy)
            pts = {
                NORTH: ((hsx, hsy),    (hsx+cp, hsy)),
                SOUTH: ((hsx, hsy+cp), (hsx+cp, hsy+cp)),
                WEST:  ((hsx, hsy),    (hsx, hsy+cp)),
                EAST:  ((hsx+cp, hsy), (hsx+cp, hsy+cp)),
            }[hd]
            pygame.draw.line(surf, (255,200,60), *pts, 4)

        # Mouse marker
        if self.phase == Phase.EXPLORE and self._known_maze:
            self._draw_mouse(surf, self._mouse_x, self._mouse_y,
                             self._mouse_head, C_MOUSE_EXPLORE)
        elif self.phase == Phase.SPEEDRUN and self._speedrun_path:
            step = min(self._run_step, len(self._speedrun_path)-1)
            rx, ry = self._speedrun_path[step]
            rh = NORTH
            if step > 0:
                px2, py2 = self._speedrun_path[step-1]
                for d in range(4):
                    nx2, ny2 = next_cell(px2, py2, d)
                    if nx2 == rx and ny2 == ry:
                        rh = d; break
            self._draw_mouse(surf, rx, ry, rh, C_MOUSE_SPEED)
        else:
            self._draw_mouse(surf, self.start_x, self.start_y,
                             self.start_heading, C_MOUSE_EXPLORE)

        # Goal labels
        gx1, gy1, gx2, gy2 = self.maze.goal
        for gx in range(gx1, gx2+1):
            for gy in range(gy1, gy2+1):
                gsx, gsy = self._maze_to_screen(gx, gy)
                if cp >= 18:
                    lbl = self.font_sm.render("G", True, C_GOAL_BORDER)
                    surf.blit(lbl, lbl.get_rect(
                        center=(gsx+cp//2, gsy+cp//2)))

    # ------------------------------------------------------------------
    # Sidebar
    # ------------------------------------------------------------------

    def _draw_sidebar(self, surf):
        H = self.screen.get_height()
        pygame.draw.rect(surf, C_SIDEBAR_BG, (0, 0, SIDEBAR_W, H))
        pygame.draw.line(surf, C_GRID, (SIDEBAR_W, 0), (SIDEBAR_W, H), 1)

        y = 12
        pad = 10

        def text(s, colour=C_TEXT, font=None):
            nonlocal y
            f = font or self.font_md
            r = f.render(s, True, colour)
            surf.blit(r, (pad, y))
            y += r.get_height() + 3

        def sep():
            nonlocal y
            pygame.draw.line(surf, C_GRID, (6, y+2), (SIDEBAR_W-6, y+2), 1)
            y += 9

        text("MicroMouse", C_ACCENT, self.font_xl)
        text("Maze Simulator", C_DIM, self.font_sm)
        sep()

        N = self.maze.size
        text(f"Preset: {self.preset_name}")
        text(f"Size:   {N}x{N}")
        gx1, gy1, gx2, gy2 = self.maze.goal
        text(f"Goal:   ({gx1},{gy1})->({gx2},{gy2})")
        text(f"Start:  ({self.start_x},{self.start_y})")
        sep()

        phase_colour = {
            Phase.IDLE:     C_DIM,
            Phase.FLOODED:  C_ACCENT,
            Phase.PATH:     C_GOOD,
            Phase.EXPLORE:  C_WARNING,
            Phase.SPEEDRUN: C_MOUSE_SPEED,
            Phase.SET_GOAL: (255, 220, 80),
        }.get(self.phase, C_TEXT)
        text(f"Phase: {self.phase.upper()}", phase_colour, self.font_lg)

        if self.phase in (Phase.FLOODED, Phase.PATH):
            v = self.maze.get_flood(self.start_x, self.start_y)
            vs = str(v) if v < INF else "unreachable"
            text(f"Dist to goal: {vs}", C_GOOD if v < INF else C_WARNING)
        if self.phase == Phase.PATH and self.path:
            text(f"Path steps:   {len(self.path)-1}", C_GOOD)

        if self.phase == Phase.EXPLORE:
            text(f"Steps so far: {self._explore_total_steps}", C_WARNING)
            text(f"Flood calls:  {self._explore_flood_calls}", C_WARNING)
            direction = "→ Goal" if self._explore_target_goal else "← Start"
            text(f"Target:       {direction}", C_WARNING)
            text(f"Clean legs:   {self._clean_legs}/2", C_DIM)
            if self._paused:
                text("  [PAUSED]", (255,80,80), self.font_lg)

        if self.phase == Phase.SPEEDRUN:
            text(f"Explored in:  {self._explore_total_steps} steps", C_DIM)
            text(f"Flood calls:  {self._explore_flood_calls}", C_DIM)
            sp = len(self._speedrun_path)-1 if self._speedrun_path else 0
            text(f"Optimal path: {sp} steps", C_GOOD)
            cur = min(self._run_step, sp)
            text(f"Speedrun:     {cur}/{sp}", C_MOUSE_SPEED)
            if self._paused:
                text("  [PAUSED]", (255,80,80), self.font_lg)

        sep()

        text("Keys:", C_ACCENT)
        shortcuts = [
            ("F",     "Flood fill"),
            ("P",     "Best path"),
            ("X",     "Start run"),
            ("Space", "Pause/resume"),
            ("R",     "Reset walls"),
            ("C",     "Clear overlay"),
            ("N",     "Toggle numbers"),
            ("G+clk", "Set goal"),
            ("1-7",   "Preset"),
            ("+/-",   "Zoom"),
            ("Arrows","Pan"),
            ("H",     "Help"),
        ]
        for k, d in shortcuts:
            ks = self.font_sm.render(f"[{k}]", True, C_ACCENT)
            ds = self.font_sm.render(f" {d}", True, C_DIM)
            surf.blit(ks, (pad, y))
            surf.blit(ds, (pad+ks.get_width(), y))
            y += ks.get_height() + 2

        sep()
        text("Presets:", C_ACCENT, self.font_sm)
        for i, name in enumerate(list(PRESETS.keys())[:7], 1):
            clr = C_ACCENT if name == self.preset_name else C_DIM
            text(f"  [{i}] {name}", clr, self.font_sm)

        bar_y = H - 26
        pygame.draw.rect(surf, (26, 26, 44), (0, bar_y, SIDEBAR_W, 26))
        s = self.font_sm.render(self.status_msg[:30], True, C_TEXT)
        surf.blit(s, (6, bar_y+6))

    def _draw_status_bar(self, surf):
        W, H = self.screen.get_width(), self.screen.get_height()
        bh = 22
        pygame.draw.rect(surf, (14,14,26), (SIDEBAR_W, H-bh, W-SIDEBAR_W, bh))
        pygame.draw.line(surf, C_GRID, (SIDEBAR_W, H-bh), (W, H-bh), 1)
        surf.blit(self.font_sm.render(self.status_msg, True, C_TEXT),
                  (SIDEBAR_W+8, H-bh+4))

    def _draw_phase_banner(self, surf):
        if self.phase not in (Phase.EXPLORE, Phase.SPEEDRUN):
            return
        if self.phase == Phase.EXPLORE:
            direction = "→ GOAL" if self._explore_target_goal else "← START"
            label  = f"Phase 1: EXPLORING  {direction}  (clean:{self._clean_legs}/2)"
        else:
            label  = "Phase 2: SPEED RUN"
        colour = C_WARNING if self.phase == Phase.EXPLORE else C_MOUSE_SPEED
        if self._paused:
            label += "  [PAUSED]"
            colour = (200, 80, 80)
        r = self.font_lg.render(label, True, colour)
        self.screen.blit(r, (SIDEBAR_W + 8, 4))

    def _draw_help(self, surf):
        W, H = self.screen.get_width(), self.screen.get_height()
        s = pygame.Surface((W, H), pygame.SRCALPHA)
        s.fill(C_HELP_BG)
        surf.blit(s, (0, 0))
        lines = __doc__.strip().split("\n")
        y = H//2 - len(lines)*9
        for line in lines:
            r = self.font_md.render(line, True, C_TEXT)
            surf.blit(r, (r.get_rect(centerx=W//2).x, y))
            y += r.get_height() + 2

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------

    def run(self):
        self.load_preset("empty")

        while True:
            self.clock.tick(FPS)

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit(); sys.exit()
                elif event.type == pygame.VIDEORESIZE:
                    self._fit_maze()
                elif event.type == pygame.KEYDOWN:
                    self._handle_key(event.key)
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    if   event.button == 1: self._handle_left_click(*event.pos)
                    elif event.button == 3: self._handle_right_click(*event.pos)
                    elif event.button == 4: self.cell_px = min(MAX_CELL_PX, self.cell_px+2)
                    elif event.button == 5: self.cell_px = max(MIN_CELL_PX, self.cell_px-2)
                elif event.type == pygame.MOUSEMOTION:
                    w = self._wall_under_cursor(*event.pos)
                    self.hovered_wall = (w if w and not self._is_border(*w) else None)

            if not self._paused:
                now = time.time()
                if self.phase == Phase.EXPLORE:
                    if now - self._step_timer >= self.explore_speed:
                        self._step_timer = now
                        self._tick_explore()
                elif self.phase == Phase.SPEEDRUN:
                    if now - self._step_timer >= self.speedrun_speed:
                        self._step_timer = now
                        self._tick_speedrun()

            self.screen.fill(C_BG)
            self._draw_grid(self.screen)
            self._draw_sidebar(self.screen)
            self._draw_status_bar(self.screen)
            self._draw_phase_banner(self.screen)
            if self.show_help:
                self._draw_help(self.screen)
            pygame.display.flip()

    # ------------------------------------------------------------------
    # Input handlers
    # ------------------------------------------------------------------

    def _handle_key(self, key):
        if key in (pygame.K_ESCAPE, pygame.K_q):
            pygame.quit(); sys.exit()
        elif key == pygame.K_h:
            self.show_help = not self.show_help
        elif key == pygame.K_f:
            self._reset_run_state(); self.do_flood_fill()
        elif key == pygame.K_p:
            self._reset_run_state(); self.do_best_path()
        elif key == pygame.K_x:
            if self.phase == Phase.EXPLORE:
                # Force-finish exploration early and start speedrun
                self._finish_explore()
            else:
                self.do_start_run()
        elif key == pygame.K_SPACE:
            if self.phase in (Phase.EXPLORE, Phase.SPEEDRUN):
                self._paused = not self._paused
                self._step_timer = time.time()
        elif key == pygame.K_r:
            self.maze.reset_walls(); self._reset_run_state()
            self.phase = Phase.IDLE; self.path = []
            self.status_msg = "Walls reset."
        elif key == pygame.K_c:
            self._reset_run_state()
            self.phase = Phase.IDLE; self.path = []
            self.status_msg = "Overlay cleared."
        elif key == pygame.K_n:
            self.show_flood_nums = not self.show_flood_nums
        elif key == pygame.K_g:
            self.phase = Phase.SET_GOAL
            self._pending_goal_click = True
            self.status_msg = "Click a cell - goal will be 2x2 block at that corner."
        elif key in (pygame.K_PLUS, pygame.K_EQUALS, pygame.K_KP_PLUS):
            self.cell_px = min(MAX_CELL_PX, self.cell_px+2)
        elif key in (pygame.K_MINUS, pygame.K_KP_MINUS):
            self.cell_px = max(MIN_CELL_PX, self.cell_px-2)
        elif key == pygame.K_LEFT:  self.offset_x -= self.cell_px
        elif key == pygame.K_RIGHT: self.offset_x += self.cell_px
        elif key == pygame.K_UP:    self.offset_y -= self.cell_px
        elif key == pygame.K_DOWN:  self.offset_y += self.cell_px
        else:
            num = key - pygame.K_1
            keys = list(PRESETS.keys())
            if 0 <= num < len(keys):
                self.load_preset(keys[num])

    def _handle_left_click(self, mx, my):
        if self._pending_goal_click:
            cx, cy, _, _ = self._screen_to_maze(mx, my)
            N = self.maze.size
            if 0 <= cx < N and 0 <= cy < N:
                gx1 = max(0, cx-1); gy1 = max(0, cy-1)
                gx2 = min(N-1, cx); gy2 = min(N-1, cy)
                self.maze.goal = (gx1, gy1, gx2, gy2)
                self._pending_goal_click = False
                self.phase = Phase.IDLE; self.path = []
                self.status_msg = f"Goal -> ({gx1},{gy1})-({gx2},{gy2}). Press F or X."
            return
        wall = self._wall_under_cursor(mx, my)
        if wall and not self._is_border(*wall):
            self.maze.toggle_wall(*wall)
            self._reset_run_state()
            self.phase = Phase.IDLE; self.path = []
            x, y, d = wall
            action = "Added" if self.maze.has_wall(*wall) else "Removed"
            self.status_msg = f"{action} wall ({x},{y}){'NESW'[d]} - press F or X."

    def _handle_right_click(self, mx, my):
        cx, cy, _, _ = self._screen_to_maze(mx, my)
        N = self.maze.size
        if 0 <= cx < N and 0 <= cy < N:
            self.start_x = cx; self.start_y = cy
            self._reset_run_state(); self.path = []
            self.status_msg = f"Start -> ({cx},{cy}). Press P or X."


# ---------------------------------------------------------------------------
if __name__ == "__main__":
    MazeSimulator().run()
