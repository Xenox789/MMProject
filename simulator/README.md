# MicroMouse Maze Simulator

A Python / pygame sandbox for designing maze layouts and verifying the flood-fill pathfinding algorithm before flashing firmware to the real robot.

The Python logic (`maze.py`) is a direct port of the C++ in `src/maze.cpp`, so any path you see here will match what the real mouse computes.

---

## Quick start

```bash
cd simulator
pip install -r requirements.txt
python simulator.py
```

Requires Python 3.9+ and pygame 2.5+.

---

## Controls

| Input | Action |
|-------|--------|
| **Left-click** on a wall edge | Toggle that wall on/off |
| **Right-click** a cell | Move the start position there |
| `F` | Run flood-fill and show distance gradient |
| `P` | Find and highlight the best path |
| `X` | Animate the mouse exploring from scratch (sensor-by-sensor) |
| `R` | Reset all internal walls (keep borders) |
| `C` | Clear path/flood overlay |
| `N` | Toggle flood-fill numbers inside cells |
| `G` + click | Re-define the goal to a 2×2 block at that corner |
| `1`–`7` | Load a preset maze |
| `+` / `-` | Zoom in / out |
| Arrow keys | Pan the grid |
| `Space` | Pause / resume simulation animation |
| `H` | Toggle in-app help overlay |
| `Esc` / `Q` | Quit |

---

## Preset mazes

| Key | Name | Description |
|-----|------|-------------|
| 1 | empty | No internal walls – trivial path |
| 2 | corridor | Single winding corridor (8×8) |
| 3 | spiral | Inward spiral, many turns required |
| 4 | dead_ends | Multiple dead-end pockets |
| 5 | competition | Full 16×16 competition-style maze |
| 6 | islands | Wall islands, multiple equal routes |
| 7 | unsolvable | Goal is sealed off – tests INF handling |

---

## File overview

```
simulator/
  maze.py          # Maze data structure + flood-fill (Python port of firmware)
  mazes.py         # Preset maze definitions
  simulator.py     # Interactive pygame application
  requirements.txt
```

### Colour legend (flood-fill view)

| Colour | Meaning |
|--------|---------|
| Blue | Very close to goal (low distance) |
| Teal → Green | Mid-range |
| Yellow → Red | Far from goal |
| Dark red cell | Unreachable (∞) |
| Purple cell | Goal region |
| Yellow dot | Mouse / start position |
| Green line | Best path |
| Orange line | Exploration path (simulation mode) |

---

## Scripting mazes

You can also use `maze.py` and `mazes.py` from Python scripts:

```python
from maze  import Maze, NORTH, EAST
from mazes import load_preset

# Load a preset
m = load_preset("spiral")
m.flood_fill()
m.print_maze(show_flood=True)

# Build a custom maze
m2 = Maze(size=8, goal=(3, 3, 4, 4))
m2.set_wall(3, 0, EAST)
m2.set_wall(3, 1, EAST)
m2.flood_fill()
path = m2.best_path()
print("Path:", path)

# Simulate full exploration
true_maze = load_preset("competition")
sim_maze  = Maze(size=16)
steps, flood_calls = sim_maze.simulate_exploration(true_maze)
print(f"Explored in {len(steps)} steps, {flood_calls} flood resets")
```
