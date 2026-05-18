"""
ml_Tdecide.py
-------------
Reads compact map result files for one or more agents and outputs movement
decisions as a comma-separated line on stdout.

File naming convention:
  compact_map_result_Ep_<EP>_Node_<NODE>.txt

Usage (called by ML_Script.sh):
  python3 ml_Tdecide.py --ep <episode> --nodes <node1> [node2 ...]

Output (stdout):
  move1,move2,...   e.g.  forward,turn_left

Debug info goes to stderr so the shell can cleanly capture the CSV.

File format:
  <TILE_STRING>,<AGENT_X>,<AGENT_Y>

Tile string:
  - Every 2 characters = one tile: <Color><Content>
      Colors : Y=Yellow, M=Magenta, B=Blue, P=Purple, ?=Border (impassable)
      Content: E=Empty, O=Obstacle, T=Target
  - Last single character = agent facing direction: L=Left, R=Right, U=Up, D=Down

Tiles are scanned row by row, top to bottom, left to right.
"""

import sys
import math
import argparse
import os

# Target coords are passed via --target X Y from the mission leader script.

# ── Results directory (must match mapingfromtxtfile.py RESULTS_DIR) ──────────
RESULTS_DIR = "Results"
# ────────────────────────────────────────────────────────────────────────────

DIRECTION_VECTORS = {
    'U': (0, -1),
    'D': (0,  1),
    'L': (-1, 0),
    'R': (1,  0),
}

COLOR_MAP = {
    'Y': 'Yellow',
    'M': 'Magenta',
    'B': 'Blue',
    'P': 'Purple',
    '?': 'Border',   # No colour — grid boundary, impassable
}

CONTENT_MAP = {
    'E': 'Empty',
    'O': 'Obstacle',
    'T': 'Target',
}

FACING_NAMES = {'U': 'Up', 'D': 'Down', 'L': 'Left', 'R': 'Right'}

TURN_LEFT  = {'U': 'L', 'L': 'D', 'D': 'R', 'R': 'U'}
TURN_RIGHT = {'U': 'R', 'R': 'D', 'D': 'L', 'L': 'U'}


def log(*args, **kwargs):
    """Print debug info to stderr so stdout stays clean for CSV output."""
    print(*args, **kwargs, file=sys.stderr)


def parse_file(filepath):
    """Parse a compact map result file."""
    with open(filepath, 'r') as f:
        line = f.read().strip()

    parts = line.split(',')
    if len(parts) != 3:
        raise ValueError(f"Expected <TILE_STRING>,<X>,<Y> — got: {line!r}")

    tile_string  = parts[0]
    agent_x      = int(parts[1])
    agent_y      = int(parts[2])

    facing = tile_string[-1]
    if facing not in DIRECTION_VECTORS:
        raise ValueError(f"Unknown facing direction: {facing!r}. Expected L/R/U/D.")

    tile_data = tile_string[:-1]
    if len(tile_data) % 2 != 0:
        raise ValueError(f"Tile data length {len(tile_data)} is odd: {tile_data!r}")

    tiles = []
    for i in range(0, len(tile_data), 2):
        color_char   = tile_data[i]
        content_char = tile_data[i + 1]
        color   = COLOR_MAP.get(color_char,   f'Unknown({color_char})')
        content = CONTENT_MAP.get(content_char, f'Unknown({content_char})')
        tiles.append({'color': color, 'content': content, 'raw': tile_data[i:i+2]})

    return tiles, facing, agent_x, agent_y


def build_scan_grid(tiles, agent_x, agent_y):
    """
    Map scanned tiles to absolute grid coordinates.
    Supported layouts:
      n = side*side        -> square grid
      n = side*side - 1    -> square grid minus agent's centre tile
      n = rows*cols        -> rectangle (most-square fit)
    """
    n = len(tiles)

    side = int(math.isqrt(n))
    if side * side == n:
        rows, cols, skip_centre = side, side, False
    elif int(math.isqrt(n + 1)) ** 2 == n + 1:
        side = int(math.isqrt(n + 1))
        rows, cols, skip_centre = side, side, True
    else:
        rows, cols, skip_centre = None, None, False
        for r in range(int(math.isqrt(n)), 0, -1):
            if n % r == 0:
                rows, cols = r, n // r
                break
        if rows is None:
            raise ValueError(f"Cannot determine scan grid shape from {n} tiles.")

    half_r, half_c = rows // 2, cols // 2
    centre_row, centre_col = rows // 2, cols // 2

    scan_grid = {}
    tile_idx = 0
    for row in range(rows):
        for col in range(cols):
            if skip_centre and row == centre_row and col == centre_col:
                continue
            abs_x = agent_x - half_c + col
            abs_y = agent_y - half_r + row
            scan_grid[(abs_x, abs_y)] = tiles[tile_idx]
            tile_idx += 1

    return scan_grid, rows, cols


def decide_action(agent_x, agent_y, facing, target_x, target_y, scan_grid):
    """Decide the next movement action toward the target."""

    # Stop only if the target tile is directly adjacent (N/S/E/W) —
    # those are the only tiles the agent can actually step onto next.
    for direction, (vx, vy) in DIRECTION_VECTORS.items():
        nx, ny = agent_x + vx, agent_y + vy
        tile = scan_grid.get((nx, ny))
        if tile and tile['content'] == 'Target':
            return None, f"Target is directly adjacent ({direction}) — holding position!"

    dx = target_x - agent_x
    dy = target_y - agent_y

    if dx == 0 and dy == 0:
        return None, "Agent is already at the target!"

    if abs(dx) >= abs(dy):
        ideal_dir = 'R' if dx > 0 else 'L'
        alt_dir   = 'D' if dy > 0 else 'U'
    else:
        ideal_dir = 'D' if dy > 0 else 'U'
        alt_dir   = 'R' if dx > 0 else 'L'

    def tile_is_passable(direction):
        vx, vy = DIRECTION_VECTORS[direction]
        nx, ny = agent_x + vx, agent_y + vy
        tile = scan_grid.get((nx, ny))
        if tile is None:
            return True  # Outside scan range — assume passable
        return tile['content'] != 'Obstacle' and tile['color'] != 'Border'

    def manhattan_after_move(direction):
        vx, vy = DIRECTION_VECTORS[direction]
        return abs((agent_x + vx) - target_x) + abs((agent_y + vy) - target_y)

    all_dirs = ['U', 'D', 'L', 'R']
    passable_dirs = [d for d in all_dirs if tile_is_passable(d)]

    if passable_dirs:
        # Pick the passable direction that gets closest to target.
        # On ties, use priority order (ideal -> alt -> rest) to break them
        # consistently rather than defaulting to an arbitrary list order.
        priority = [ideal_dir, alt_dir] + [d for d in all_dirs if d not in (ideal_dir, alt_dir)]
        priority_passable = [d for d in priority if d in passable_dirs]
        best_dist = min(manhattan_after_move(d) for d in priority_passable)
        chosen_dir = next(d for d in priority_passable if manhattan_after_move(d) == best_dist)
    else:
        chosen_dir = ideal_dir  # Fully surrounded — fallback

    if chosen_dir == facing:
        action = 'forward'
    elif chosen_dir == TURN_LEFT[facing]:
        action = 'turn_left'
    elif chosen_dir == TURN_RIGHT[facing]:
        action = 'turn_right'
    else:
        action = 'backward'

    return action, None


def print_scan_grid(scan_grid, agent_x, agent_y, target_x, target_y, rows, cols):
    half_r, half_c = rows // 2, cols // 2
    log("\n-- Scanned Grid (A=Agent, *T*=Target, [X]=Obstacle, ///=Border) --")
    for row in range(rows):
        row_str = ""
        for col in range(cols):
            abs_x = agent_x - half_c + col
            abs_y = agent_y - half_r + row
            if abs_x == agent_x and abs_y == agent_y:
                row_str += "  [A] "
            elif abs_x == target_x and abs_y == target_y:
                row_str += "  *T* "
            else:
                tile = scan_grid.get((abs_x, abs_y))
                if tile:
                    if tile['color'] == 'Border':
                        sym = '///'
                    elif tile['content'] == 'Obstacle':
                        sym = '[X]'
                    else:
                        sym = tile['raw'] + ' '
                    row_str += f"  {sym} "
                else:
                    row_str += "   ?  "
        log(row_str)
    log()


def process_node(ep, node, target_x, target_y):
    """Load and process one agent node. Returns (action_str, error_str)."""
    filepath = os.path.join(RESULTS_DIR, f"compact_map_result_Ep_{ep}_Node_{node}.txt")
    log(f"\n{'='*48}")
    log(f"  Node {node} | File: {filepath}")
    log(f"{'='*48}")

    try:
        tiles, facing, agent_x, agent_y = parse_file(filepath)
    except FileNotFoundError:
        err = f"hold"   # Safe default if file missing
        log(f"  [ERROR] File not found: {filepath} — defaulting to '{err}'")
        return err, f"File not found: {filepath}"
    except Exception as e:
        log(f"  [ERROR] Parse failed: {e}")
        return "hold", str(e)

    log(f"  Agent position : ({agent_x}, {agent_y})")
    log(f"  Agent facing   : {facing} ({FACING_NAMES[facing]})")
    log(f"  Target position: ({target_x}, {target_y})")
    log(f"  Tiles scanned  : {len(tiles)}")

    scan_grid, rows, cols = build_scan_grid(tiles, agent_x, agent_y)
    skip_centre = (rows * cols - 1 == len(tiles))
    layout = f"{rows}x{cols}" + (" (centre=agent excluded)" if skip_centre else "")
    log(f"  Scan grid shape: {layout}")

    print_scan_grid(scan_grid, agent_x, agent_y, target_x, target_y, rows, cols)

    # Tile detail listing
    half_r, half_c = rows // 2, cols // 2
    centre_row, centre_col = rows // 2, cols // 2
    log("  -- Tile Details --")
    tile_idx = 0
    for row in range(rows):
        for col in range(cols):
            if skip_centre and row == centre_row and col == centre_col:
                continue
            abs_x = agent_x - half_c + col
            abs_y = agent_y - half_r + row
            tile = tiles[tile_idx]
            log(f"    ({abs_x:>3},{abs_y:>3})  {tile['raw']}  {tile['color']:<10} {tile['content']}")
            tile_idx += 1

    action, message = decide_action(agent_x, agent_y, facing, target_x, target_y, scan_grid)

    log("\n  -- Decision --")
    if action is None:
        log(f"  {message}")
        return "hold", None
    else:
        dx = target_x - agent_x
        dy = target_y - agent_y
        log(f"  Manhattan distance to target : {abs(dx) + abs(dy)}")
        log(f"  >>> ACTION: {action.upper()}")
        return action, None


def main():
    parser = argparse.ArgumentParser(description="ML movement decision script for multi-agent missions.")
    parser.add_argument('--ep',     required=True,          help="Episode number (e.g. 1)")
    parser.add_argument('--nodes',  required=True, nargs='+', help="One or more node IDs (e.g. 120 121)")
    parser.add_argument('--target', required=True, nargs=2, type=int, metavar=('X', 'Y'),
                        help="Target grid coordinates (e.g. --target 5 5)")
    args = parser.parse_args()

    ep       = args.ep
    nodes    = args.nodes
    target_x, target_y = args.target

    log(f"\n{'='*48}")
    log(f"  Episode {ep} | Nodes: {', '.join(nodes)}")
    log(f"  Target: ({target_x}, {target_y})")
    log(f"{'='*48}")

    moves = []
    for node in nodes:
        action, _ = process_node(ep, node, target_x, target_y)
        moves.append(action)

    # Clean CSV to stdout — this is what ML_Script.sh captures
    csv_out = ','.join(moves)
    log(f"\n  CSV output → {csv_out}")
    print(csv_out)


if __name__ == "__main__":
    main()
