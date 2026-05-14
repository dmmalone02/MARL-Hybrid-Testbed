"""
=============================================================
ML DECIDE - Q-Learning Brain for MARL Testbed
Team 4 Senior Design 2025-2026

Reads compact_map_result files for all agents,
uses Q-learning to decide the next move for each agent,
and outputs movement commands.

HOW TO RUN (called by ML_Script.sh):
  python3 ml_decide.py --ep 1 --nodes 120 121

OUTPUT (printed to stdout, one line):
  forward,turn_left
  (move for agent 1, move for agent 2 separated by comma)

MOVEMENT COMMANDS:
  forward    = move forward 1 block (no rotation)
  backward   = move backward 1 block (no rotation)
  turn_right = rotate 90° right AND move forward 1 block
  turn_left  = rotate 90° left  AND move forward 1 block

FACING DIRECTION (from compact_map_result file):
  U = facing North (row decreases when moving forward)
  D = facing South (row increases when moving forward)
  R = facing East  (col increases when moving forward)
  L = facing West  (col decreases when moving forward)
=============================================================
"""

import os
import sys
import argparse
import pickle
import random
import numpy as np

RESULTS_DIR = "Results"
QTABLE_PATH = "q_tables.pkl"

GRID_ROWS = 7
GRID_COLS = 9

# Map directions
MAP_MOVES = {
    'N': (-1,  0),
    'S': ( 1,  0),
    'E': ( 0,  1),
    'W': ( 0, -1),
}

# Convert desired map direction to agent command based on current facing
# facing -> desired_map_dir -> command
# turn_right = rotate 90° right + move forward
# turn_left  = rotate 90° left  + move forward
FACING_TO_CMD = {
    'U': {'N':'forward',    'S':'backward', 'E':'turn_right','W':'turn_left'},
    'D': {'N':'backward',   'S':'forward',  'E':'turn_left', 'W':'turn_right'},
    'R': {'N':'turn_left',  'S':'turn_right','E':'forward',  'W':'backward'},
    'L': {'N':'turn_right', 'S':'turn_left', 'E':'backward', 'W':'forward'},
}

# After each command, what is the new facing direction?
NEW_FACING = {
    'U': {'forward':'U','backward':'U','turn_right':'R','turn_left':'L'},
    'D': {'forward':'D','backward':'D','turn_right':'L','turn_left':'R'},
    'R': {'forward':'R','backward':'R','turn_right':'D','turn_left':'U'},
    'L': {'forward':'L','backward':'L','turn_right':'U','turn_left':'D'},
}

# =============================================================
# LOAD Q-TABLES
# =============================================================
def load_qtables():
    if not os.path.exists(QTABLE_PATH):
        return None
    with open(QTABLE_PATH, 'rb') as f:
        data = pickle.load(f)
    return data.get('q_tables', None)

# =============================================================
# READ COMPACT MAP RESULT FOR ONE AGENT
# Returns (row, col, facing) or None
# =============================================================
def read_agent_result(node):
    path = os.path.join(RESULTS_DIR, f"compact_map_result_Ep_{ep}_Node_{node}.txt")
    if not os.path.exists(path):
        return None
    with open(path, 'r') as f:
        content = f.read().strip()
    # Format: "17chars,row,col"
    # Example: "PEYEBEMEPEBTMEBEL,3,5"
    parts = content.split(',')
    if len(parts) < 3:
        return None
    try:
        compact_17 = parts[0]   # 17-char string
        row        = int(parts[1])
        col        = int(parts[2])
        facing     = compact_17[-1].upper()  # last char = U/D/L/R
        if facing not in ('U','D','L','R'):
            facing = 'U'        # fallback
        return row, col, facing
    except (ValueError, IndexError):
        return None

# =============================================================
# Q-LEARNING STATE AND DECISION
# =============================================================
DIRS_ENCODE = ['N','NE','E','SE','S','SW','W','NW','NONE']

def encode_direction(from_pos, to_pos):
    if to_pos is None:
        return 'NONE'
    dr = to_pos[0] - from_pos[0]
    dc = to_pos[1] - from_pos[1]
    if dr == 0 and dc == 0:
        return 'NONE'
    angle = np.degrees(np.arctan2(dc, -dr)) % 360
    idx   = int((angle + 22.5) / 45) % 8
    return ['N','NE','E','SE','S','SW','W','NW'][idx]

def nearest_unexplored(pos, explored):
    best_pos = None
    best_d   = float('inf')
    for r in range(GRID_ROWS):
        for c in range(GRID_COLS):
            if (r,c) not in explored:
                d = abs(r-pos[0]) + abs(c-pos[1])
                if d < best_d:
                    best_d = d
                    best_pos = (r,c)
    return best_pos

def get_valid_map_dirs(pos):
    valid = []
    for d,(dr,dc) in MAP_MOVES.items():
        nr,nc = pos[0]+dr, pos[1]+dc
        if 0 <= nr < GRID_ROWS and 0 <= nc < GRID_COLS:
            valid.append(d)
    return valid

def heuristic(a, b):
    return abs(a[0]-b[0]) + abs(a[1]-b[1])

def q_decide_map_dir(agent_idx, agent_pos, explored,
                     ally_positions, q_tables):
    """
    Returns best MAP direction (N/S/E/W) for this agent.
    Uses Q-table if available, otherwise navigates to nearest unexplored.
    """
    valid = get_valid_map_dirs(agent_pos)
    if not valid:
        return None

    # Find nearest unexplored cell
    nearest = nearest_unexplored(agent_pos, explored)

    # Find nearest ally
    if ally_positions:
        nearest_ally = min(
            ally_positions,
            key=lambda p: heuristic(agent_pos, p)
        )
        dir_ally = encode_direction(agent_pos, nearest_ally)
    else:
        dir_ally = 'NONE'

    dir_unexp = encode_direction(agent_pos, nearest)
    state     = (agent_pos[0], agent_pos[1], dir_unexp, dir_ally)

    # Try Q-table first
    if q_tables and agent_idx < len(q_tables):
        q_table = q_tables[agent_idx]
        # Q-table uses Forward/Backward/Left/Right as action names
        action_to_mapdir = {
            'Forward':'N', 'Backward':'S',
            'Left':'W',    'Right':'E'
        }
        mapdir_to_action = {v:k for k,v in action_to_mapdir.items()}

        if state in q_table:
            # Pick best action that results in a valid map direction
            valid_actions = [mapdir_to_action[d] for d in valid
                             if d in mapdir_to_action]
            if valid_actions:
                best_action = max(
                    valid_actions,
                    key=lambda a: q_table[state].get(a, 0.0)
                )
                return action_to_mapdir[best_action]

    # Fallback: move toward nearest unexplored cell
    if nearest:
        return min(valid, key=lambda d:
                   heuristic((agent_pos[0]+MAP_MOVES[d][0],
                              agent_pos[1]+MAP_MOVES[d][1]),
                             nearest))

    return random.choice(valid)

# =============================================================
# CONVERT MAP DIRECTION TO AGENT COMMAND
# =============================================================
def map_dir_to_command(map_dir, facing):
    """
    Convert map direction + current facing to movement command.
    Returns: forward / backward / turn_right / turn_left
    """
    if facing not in FACING_TO_CMD:
        facing = 'U'
    if map_dir not in FACING_TO_CMD[facing]:
        return 'forward'
    return FACING_TO_CMD[facing][map_dir]

# =============================================================
# MAIN
# =============================================================
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--ep',    type=str, default='',
                        help='Episode number')
    parser.add_argument('--nodes', type=str, nargs='+', required=True,
                        help='Agent node IDs (e.g. 120 121)')
    args = parser.parse_args()

    nodes     = args.nodes
    q_tables  = load_qtables()
    explored  = set()
    positions = []
    facings   = []

    # Read all agent results
    for node in nodes:
        result = read_agent_result(node)
        if result:
            row, col, facing = result
            positions.append((row, col))
            facings.append(facing)
            explored.add((row, col))
        else:
            # Agent not localized - add placeholder
            positions.append(None)
            facings.append('U')
            sys.stderr.write(f"[WARN] Could not read result for node {node}\n")

    # Decide move for each agent
    commands = []
    for i, node in enumerate(nodes):
        agent_pos = positions[i]
        facing    = facings[i]

        if agent_pos is None:
            commands.append('forward')
            continue

        # Other agent positions (allies)
        ally_positions = [p for j,p in enumerate(positions)
                          if j != i and p is not None]

        # Q-learning decides map direction
        map_dir = q_decide_map_dir(
            i, agent_pos, explored, ally_positions, q_tables
        )

        if map_dir is None:
            commands.append('forward')
            continue

        # Convert to agent command
        cmd = map_dir_to_command(map_dir, facing)
        commands.append(cmd)

        sys.stderr.write(
            f"[ML] Node {node}: pos=({agent_pos[0]},{agent_pos[1]}) "
            f"facing={facing} map_dir={map_dir} cmd={cmd}\n"
        )

    # Output: one command per agent, comma separated
    # e.g. "forward,turn_left"
    print(','.join(commands))

if __name__ == '__main__':
    main()
