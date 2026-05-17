import os
import argparse

# =========================================================
# MAP LOCATION FROM COMPACT LOCAL RESULT
# =========================================================
# Input:
#   results/compact_local_result_{node}.txt
#
# The compact local result is 16 characters:
#   8 surrounding cells x 2 chars each = 16
#
# Each pair:
#   floor color + object state
#
# Example:
#   PEYEBEMEPEBTMEBE
#
# Output:
#   results/compact_map_result_{node}.txt
#
# The output is 20 characters:
#   17 chars (original format) + "," + row + "," + col
#   Example: "PEYEBEMEPEBTMEBEL,3,5"
#
# --- CHANGES FROM ORIGINAL ---
# 1. BIG_GRID updated to confirmed 7x9 map
# 2. Added --ep and --node command line arguments
#    so each agent has its own input/output files
# 3. Output now includes ",row,col" for Mission Leader
# --- END CHANGES ---
#
# HOW TO RUN:
#   python3 mapingfromtxtfile.py --ep 1 --node 120
#   python3 mapingfromtxtfile.py --ep 1 --node 121
# =========================================================

RESULTS_DIR = "Results"
SENSING_DIR = "Sensing_Files"

# =========================================================
# BIG GRID - CONFIRMED 7x9 MAP
# =========================================================
# P = Purple
# Y = Yellow
# B = Blue
# M = Pink/Magenta
# =========================================================

BIG_GRID = [
    ["B", "M", "Y", "P", "P", "P", "P", "B", "M"],  # Row 0
    ["Y", "P", "Y", "M", "Y", "B", "B", "B", "Y"],  # Row 1
    ["M", "B", "M", "P", "B", "M", "P", "P", "M"],  # Row 2
    ["B", "M", "B", "M", "Y", "Y", "P", "Y", "B"],  # Row 3
    ["M", "P", "M", "B", "M", "P", "Y", "M", "P"],  # Row 4
    ["B", "B", "Y", "Y", "M", "Y", "Y", "P", "B"],  # Row 5
    ["P", "B", "P", "M", "P", "M", "B", "Y", "Y"],  # Row 6
]

MIN_KNOWN_NEIGHBORS = 5
MAX_MISMATCHES = 3

SCAN_START_LOCAL = "FRONT"
SCAN_SWEEP = "cw"
NUM_VIEWS = 4


# =========================================================
# COMPACT LOCAL INPUT
# =========================================================

def read_compact_local_result(path):
    if not os.path.exists(path):
        raise FileNotFoundError(f"Could not find file: {path}")

    with open(path, "r") as f:
        compact = f.read().strip().replace(" ", "").replace("\n", "")

    if len(compact) != 16:
        raise ValueError(
            f"Expected 16 characters, got {len(compact)}: {compact}"
        )

    compact = compact.upper()

    allowed_colors  = {"P", "Y", "B", "M", "?"}
    allowed_objects = {"T", "O", "E", "?"}
    pairs = []

    for i in range(0, 16, 2):
        color_char  = compact[i]
        object_char = compact[i + 1]

        if color_char not in allowed_colors:
            raise ValueError(f"Bad color '{color_char}' at index {i}")
        if object_char not in allowed_objects:
            raise ValueError(f"Bad object '{object_char}' at index {i+1}")

        pairs.append((color_char, object_char))

    return pairs


def compact_pairs_to_local_grids(pairs):
    color = [["?","?","?"],["?","A","?"],["?","?","?"]]
    obj   = [["?","?","?"],["?","A","?"],["?","?","?"]]

    positions = [
        (0,0),(0,1),(0,2),
        (1,0),     (1,2),
        (2,0),(2,1),(2,2),
    ]

    for (r,c),(color_char,object_char) in zip(positions, pairs):
        color[r][c] = color_char
        obj[r][c]   = object_char

    return color, obj


# =========================================================
# ROTATION HELPERS
# =========================================================

def rotate_3x3_ccw(mat):
    return [
        [mat[0][2], mat[1][2], mat[2][2]],
        [mat[0][1], mat[1][1], mat[2][1]],
        [mat[0][0], mat[1][0], mat[2][0]],
    ]

def rotate_n_ccw(mat, n):
    out = [row[:] for row in mat]
    for _ in range(n % 4):
        out = rotate_3x3_ccw(out)
    return out


# =========================================================
# BIG GRID MATCHING
# =========================================================

def get_window_3x3(grid, center_r, center_c):
    rows = len(grid)
    cols = len(grid[0])

    if center_r - 1 < 0 or center_r + 1 >= rows:
        return None
    if center_c - 1 < 0 or center_c + 1 >= cols:
        return None

    window = [
        [grid[center_r-1][center_c-1], grid[center_r-1][center_c], grid[center_r-1][center_c+1]],
        [grid[center_r][center_c-1],   "A",                        grid[center_r][center_c+1]],
        [grid[center_r+1][center_c-1], grid[center_r+1][center_c], grid[center_r+1][center_c+1]],
    ]

    for r in range(3):
        for c in range(3):
            if window[r][c] == "X":
                return None

    return window


def score_match(local_3x3, window_3x3):
    known = matches = mismatches = 0
    for r in range(3):
        for c in range(3):
            lv = local_3x3[r][c]
            wv = window_3x3[r][c]
            if lv in ("A", "?"):
                continue
            known += 1
            if lv == wv:
                matches += 1
            else:
                mismatches += 1
    return {"known":known,"matches":matches,
            "mismatches":mismatches,"score":matches}


def rotation_to_facing(rotation_ccw_deg):
    return {0:"UP",90:"RIGHT",180:"DOWN",270:"LEFT"}[rotation_ccw_deg]


def find_best_match(local_color_3x3, big_grid):
    rows = len(big_grid)
    cols = len(big_grid[0])
    candidates = []

    for rot_steps in range(4):
        rotated       = rotate_n_ccw(local_color_3x3, rot_steps)
        rotation_deg  = rot_steps * 90

        for center_r in range(1, rows-1):
            for center_c in range(1, cols-1):
                window = get_window_3x3(big_grid, center_r, center_c)
                if window is None:
                    continue
                s = score_match(rotated, window)
                if s["known"] < MIN_KNOWN_NEIGHBORS:
                    continue
                if s["mismatches"] > MAX_MISMATCHES:
                    continue
                candidates.append({
                    "center_row":            center_r,
                    "center_col":            center_c,
                    "rot_steps":             rot_steps,
                    "rotation_ccw_deg":      rotation_deg,
                    "facing_raw":            rotation_to_facing(rotation_deg),
                    "known":                 s["known"],
                    "matches":               s["matches"],
                    "mismatches":            s["mismatches"],
                    "score":                 s["score"],
                    "matched_biggrid_window":window,
                })

    if not candidates:
        return None

    candidates.sort(
        key=lambda x: (x["score"], -x["mismatches"], x["known"]),
        reverse=True
    )
    return candidates[0]


# =========================================================
# DIRECTION HELPERS
# =========================================================

def physical_direction_fix(direction):
    return {"UP":"DOWN","DOWN":"UP","RIGHT":"RIGHT","LEFT":"LEFT"}[direction]


def rotate_direction(direction, steps_ccw):
    dirs = ["UP","LEFT","DOWN","RIGHT"]
    idx  = dirs.index(direction)
    return dirs[(idx + steps_ccw) % 4]


def get_scan_order(scan_start_local="FRONT", scan_sweep="cw", num_views=4):
    scan_start_local = scan_start_local.upper()
    if scan_sweep == "cw":
        base_order = ["FRONT","RIGHT","BACK","LEFT"]
    else:
        base_order = ["FRONT","LEFT","BACK","RIGHT"]
    start_idx = base_order.index(scan_start_local)
    ordered   = base_order[start_idx:] + base_order[:start_idx]
    return ordered[:num_views]


def local_heading_to_map_direction(start_map_direction, local_heading):
    local_steps_ccw = {"FRONT":0,"LEFT":1,"BACK":2,"RIGHT":3}
    return rotate_direction(start_map_direction,
                            local_steps_ccw[local_heading.upper()])


def get_final_camera_direction_after_scan(
    start_map_direction,
    scan_start_local="FRONT",
    scan_sweep="cw",
    num_views=4
):
    order            = get_scan_order(scan_start_local, scan_sweep, num_views)
    final_local      = order[-1]
    final_map        = local_heading_to_map_direction(
                           start_map_direction, final_local)
    return final_local, final_map


def direction_to_char(direction):
    return {"UP":"U","RIGHT":"R","DOWN":"D","LEFT":"L"}[direction]


# =========================================================
# COMPACT MAP OUTPUT
# =========================================================

def build_compact_17char(matched_biggrid_window,
                         object_biggrid_perspective,
                         final_direction_physical):
    out = []
    for r in range(3):
        for c in range(3):
            if r == 1 and c == 1:
                continue
            floor_char = str(matched_biggrid_window[r][c]).strip().upper()[:1]
            obj_char   = str(object_biggrid_perspective[r][c]).strip().upper()[:1]
            if obj_char not in ["T","O","E","?"]:
                obj_char = "?"
            out.append(floor_char + obj_char)
    out.append(direction_to_char(final_direction_physical))
    return "".join(out)


# =========================================================
# MAIN LOCALIZATION
# =========================================================

def map_location_from_compact_local(compact_local_file):
    pairs = read_compact_local_result(compact_local_file)
    local_color_3x3, local_object_3x3 = compact_pairs_to_local_grids(pairs)

    best = find_best_match(local_color_3x3, BIG_GRID)

    if best is None:
        raise RuntimeError("No valid BIG_GRID match found.")

    camera_direction_before_scan_raw = best["facing_raw"]

    _, camera_direction_after_scan_raw = get_final_camera_direction_after_scan(
        start_map_direction=camera_direction_before_scan_raw,
        scan_start_local=SCAN_START_LOCAL,
        scan_sweep=SCAN_SWEEP,
        num_views=NUM_VIEWS
    )

    camera_direction_after_scan_physical = physical_direction_fix(
        camera_direction_after_scan_raw
    )

    object_biggrid_perspective = rotate_n_ccw(
        local_object_3x3, best["rot_steps"]
    )

    compact_map_result = build_compact_17char(
        best["matched_biggrid_window"],
        object_biggrid_perspective,
        camera_direction_after_scan_physical
    )

    row = best["center_row"]
    col = best["center_col"]

    return compact_map_result, row, col


# =========================================================
# MAIN
# =========================================================

def main():
    # CHANGE: Added --ep and --node arguments
    # --ep   : episode number (used for file naming)
    # --node : agent node ID (e.g. 120 for agent 1, 121 for agent 2)
    parser = argparse.ArgumentParser()
    
    parser.add_argument("--ep",   type=int, default="",
                        help="Episode number")
    parser.add_argument("--node", type=int, default="",
                        help="Agent node ID (e.g. 120)")
                        
    args = parser.parse_args()
    
    ep = args.ep
    node = args.node

    os.makedirs(RESULTS_DIR, exist_ok=True)
    os.makedirs(SENSING_DIR, exist_ok=True)

    # Input:  Ep_{ep}_Node_{node}.txt
    # Output: results/compact_map_result_{node}.txt
    if args.node:
        compact_local_file = os.path.join(SENSING_DIR, f"Ep_{ep}_Node_{node}.txt")
        compact_map_file   = os.path.join(
            RESULTS_DIR, f"compact_map_result_Ep_{ep}_Node_{node}.txt")
    else:
        # Fallback: original filenames if no node given
        compact_local_file = os.path.join(
            SENSING_DIR, "Ep_1_Node_120.txt")
        compact_map_file   = os.path.join(
            RESULTS_DIR, "compact_map_result.txt")

    compact_map_result, row, col = map_location_from_compact_local(
        compact_local_file
    )

    # Format: "17chars,row,col"
    # Example: "PEYEBEMEPEBTMEBEL,3,5"
    full_output = f"{compact_map_result},{row},{col}"

    with open(compact_map_file, "w") as f:
        f.write(full_output + "\n")

    print(full_output)


if __name__ == "__main__":
    main()
