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
#   17 chars (original format) + "," + col + "," + row
#   Coordinates now use the bordered BIG_GRID coordinates.
#   Example: top-left physical B outputs as col=1,row=1.
#
# --- CHANGES FROM ORIGINAL ---
# 1. BIG_GRID updated to confirmed 9x11 map with X border
# 2. Added --ep and --node command line arguments
#    so each agent has its own input/output files
# 3. Output now includes ",col,row" for Mission Leader using bordered BIG_GRID coordinates
# --- END CHANGES ---
#
# HOW TO RUN:
#   python3 mapingfromtxtfile.py --ep 1 --node 120
#   python3 mapingfromtxtfile.py --ep 1 --node 121
# =========================================================

RESULTS_DIR = "Results"
SENSING_DIR = "Sensing_Files"

# =========================================================
# BIG_GRID - 9x11 MAP WITH EXPLICIT X BORDER
# =========================================================
# P = Purple
# Y = Yellow
# B = Blue
# M = Pink/Magenta
# X = Border / impassable wall
#
# Coordinates use the full BIG_GRID coordinate system.
# Rows are 0 through 8.
# Cols are 0 through 10.
# Example:
#   Top-left X border -> BIG_GRID row 0, col 0
#   Top-left B tile   -> BIG_GRID row 1, col 1
# =========================================================

BIG_GRID = [
    ["X", "X", "X", "X", "X", "X", "X", "X", "X", "X", "X"],  # Row 0
    ["X", "B", "B", "Y", "P", "B", "P", "P", "B", "M", "X"],  # Row 1
    ["X", "Y", "P", "Y", "M", "Y", "B", "B", "M", "Y", "X"],  # Row 2
    ["X", "M", "B", "M", "P", "B", "M", "P", "P", "M", "X"],  # Row 3
    ["X", "B", "M", "B", "M", "Y", "Y", "P", "Y", "B", "X"],  # Row 4
    ["X", "M", "P", "M", "P", "M", "P", "Y", "M", "P", "X"],  # Row 5
    ["X", "B", "B", "Y", "Y", "M", "Y", "Y", "P", "B", "X"],  # Row 6
    ["X", "P", "B", "P", "Y", "P", "M", "B", "Y", "Y", "X"],  # Row 7
    ["X", "X", "X", "X", "X", "X", "X", "X", "X", "X", "X"],  # Row 8
]
# BIG_GRID col:
#          0    1    2    3    4    5    6    7    8    9    10

# Edge cells have fewer real color neighbors. A corner only has 3 valid in-grid
# neighbors, so this must be 3 if corners are allowed.
MIN_KNOWN_NEIGHBORS = 3
MAX_MISMATCHES = 1

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

    allowed_colors = {"P", "Y", "B", "M", "X", "?"}
    allowed_objects = {"T", "O", "E", "?"}
    pairs = []

    for i in range(0, 16, 2):
        color_char = compact[i]
        object_char = compact[i + 1]

        if color_char not in allowed_colors:
            raise ValueError(f"Bad color '{color_char}' at index {i}")
        if object_char not in allowed_objects:
            raise ValueError(f"Bad object '{object_char}' at index {i+1}")

        pairs.append((color_char, object_char))

    return pairs


def compact_pairs_to_local_grids(pairs):
    color = [["?", "?", "?"], ["?", "A", "?"], ["?", "?", "?"]]
    obj = [["?", "?", "?"], ["?", "A", "?"], ["?", "?", "?"]]

    positions = [
        (0, 0), (0, 1), (0, 2),
        (1, 0),         (1, 2),
        (2, 0), (2, 1), (2, 2),
    ]

    for (r, c), (color_char, object_char) in zip(positions, pairs):
        color[r][c] = color_char
        obj[r][c] = object_char

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

    # Do NOT reject windows containing X.
    # X represents the virtual border outside the physical grid, so edge
    # positions can still produce a valid 3x3 match.
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
    return {"known": known, "matches": matches,
            "mismatches": mismatches, "score": matches}


def rotation_to_facing(rotation_ccw_deg):
    return {0: "UP", 90: "RIGHT", 180: "DOWN", 270: "LEFT"}[rotation_ccw_deg]


def find_best_match(local_color_3x3, big_grid):
    rows = len(big_grid)
    cols = len(big_grid[0])
    candidates = []

    for rot_steps in range(4):
        rotated = rotate_n_ccw(local_color_3x3, rot_steps)
        rotation_deg = rot_steps * 90

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
                    "center_row": center_r,
                    "center_col": center_c,
                    "rot_steps": rot_steps,
                    "rotation_ccw_deg": rotation_deg,
                    "facing_raw": rotation_to_facing(rotation_deg),
                    "known": s["known"],
                    "matches": s["matches"],
                    "mismatches": s["mismatches"],
                    "score": s["score"],
                    "matched_biggrid_window": window,
                })

    if not candidates:
        return None

    candidates.sort(
        key=lambda x: (x["known"], x["score"], -x["mismatches"]),
        reverse=True
    )
    return candidates[0]


# =========================================================
# DIRECTION HELPERS
# =========================================================

def physical_direction_fix(direction):
    # Camera faces the same direction as the agent — no inversion needed
    return direction


def rotate_direction(direction, steps_ccw):
    dirs = ["UP", "LEFT", "DOWN", "RIGHT"]
    idx = dirs.index(direction)
    return dirs[(idx + steps_ccw) % 4]


def get_scan_order(scan_start_local="FRONT", scan_sweep="cw", num_views=4):
    scan_start_local = scan_start_local.upper()
    if scan_sweep == "cw":
        base_order = ["FRONT", "RIGHT", "BACK", "LEFT"]
    else:
        base_order = ["FRONT", "LEFT", "BACK", "RIGHT"]
    start_idx = base_order.index(scan_start_local)
    ordered = base_order[start_idx:] + base_order[:start_idx]
    return ordered[:num_views]


def local_heading_to_map_direction(start_map_direction, local_heading):
    local_steps_ccw = {"FRONT": 0, "LEFT": 1, "BACK": 2, "RIGHT": 3}
    return rotate_direction(start_map_direction,
                            local_steps_ccw[local_heading.upper()])


def get_final_camera_direction_after_scan(
    start_map_direction,
    scan_start_local="FRONT",
    scan_sweep="cw",
    num_views=4
):
    order = get_scan_order(scan_start_local, scan_sweep, num_views)
    final_local = order[-1]

    # Direction the camera is at after the last scan view
    final_map_after_last_view = local_heading_to_map_direction(
        start_map_direction, final_local)

    # After scanning, the agent rotates back one step in the opposite direction
    # to return to its resting pose. CW scan = rotate back one step CW from
    # the last scan position. One CW step = 3 CCW steps in rotate_direction.
    if scan_sweep == "cw":
        rotate_back_ccw_steps = 3   # i.e. one step clockwise
    else:
        rotate_back_ccw_steps = 1   # i.e. one step counter-clockwise

    final_map = rotate_direction(final_map_after_last_view, rotate_back_ccw_steps)

    return final_local, final_map


def direction_to_char(direction):
    return {"UP": "U", "RIGHT": "R", "DOWN": "D", "LEFT": "L"}[direction]


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
            obj_char = str(object_biggrid_perspective[r][c]).strip().upper()[:1]
            if obj_char not in ["T", "O", "E", "?"]:
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

    # Use BIG_GRID coordinates directly.
    # This means the top-left X border is row 0, col 0,
    # and the top-left B tile is row 1, col 1.
    row = best["center_row"]
    col = best["center_col"]

    return compact_map_result, col, row


# =========================================================
# MAIN
# =========================================================

def main():
    # CHANGE: Added --ep and --node arguments
    # --ep   : episode number (used for file naming)
    # --node : agent node ID (e.g. 120 for agent 1, 121 for agent 2)
    parser = argparse.ArgumentParser()

    parser.add_argument("--ep", type=int, required=True,
                        help="Episode number")
    parser.add_argument("--node", type=int, required=True,
                        help="Agent node ID (e.g. 120)")

    args = parser.parse_args()

    ep = args.ep
    node = args.node

    os.makedirs(RESULTS_DIR, exist_ok=True)
    os.makedirs(SENSING_DIR, exist_ok=True)

    # Input:  Sensing_Files/Ep_{ep}_Node_{node}.txt
    # Output: Results/compact_map_result_Ep_{ep}_Node_{node}.txt
    compact_local_file = os.path.join(SENSING_DIR, f"Ep_{ep}_Node_{node}.txt")
    compact_map_file = os.path.join(RESULTS_DIR, f"compact_map_result_Ep_{ep}_Node_{node}.txt")

    compact_map_result, col, row = map_location_from_compact_local(
        compact_local_file
    )

    # Format: "17chars,col,row"
    # Coordinates are BIG_GRID coordinates.
    # Example: top-left B tile outputs col=1,row=1.
    full_output = f"{compact_map_result},{col},{row}"

    with open(compact_map_file, "w") as f:
        f.write(full_output + "\n")

    print(full_output)


if __name__ == "__main__":
    main()
