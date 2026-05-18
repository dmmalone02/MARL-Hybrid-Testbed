import os
import time
import subprocess
import cv2
import numpy as np

# =========================================================
# TEAM 4 SD26 FULL SENSING RUNNER
# =========================================================
# Pipeline:
#   1. Capture 4 still images: front, right, back, left
#   2. Detect floor colors: P/Y/B/M/X
#   3. Detect objects: green target, red obstacle
#   4. Build compact local sensing result
#   5. Save and print compact_local_map_result.txt only
# =========================================================

# =========================================================
# PATHS
# =========================================================

SCAN_DIR = "scan_images"
RESULTS_DIR = "/home/ucanlab/Sensing/results"
COMPACT_RESULT_FILE = os.path.join(RESULTS_DIR, "compact_local_map_result.txt")

# =========================================================
# CAMERA CONFIG
# =========================================================

CAMERA_INDEX = 0
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

HEADINGS = ["front", "right", "back", "left"]

# Number of consecutive still frames required before saving
STILL_NEEDED = 3
WARM_FRAMES = 12
# Motion threshold: mean pixel difference below this = still
MOTION_THRESH = 4.0

# Small delay between frame reads
CAPTURE_DELAY_SEC = 0.3

# =========================================================
# ROI CONFIG
# =========================================================

ROI_TOP_FRAC = 0.55
ROI_BOT_FRAC = 0.95

SLOT_PAD_X_FRAC = 0.03
SLOT_PAD_Y_FRAC = 0.06

HEADING_TO_POSITIONS = {
    "front": [(-1, +1), (0, +1), (+1, +1)],
    "right": [(+1, +1), (+1, 0), (+1, -1)],
    "back":  [(+1, -1), (0, -1), (-1, -1)],
    "left":  [(-1, -1), (-1, 0), (-1, +1)],
}

# =========================================================
# OBJECT DETECTION CONFIG
# =========================================================
# Green box = Target
# Red box   = Obstacle
# Red border tile = X floor / obstacle boundary
# Pink/magenta tile is rejected as fake red.
# =========================================================

GREEN_S_MIN = 70
GREEN_V_MIN = 70

RED_S_MIN = 110
RED_V_MIN = 80

RED_HUE1_LOW = 0
RED_HUE1_HIGH = 8
RED_HUE2_LOW = 176
RED_HUE2_HIGH = 180

PINK_HUE_LOW = 145
PINK_HUE_HIGH = 175
PINK_S_MIN = 60
PINK_V_MIN = 50
PINK_REJECT_RATIO = 0.20

MIN_OBJECT_RATIO = 0.025
MIN_CONTOUR_AREA_FRAC = 0.010
MAX_CONTOUR_AREA_FRAC = 0.65

MIN_OBJECT_W_FRAC = 0.08
MIN_OBJECT_H_FRAC = 0.08

MAX_CENTER_OFFSET = 0.95

# Red physical border tile handling.
# If a floor slot is mostly red, classify its floor color as X so
# localization can match the X border in mapingfromtxtfile.py.
RED_BORDER_FLOOR_RATIO = 0.35

# If a slot is mostly red, also treat it as blocked for movement/object output.
RED_BORDER_OBJECT_RATIO = 0.35

# =========================================================
# GENERAL HELPERS
# =========================================================

def ensure_clean_dirs():
    os.makedirs(SCAN_DIR, exist_ok=True)
    os.makedirs(RESULTS_DIR, exist_ok=True)

    for heading in HEADINGS:
        path = os.path.join(SCAN_DIR, f"{heading}.jpg")
        if os.path.exists(path):
            os.remove(path)

    if os.path.exists(COMPACT_RESULT_FILE):
        os.remove(COMPACT_RESULT_FILE)


def call_move(cmd, angle, speed):
    print(f"[MOVE] {cmd} {angle} deg @ {speed}")
    subprocess.run(["./move_agent.sh", cmd, str(angle), str(speed)])


def is_still(prev_gray, gray):
    diff = cv2.absdiff(prev_gray, gray)
    return np.mean(diff) < MOTION_THRESH


def make_empty_local_grid():
    return {
        (-1, +1): "?",
        (0, +1): "?",
        (+1, +1): "?",
        (-1, 0): "?",
        (0, 0): "A",
        (+1, 0): "?",
        (-1, -1): "?",
        (0, -1): "?",
        (+1, -1): "?",
    }


def local_grid_to_matrix(final_grid):
    rows = []
    for row in [1, 0, -1]:
        vals = []
        for col in [-1, 0, 1]:
            vals.append(final_grid.get((col, row), "?"))
        rows.append(vals)
    return rows


# =========================================================
# STEP 1: SCAN CAPTURE
# =========================================================

def capture_frame(cap):
    """
    Read one frame from the camera.
    Returns (gray, frame) or (None, None) on failure.
    """
    ret, frame = cap.read()
    if not ret or frame is None:
        return None, None
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    return gray, frame


def capture_scan_images():
    """
    Autonomous capture using stillness detection.
    Rotates robot between captures.
    """
    cap = cv2.VideoCapture(CAMERA_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

    if not cap.isOpened():
        raise RuntimeError("Could not open camera.")

    try:
        for heading in HEADINGS:
            print(f"\n[SCAN] {heading.upper()}")

            still_frames = 0
            prev_gray = None
            frame = None

            # Warmup frames (camera auto-adjust)
            for _ in range(WARM_FRAMES):
                ret, frame = cap.read()
                if not ret or frame is None:
                    raise RuntimeError(f"Failed to read frame from heading: {heading}")

            # Wait for stillness
            start_time = time.time()
            while still_frames < STILL_NEEDED:
                gray, frame = capture_frame(cap)
                if gray is None:
                    continue

                if prev_gray is not None:
                    if is_still(prev_gray, gray):
                        still_frames += 1
                    else:
                        still_frames = 0

                prev_gray = gray
                print(f"Still frames: {still_frames}/{STILL_NEEDED}", end="\r")
                time.sleep(CAPTURE_DELAY_SEC)

                if time.time() - start_time > 10:
                    print(f"Stillness timeout for heading {heading}, capturing image anyway.")
                    break

            print("Captured.")

            # Save image (ONLY once, after stillness)
            filename = os.path.join(SCAN_DIR, f"{heading}.jpg")
            if not cv2.imwrite(filename, frame):
                raise RuntimeError(f"Failed to save image: {filename}")

            # Rotate robot except after last capture
            if heading != HEADINGS[-1]:
                call_move("rotate", -90.0, 0.3)
                time.sleep(3)

    finally:
        cap.release()
        print("\nAll captures complete. Restoring original orientation...")
        call_move("rotate", -90.0, 0.3)
        time.sleep(0.1)


# =========================================================
# ROI EXTRACTION
# =========================================================

def get_three_slot_rois(img):
    h, w = img.shape[:2]

    y0 = int(ROI_TOP_FRAC * h)
    y1 = int(ROI_BOT_FRAC * h)

    if y1 <= y0:
        return []

    band = img[y0:y1, :]
    bh, bw = band.shape[:2]

    slots = []
    for i in range(3):
        sx0 = int(i * bw / 3)
        sx1 = int((i + 1) * bw / 3)

        pad_x = int(SLOT_PAD_X_FRAC * (sx1 - sx0))
        pad_y = int(SLOT_PAD_Y_FRAC * bh)

        cx0 = max(0, sx0 + pad_x)
        cx1 = min(bw, sx1 - pad_x)
        cy0 = max(0, pad_y)
        cy1 = min(bh, bh - pad_y)

        crop = band[cy0:cy1, cx0:cx1]
        slots.append(crop)

    return slots


def center_crop(img, frac=0.55):
    h, w = img.shape[:2]

    y0 = int((1.0 - frac) * 0.5 * h)
    y1 = int(h - y0)
    x0 = int((1.0 - frac) * 0.5 * w)
    x1 = int(w - x0)

    return img[y0:y1, x0:x1]


# =========================================================
# STEP 2: FLOOR COLOR DETECTION
# =========================================================

def classify_floor_color_opencv(tile_bgr):
    """
    Detects floor colors:
        P = purple
        Y = yellow
        B = blue
        M = pink/magenta
        X = red physical border tile

    Red and green are still used in object detection, but a mostly red FLOOR
    slot is treated as X so edge localization can match the X border.
    """
    if tile_bgr is None or tile_bgr.size == 0:
        return "?"

    roi = center_crop(tile_bgr, 0.55)
    if roi.size == 0:
        return "?"

    roi = cv2.GaussianBlur(roi, (5, 5), 0)

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    lab = cv2.cvtColor(roi, cv2.COLOR_BGR2LAB)

    H = hsv[:, :, 0]
    S = hsv[:, :, 1]
    V = hsv[:, :, 2]
    A = lab[:, :, 1]
    LB = lab[:, :, 2]

    valid = (S >= 45) & (V >= 45)
    valid_ratio = float(np.count_nonzero(valid)) / float(valid.size)

    if valid_ratio < 0.10:
        return "?"

    # Red physical border detection.
    # This happens before yellow/blue/purple/magenta classification so
    # strong red does not accidentally become a random floor color.
    red_border_mask_1 = (
        (H >= RED_HUE1_LOW) &
        (H <= RED_HUE1_HIGH) &
        (S >= RED_S_MIN) &
        (V >= RED_V_MIN)
    )
    red_border_mask_2 = (
        (H >= RED_HUE2_LOW) &
        (H <= RED_HUE2_HIGH) &
        (S >= RED_S_MIN) &
        (V >= RED_V_MIN)
    )
    red_border_mask = (red_border_mask_1 | red_border_mask_2) & valid
    red_border_ratio = float(np.count_nonzero(red_border_mask)) / float(valid.size)

    if red_border_ratio >= RED_BORDER_FLOOR_RATIO:
        return "X"

    yellow_mask = ((H >= 18) & (H <= 38) & valid)
    blue_mask = ((H >= 95) & (H <= 135) & valid)
    purple_pink_mask = ((H >= 136) & (H <= 169) & valid)

    ratios = {
        "yellow": float(np.count_nonzero(yellow_mask)) / float(valid.size),
        "blue": float(np.count_nonzero(blue_mask)) / float(valid.size),
        "purple_pink": float(np.count_nonzero(purple_pink_mask)) / float(valid.size),
    }

    h_mean = float(np.mean(H[valid]))
    v_mean = float(np.mean(V[valid]))
    a_mean = float(np.mean(A[valid]))
    lb_mean = float(np.mean(LB[valid]))

    best_basic = max(ratios, key=lambda k: ratios[k])
    best_ratio = ratios[best_basic]

    if best_ratio >= 0.15:
        if best_basic == "yellow":
            return "Y"

        if best_basic == "blue":
            return "B"

        if best_basic == "purple_pink":
            # Pink/magenta tends to be brighter/warmer than purple.
            if v_mean >= 125 or lb_mean >= 145:
                return "M"
            else:
                return "P"

    # Fallback for pink/purple if hue mask is weak.
    if a_mean >= 145 and lb_mean >= 135 and v_mean >= 105:
        return "M"

    if a_mean >= 140 and lb_mean < 140:
        return "P"

    # Extra fallback by hue average.
    if 18 <= h_mean <= 38:
        return "Y"

    if 95 <= h_mean <= 135:
        return "B"

    return "?"


def detect_floor_colors_from_images():
    local_grid = make_empty_local_grid()

    for heading in HEADINGS:
        path = os.path.join(SCAN_DIR, f"{heading}.jpg")

        img = cv2.imread(path)
        if img is None:
            raise RuntimeError(f"Could not read image: {path}")

        slots = get_three_slot_rois(img)
        if len(slots) != 3:
            raise RuntimeError(f"Could not build 3 slots for heading: {heading}")

        for i, tile in enumerate(slots):
            ch = classify_floor_color_opencv(tile)
            pos = HEADING_TO_POSITIONS[heading][i]
            local_grid[pos] = ch

    return local_grid_to_matrix(local_grid)


# =========================================================
# STEP 3: OBJECT DETECTION
# =========================================================

def clean_mask(mask):
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def build_red_green_pink_masks(slot_bgr):
    hsv = cv2.cvtColor(slot_bgr, cv2.COLOR_BGR2HSV)

    H = hsv[:, :, 0]
    S = hsv[:, :, 1]
    V = hsv[:, :, 2]

    red_mask_1 = (
        (H >= RED_HUE1_LOW) &
        (H <= RED_HUE1_HIGH) &
        (S >= RED_S_MIN) &
        (V >= RED_V_MIN)
    )
    red_mask_2 = (
        (H >= RED_HUE2_LOW) &
        (H <= RED_HUE2_HIGH) &
        (S >= RED_S_MIN) &
        (V >= RED_V_MIN)
    )
    red_mask = (red_mask_1 | red_mask_2).astype(np.uint8) * 255

    green_mask = (
        (H >= 40) &
        (H <= 90) &
        (S >= GREEN_S_MIN) &
        (V >= GREEN_V_MIN)
    ).astype(np.uint8) * 255

    pink_reject_mask = (
        (H >= PINK_HUE_LOW) &
        (H <= PINK_HUE_HIGH) &
        (S >= PINK_S_MIN) &
        (V >= PINK_V_MIN)
    ).astype(np.uint8) * 255

    red_mask = clean_mask(red_mask)
    green_mask = clean_mask(green_mask)
    pink_reject_mask = clean_mask(pink_reject_mask)

    return red_mask, green_mask, pink_reject_mask


def get_largest_valid_blob(mask, slot_shape):
    h, w = slot_shape[:2]
    slot_area = h * w

    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    best = None
    best_area = 0.0

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area <= 0:
            continue

        area_frac = area / float(slot_area)
        if area_frac < MIN_CONTOUR_AREA_FRAC:
            continue
        if area_frac > MAX_CONTOUR_AREA_FRAC:
            continue

        x, y, bw, bh = cv2.boundingRect(cnt)
        w_frac = bw / float(w)
        h_frac = bh / float(h)

        if w_frac < MIN_OBJECT_W_FRAC:
            continue
        if h_frac < MIN_OBJECT_H_FRAC:
            continue

        cx = x + bw / 2.0
        cy = y + bh / 2.0
        center_dx = abs(cx - w / 2.0) / max(1.0, w / 2.0)
        center_dy = abs(cy - h / 2.0) / max(1.0, h / 2.0)

        if center_dx > MAX_CENTER_OFFSET:
            continue
        if center_dy > MAX_CENTER_OFFSET:
            continue

        if area > best_area:
            best_area = area
            best = {
                "area": area,
                "area_frac": area_frac,
                "bbox": (x, y, bw, bh),
            }

    return best


def detect_one_object_slot(slot_bgr):
    """
    Return:
        T = green target
        O = red obstacle / red border
        E = empty
        ? = uncertain
    """
    if slot_bgr is None or slot_bgr.size == 0:
        return "?"

    red_mask, green_mask, pink_reject_mask = build_red_green_pink_masks(slot_bgr)

    red_ratio = float(np.count_nonzero(red_mask)) / float(red_mask.size)
    green_ratio = float(np.count_nonzero(green_mask)) / float(green_mask.size)
    pink_ratio = float(np.count_nonzero(pink_reject_mask)) / float(pink_reject_mask.size)

    red_blob = get_largest_valid_blob(red_mask, slot_bgr.shape)
    green_blob = get_largest_valid_blob(green_mask, slot_bgr.shape)

    # A mostly red slot is the physical boundary, so movement should treat it
    # as blocked even if the contour filter is not perfect.
    if red_ratio >= RED_BORDER_OBJECT_RATIO:
        return "O"

    red_detected = red_ratio >= MIN_OBJECT_RATIO and red_blob is not None
    green_detected = green_ratio >= MIN_OBJECT_RATIO and green_blob is not None

    # Reject fake red from pink/magenta floor tile.
    if pink_ratio >= PINK_REJECT_RATIO and red_ratio < 0.08:
        red_detected = False
    if pink_ratio > (red_ratio * 2.5) and red_ratio < 0.12:
        red_detected = False

    if red_detected and green_detected:
        return "O" if red_ratio > green_ratio else "T"

    if red_detected:
        return "O"
    if green_detected:
        return "T"

    return "E"


def detect_objects_from_images():
    local_grid = make_empty_local_grid()

    for heading in HEADINGS:
        path = os.path.join(SCAN_DIR, f"{heading}.jpg")

        img = cv2.imread(path)
        if img is None:
            raise RuntimeError(f"Could not read image: {path}")

        slots = get_three_slot_rois(img)
        if len(slots) != 3:
            raise RuntimeError(f"Could not build 3 slots for heading: {heading}")

        for i, tile in enumerate(slots):
            obj = detect_one_object_slot(tile)
            pos = HEADING_TO_POSITIONS[heading][i]
            local_grid[pos] = obj

    return local_grid_to_matrix(local_grid)


# =========================================================
# STEP 4: LOCAL COMPACT OUTPUT
# =========================================================

def build_compact_local_16char(local_color_3x3, local_object_3x3):
    """
    Builds compact local result:
        8 surrounding cells x 2 chars = 16 chars

    Each non-center cell:
        floor color + object state

    Center A is skipped.

    Order:
        top-left, top-middle, top-right,
        middle-left, middle-right,
        bottom-left, bottom-middle, bottom-right
    """
    out = []

    for r in range(3):
        for c in range(3):
            if r == 1 and c == 1:
                continue

            color_char = str(local_color_3x3[r][c]).strip().upper()[:1]
            obj_char = str(local_object_3x3[r][c]).strip().upper()[:1]

            if color_char not in ["P", "Y", "B", "M", "X", "?"]:
                color_char = "?"

            if obj_char not in ["T", "O", "E", "?"]:
                obj_char = "?"

            # If the floor color is X, the first character is what matters for
            # localization. Keeping O is useful for movement logic because it
            # says this neighbor is blocked.
            if color_char == "X" and obj_char == "?":
                obj_char = "O"

            out.append(color_char + obj_char)

    return "".join(out)


# =========================================================
# MAIN
# =========================================================

def main():
    ensure_clean_dirs()
    capture_scan_images()

    local_color_3x3 = detect_floor_colors_from_images()
    local_object_3x3 = detect_objects_from_images()

    compact_local_result = build_compact_local_16char(
        local_color_3x3,
        local_object_3x3
    )

    with open(COMPACT_RESULT_FILE, "w") as f:
        f.write(compact_local_result + "\n\n")

    # Print only the compact local result.
    print(compact_local_result)


if __name__ == "__main__":
    main()
