import cv2
import numpy as np
import os

# Specify your folder path
folder_path = "./datasets/Organised/20260306-handFlying-3"  # Change this to your image folder

# Get all image files sorted
image_files = sorted([f for f in os.listdir(folder_path) 
                     if f.lower().endswith(('.png', '.jpg', '.jpeg', '.bmp', '.tiff'))])

if not image_files:
    print("No images found in the folder")
    exit()

# Read first image to get dimensions
first_image = cv2.imread(os.path.join(folder_path, image_files[0]))
height, width = first_image.shape[:2]

# Rotate each frame by 90 degrees counterclockwise before display.
rotation_code = cv2.ROTATE_90_COUNTERCLOCKWISE

# HSV orange range (adjust if needed for your camera/lighting).
orange_lower = np.array([0, 80, 80], dtype=np.uint8) # 5
orange_upper = np.array([25, 255, 255], dtype=np.uint8) # 25

# Obstacle decision thresholds per column.
orange_fraction_threshold = 0.25 #0.06
orange_pixel_threshold = 150 # 250


def get_column_sector_ranges(frame_width, frame_height):
    col_w = frame_width // 3
    return {
        "L": (0, 0, col_w, frame_height),
        "C": (col_w, 0, 2 * col_w, frame_height),
        "R": (2 * col_w, 0, frame_width, frame_height),
    }


def analyze_column_obstacles(orange_mask):
    frame_height, frame_width = orange_mask.shape
    sectors = get_column_sector_ranges(frame_width, frame_height)
    stats = {}

    for name, (x1, y1, x2, y2) in sectors.items():
        roi_mask = orange_mask[y1:y2, x1:x2]
        orange_pixels = int(np.count_nonzero(roi_mask))
        total_pixels = max(1, roi_mask.size)
        orange_fraction = orange_pixels / float(total_pixels)
        obstacle = (
            orange_pixels >= orange_pixel_threshold
            and orange_fraction >= orange_fraction_threshold
        )

        stats[name] = {
            "rect": (x1, y1, x2, y2),
            "orange_pixels": orange_pixels,
            "orange_fraction": orange_fraction,
            "obstacle": obstacle,
        }

    return stats


def draw_grid_and_obstacles(display_image, sector_stats):
    height, width = display_image.shape[:2]
    col_w = width // 3

    # Draw 1x3 column grid.
    cv2.line(display_image, (col_w, 0), (col_w, height), (255, 255, 255), 1)
    cv2.line(display_image, (2 * col_w, 0), (2 * col_w, height), (255, 255, 255), 1)

    for name in ("L", "C", "R"):
        x1, y1, x2, y2 = sector_stats[name]["rect"]
        is_obstacle = sector_stats[name]["obstacle"]
        color = (0, 0, 255) if is_obstacle else (0, 255, 0)
        label = f"{name}: {'OBS' if is_obstacle else 'CLEAR'}"
        cv2.rectangle(display_image, (x1, y1), (x2, y2), color, 2)
        cv2.putText(
            display_image,
            label,
            (x1 + 10, y1 + 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            color,
            2,
            cv2.LINE_AA,
        )


def build_orange_mask(image_bgr):
    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, orange_lower, orange_upper)
    kernel = np.ones((3, 3), dtype=np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask

# Play images with keyboard indexing controls.
frame_idx = 0
is_playing = True
last_index = len(image_files) - 1

while True:
    image_file = image_files[frame_idx]
    image_path = os.path.join(folder_path, image_file)
    image = cv2.imread(image_path)

    if image is None:
        if is_playing and frame_idx < last_index:
            frame_idx += 1
            continue
        if is_playing and frame_idx >= last_index:
            is_playing = False
        key = cv2.waitKey(0) & 0xFF
        if key == ord('q'):
            break
        if key in (ord(' '), ord('s')):
            is_playing = not is_playing
        if key in (ord('n'), 83):
            frame_idx = min(last_index, frame_idx + 1)
            is_playing = False
        if key in (ord('b'), 81):
            frame_idx = max(0, frame_idx - 1)
            is_playing = False
        continue

    # Normalize frame size at full resolution.
    image = cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA)

    image = cv2.rotate(image, rotation_code)

    orange_mask = build_orange_mask(image)
    sector_stats = analyze_column_obstacles(orange_mask)

    # Overlay orange mask on image for visual confirmation.
    orange_overlay = np.zeros_like(image)
    orange_overlay[orange_mask > 0] = (0, 110, 255)
    display_image = cv2.addWeighted(image, 0.75, orange_overlay, 0.50, 0.0)

    draw_grid_and_obstacles(display_image, sector_stats)

    cv2.putText(
        display_image,
        (
            f"Orange thresholds: px>={orange_pixel_threshold}, "
            f"frac>={orange_fraction_threshold:.2f}"
        ),
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        display_image,
        (
            "Column OBS L/C/R: "
            f"{int(sector_stats['L']['obstacle'])}/"
            f"{int(sector_stats['C']['obstacle'])}/"
            f"{int(sector_stats['R']['obstacle'])}"
        ),
        (10, 60),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        display_image,
        (
            f"Orange px L/C/R: "
            f"{sector_stats['L']['orange_pixels']}/"
            f"{sector_stats['C']['orange_pixels']}/"
            f"{sector_stats['R']['orange_pixels']}"
        ),
        (10, 90),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        display_image,
        (
            f"Frame {frame_idx + 1}/{len(image_files)} | "
            f"Mode: {'PLAY' if is_playing else 'PAUSE'} | "
            "space/s: play-pause, n/right: next, b/left: back"
        ),
        (10, 120),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )

    cv2.imshow("Image Sequence", display_image)

    key = cv2.waitKey(100 if is_playing else 0) & 0xFF
    if key == ord('q'):
        break
    if key in (ord(' '), ord('s')):
        is_playing = not is_playing
        continue
    if key in (ord('n'), 83):
        frame_idx = min(last_index, frame_idx + 1)
        is_playing = False
        continue
    if key in (ord('b'), 81):
        frame_idx = max(0, frame_idx - 1)
        is_playing = False
        continue

    if is_playing:
        if frame_idx < last_index:
            frame_idx += 1
        else:
            is_playing = False

cv2.destroyAllWindows()