import cv2
import numpy as np
import os

# Specify your folder path
folder_path = "./datasets/Organised/20260306-handFlying-2"  # Change this to your image folder

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

# Sparse feature detector setup with runtime toggling.
max_features = 150
min_features_to_redetect = 60
feature_methods = ["FAST", "GFTT", "ORB"]
feature_method_idx = 0

fast_detector = cv2.FastFeatureDetector_create()
gftt_params = dict(
    maxCorners=max_features,
    qualityLevel=0.01,
    minDistance=7,
    blockSize=7,
)
orb_detector = cv2.ORB_create(nfeatures=max_features)

lk_params = dict(
    winSize=(21, 21),
    maxLevel=3,
    criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01),
)

# Middle-row obstacle thresholds (tuned for 100 ms frame delay).
motion_threshold_px = 1.5
hot_fraction_threshold = 0.25
min_tracks_per_sector = 6

prev_gray = None
prev_points = None
force_redetect = False


def keypoints_to_points(keypoints, count):
    keypoints = sorted(keypoints, key=lambda kp: kp.response, reverse=True)[:count]
    if not keypoints:
        return None
    points = np.array([kp.pt for kp in keypoints], dtype=np.float32).reshape(-1, 1, 2)
    return points


def detect_points(gray_image, method_name, count):
    if method_name == "FAST":
        keypoints = fast_detector.detect(gray_image, None)
        return keypoints_to_points(keypoints, count)

    if method_name == "GFTT":
        points = cv2.goodFeaturesToTrack(gray_image, mask=None, **gftt_params)
        if points is None:
            return None
        return points.astype(np.float32)

    if method_name == "ORB":
        keypoints = orb_detector.detect(gray_image, None)
        return keypoints_to_points(keypoints, count)

    return None


def get_middle_row_sector_ranges(frame_width, frame_height):
    row_h = frame_height // 3
    col_w = frame_width // 3
    y1, y2 = row_h, 2 * row_h
    return {
        "L": (0, y1, col_w, y2),
        "C": (col_w, y1, 2 * col_w, y2),
        "R": (2 * col_w, y1, frame_width, y2),
    }


def analyze_middle_row_obstacles(good_old, good_new, frame_width, frame_height):
    sectors = get_middle_row_sector_ranges(frame_width, frame_height)
    stats = {}

    for name, (x1, y1, x2, y2) in sectors.items():
        stats[name] = {
            "rect": (x1, y1, x2, y2),
            "track_count": 0,
            "hot_fraction": 0.0,
            "mean_motion": 0.0,
            "obstacle": False,
        }

    if good_new.size == 0:
        return stats

    motion = np.linalg.norm(good_new - good_old, axis=1)

    for name, sector in stats.items():
        x1, y1, x2, y2 = sector["rect"]
        in_sector = (
            (good_new[:, 0] >= x1)
            & (good_new[:, 0] < x2)
            & (good_new[:, 1] >= y1)
            & (good_new[:, 1] < y2)
        )

        sector_motion = motion[in_sector]
        count = int(sector_motion.size)
        sector["track_count"] = count

        if count == 0:
            continue

        hot_fraction = float(np.mean(sector_motion > motion_threshold_px))
        mean_motion = float(np.mean(sector_motion))
        sector["hot_fraction"] = hot_fraction
        sector["mean_motion"] = mean_motion
        sector["obstacle"] = (
            count >= min_tracks_per_sector and hot_fraction >= hot_fraction_threshold
        )

    return stats


def draw_grid_and_obstacles(display_image, sector_stats):
    height, width = display_image.shape[:2]
    row_h = height // 3
    col_w = width // 3

    # Draw full 3x3 grid.
    cv2.line(display_image, (col_w, 0), (col_w, height), (255, 255, 255), 1)
    cv2.line(display_image, (2 * col_w, 0), (2 * col_w, height), (255, 255, 255), 1)
    cv2.line(display_image, (0, row_h), (width, row_h), (255, 255, 255), 1)
    cv2.line(display_image, (0, 2 * row_h), (width, 2 * row_h), (255, 255, 255), 1)

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

# Play images
for image_file in image_files:
    image_path = os.path.join(folder_path, image_file)
    image = cv2.imread(image_path)
    
    if image is None:
        continue
    
    # Resize to match first image dimensions
    image = cv2.resize(image, (width, height))
    
    image = cv2.rotate(image, rotation_code)

    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    display_image = image.copy()

    sectors = get_middle_row_sector_ranges(image.shape[1], image.shape[0])
    sector_stats = {
        name: {
            "rect": rect,
            "track_count": 0,
            "hot_fraction": 0.0,
            "mean_motion": 0.0,
            "obstacle": False,
        }
        for name, rect in sectors.items()
    }
    tracked_count = 0

    current_method = feature_methods[feature_method_idx]

    if force_redetect or prev_gray is None or prev_points is None or len(prev_points) < min_features_to_redetect:
        prev_points = detect_points(gray_image, current_method, max_features)
        force_redetect = False
    else:
        next_points, status, _ = cv2.calcOpticalFlowPyrLK(
            prev_gray,
            gray_image,
            prev_points,
            None,
            **lk_params,
        )

        if next_points is not None and status is not None:
            good_new = next_points[status.flatten() == 1].reshape(-1, 2)
            good_old = prev_points[status.flatten() == 1].reshape(-1, 2)

            if good_new.size > 0:
                sector_stats = analyze_middle_row_obstacles(
                    good_old,
                    good_new,
                    image.shape[1],
                    image.shape[0],
                )

                for new_pt, old_pt in zip(good_new, good_old):
                    new_x, new_y = new_pt.astype(int)
                    old_x, old_y = old_pt.astype(int)
                    cv2.line(display_image, (old_x, old_y), (new_x, new_y), (0, 255, 255), 1)
                    cv2.circle(display_image, (new_x, new_y), 2, (0, 255, 0), -1)

            prev_points = good_new.reshape(-1, 1, 2).astype(np.float32)
        else:
            prev_points = None

    tracked_count = 0 if prev_points is None else len(prev_points)
    draw_grid_and_obstacles(display_image, sector_stats)

    cv2.putText(
        display_image,
        f"Tracked features: {tracked_count}",
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        display_image,
        (
            "Middle row OBS L/C/R: "
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
        f"Feature method: {current_method} (press 't' to toggle)",
        (10, 90),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )

    cv2.imshow("Image Sequence", display_image)
    prev_gray = gray_image.copy()
    
    # Press 'q' to quit, 't' to switch feature detector.
    key = cv2.waitKey(100) & 0xFF
    if key == ord('t'):
        feature_method_idx = (feature_method_idx + 1) % len(feature_methods)
        prev_points = None
        force_redetect = True
    if key == ord('q'):
        break

cv2.destroyAllWindows()