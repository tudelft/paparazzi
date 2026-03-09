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

# HSV green range for pixel filtering.
green_lower = np.array([25, 35, 60], dtype=np.uint8)
green_upper = np.array([95, 255, 255], dtype=np.uint8)

# Morphological growth for green regions.
expand_kernel_size = 3
expand_iterations = 1
dilate_kernel_size = 5
dilate_iterations = 1

# Sparse feature detector setup with runtime toggling.
max_features = 150
min_features_to_redetect = 60
feature_methods = ["FAST", "GFTT", "ORB"]
feature_method_idx = 0
show_masked_view = True

fast_detector = cv2.FastFeatureDetector_create()
gftt_params = dict(
    maxCorners=max_features,
    qualityLevel=0.01,
    minDistance=7,
    blockSize=7,
)
orb_detector = cv2.ORB_create(nfeatures=max_features)


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


def draw_grid(display_image):
    height, width = display_image.shape[:2]
    row_h = height // 3
    col_w = width // 3

    # Draw full 3x3 grid.
    cv2.line(display_image, (col_w, 0), (col_w, height), (255, 255, 255), 1)
    cv2.line(display_image, (2 * col_w, 0), (2 * col_w, height), (255, 255, 255), 1)
    cv2.line(display_image, (0, row_h), (width, row_h), (255, 255, 255), 1)
    cv2.line(display_image, (0, 2 * row_h), (width, 2 * row_h), (255, 255, 255), 1)


def keep_only_green_pixels(image_bgr):
    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    green_mask = cv2.inRange(hsv, green_lower, green_upper)

    expand_kernel = np.ones((expand_kernel_size, expand_kernel_size), dtype=np.uint8)
    dilate_kernel = np.ones((dilate_kernel_size, dilate_kernel_size), dtype=np.uint8)

    # Expand connected green regions, then dilate further to grow coverage.
    expanded_mask = cv2.morphologyEx(
        green_mask,
        cv2.MORPH_CLOSE,
        expand_kernel,
        iterations=expand_iterations,
    )
    grown_mask = cv2.dilate(expanded_mask, dilate_kernel, iterations=dilate_iterations)

    green_only = cv2.bitwise_and(image_bgr, image_bgr, mask=grown_mask)
    return green_only, grown_mask


def overlay_red_mask(image_bgr, green_mask, alpha=0.5):
    overlay = image_bgr.copy()
    overlay[green_mask > 0] = (0, 0, 255)
    return cv2.addWeighted(overlay, alpha, image_bgr, 1.0 - alpha, 0.0)

# Play images
for image_file in image_files:
    image_path = os.path.join(folder_path, image_file)
    image = cv2.imread(image_path)
    
    if image is None:
        continue
    
    # Resize to match first image dimensions
    image = cv2.resize(image, (width, height))
    
    image = cv2.rotate(image, rotation_code)
    original_image = image.copy()
    masked_image, green_mask = keep_only_green_pixels(image)
    red_overlay_image = overlay_red_mask(original_image, green_mask)

    gray_image = cv2.cvtColor(masked_image, cv2.COLOR_BGR2GRAY)
    display_image = red_overlay_image.copy() if show_masked_view else original_image.copy()

    current_method = feature_methods[feature_method_idx]
    points = detect_points(gray_image, current_method, max_features)

    if points is not None:
        for pt in points.reshape(-1, 2):
            x, y = pt.astype(int)
            cv2.circle(display_image, (x, y), 3, (0, 255, 0), -1)

    tracked_count = 0 if points is None else len(points)
    draw_grid(display_image)

    cv2.putText(
        display_image,
        f"Detected features: {tracked_count}",
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        display_image,
        "Optical flow: OFF",
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
    cv2.putText(
        display_image,
        f"View: {'RED MASK OVERLAY' if show_masked_view else 'ORIGINAL'} (press 'm')",
        (10, 120),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )

    cv2.imshow("Image Sequence", display_image)
    
    # Press 'q' to quit, 't' to switch detector, 'm' to toggle masked/original view.
    key = cv2.waitKey(100) & 0xFF
    if key == ord('t'):
        feature_method_idx = (feature_method_idx + 1) % len(feature_methods)
    if key == ord('m'):
        show_masked_view = not show_masked_view
    if key == ord('q'):
        break

cv2.destroyAllWindows()