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
green_lower = np.array([25, 30, 60], dtype=np.uint8)
green_upper = np.array([100, 255, 255], dtype=np.uint8)

show_masked_view = True

min_region_area = 120


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
    return cv2.bitwise_and(image_bgr, image_bgr, mask=green_mask)


def detect_green_regions(masked_image_bgr):
    gray = cv2.cvtColor(masked_image_bgr, cv2.COLOR_BGR2GRAY)
    green_mask = np.zeros_like(gray, dtype=np.uint8)
    green_mask[gray > 0] = 255

    # Morphology to clean small noise and fill small holes in green areas.
    kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    kernel_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    region_mask = cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, kernel_close)
    region_mask = cv2.morphologyEx(region_mask, cv2.MORPH_OPEN, kernel_open)

    contours, _ = cv2.findContours(region_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    filtered_contours = []
    filtered_mask = np.zeros_like(region_mask)
    image_height = region_mask.shape[0]

    for cnt in contours:
        if cv2.contourArea(cnt) < min_region_area:
            continue

        x, y, w, h = cv2.boundingRect(cnt)
        touches_bottom = (y + h) >= (image_height - 1)
        if touches_bottom:
            continue

        filtered_contours.append(cnt)
        cv2.drawContours(filtered_mask, [cnt], -1, 255, thickness=cv2.FILLED)

    return filtered_mask, filtered_contours

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
    masked_image = keep_only_green_pixels(image)
    display_image = masked_image.copy() if show_masked_view else original_image.copy()

    region_mask, contours = detect_green_regions(masked_image)
    region_pixels = int(np.count_nonzero(region_mask))

    # Show the mask immediately after morphology for inspection.
    morph_display = cv2.cvtColor(region_mask, cv2.COLOR_GRAY2BGR)
    cv2.putText(
        morph_display,
        "Post-morph green mask",
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )

    texture_overlay = np.zeros_like(display_image)
    texture_overlay[region_mask > 0] = (0, 0, 255)
    display_image = cv2.addWeighted(display_image, 0.75, texture_overlay, 0.45, 0.0)

    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        cv2.rectangle(display_image, (x, y), (x + w, y + h), (0, 255, 255), 2)

    region_count = len(contours)
    draw_grid(display_image)

    cv2.putText(
        display_image,
        f"Green regions: {region_count}",
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        display_image,
        f"Green pixels in regions: {region_pixels}",
        (10, 60),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        display_image,
        "Detection: green-mask regions (bottom-touch excluded)",
        (10, 90),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        display_image,
        f"View: {'MASKED' if show_masked_view else 'ORIGINAL'} (press 'm')",
        (10, 120),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )

    cv2.imshow("Image Sequence", display_image)
    cv2.imshow("Morph Mask", morph_display)
    
    # Press 'q' to quit, 'm' to toggle masked/original view.
    key = cv2.waitKey(100) & 0xFF
    if key == ord('m'):
        show_masked_view = not show_masked_view
    if key == ord('q'):
        break

cv2.destroyAllWindows()