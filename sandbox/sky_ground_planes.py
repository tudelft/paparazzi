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

show_edge_visualization = True


def estimate_floor_wall_from_edges(frame_bgr):
    height, width = frame_bgr.shape[:2]
    gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blurred, 60, 150)

    lines = cv2.HoughLinesP(
        edges,
        rho=1,
        theta=np.pi / 180,
        threshold=70,
        minLineLength=max(30, width // 8),
        maxLineGap=20,
    )

    floor_candidates = []
    vertical_candidates = []
    if lines is not None:
        for line in lines[:, 0]:
            x1, y1, x2, y2 = line
            dx = x2 - x1
            dy = y2 - y1
            length = float(np.hypot(dx, dy))
            if length < max(20, width * 0.03):
                continue

            angle_deg = abs(np.degrees(np.arctan2(dy, dx)))
            y_mid = 0.5 * (y1 + y2)

            # Horizontal lines in the lower half are likely floor boundaries.
            if angle_deg < 20 and y_mid > height * 0.45:
                floor_candidates.append((y_mid, length))

            # Near-vertical lines are likely wall boundaries.
            if angle_deg > 70:
                x_mid = 0.5 * (x1 + x2)
                vertical_candidates.append((x_mid, length))

    if floor_candidates:
        y_floor = int(
            np.average(
                [candidate[0] for candidate in floor_candidates],
                weights=[candidate[1] for candidate in floor_candidates],
            )
        )
    else:
        y_floor = int(height * 0.62)

    y_floor = int(np.clip(y_floor, int(height * 0.45), int(height * 0.9)))

    # Build simple masks: floor below boundary, walls above it.
    floor_mask = np.zeros((height, width), dtype=np.uint8)
    wall_mask = np.zeros((height, width), dtype=np.uint8)
    floor_mask[y_floor:, :] = 255
    wall_mask[:y_floor, :] = 255

    # If strong vertical boundaries exist, keep walls mostly near image sides.
    if len(vertical_candidates) >= 2:
        x_values = [candidate[0] for candidate in vertical_candidates]
        left_bound = int(np.percentile(x_values, 25))
        right_bound = int(np.percentile(x_values, 75))
        center_mask = np.zeros_like(wall_mask)
        center_mask[:, max(0, left_bound):min(width, right_bound)] = 255
        wall_mask = cv2.bitwise_and(wall_mask, cv2.bitwise_not(center_mask))

    overlay = frame_bgr.copy()
    overlay[floor_mask > 0] = (0, 180, 0)
    overlay[wall_mask > 0] = (255, 120, 0)
    blended = cv2.addWeighted(frame_bgr, 0.55, overlay, 0.45, 0)

    # Keep edge map as a debug view.
    edges_bgr = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
    cv2.line(blended, (0, y_floor), (width - 1, y_floor), (0, 255, 255), 2)

    floor_ratio = float(np.count_nonzero(floor_mask)) / float(height * width)
    wall_ratio = float(np.count_nonzero(wall_mask)) / float(height * width)
    return blended, edges_bgr, y_floor, floor_ratio, wall_ratio

# Play images
for image_file in image_files:
    image_path = os.path.join(folder_path, image_file)
    image = cv2.imread(image_path)
    
    if image is None:
        continue
    
    # Resize to match first image dimensions
    image = cv2.resize(image, (width, height))
    
    image = cv2.rotate(image, rotation_code)

    segmentation_view, edge_view, y_floor, floor_ratio, wall_ratio = estimate_floor_wall_from_edges(image)
    display_image = edge_view if show_edge_visualization else segmentation_view

    cv2.putText(
        display_image,
        f"Floor boundary y: {y_floor}",
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        display_image,
        f"Floor area: {floor_ratio*100:.1f}% | Wall area: {wall_ratio*100:.1f}%",
        (10, 60),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        display_image,
        f"Edge view: {'ON' if show_edge_visualization else 'OFF'} (press 'e')",
        (10, 90),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )

    cv2.imshow("Image Sequence", display_image)
    
    # Press 'q' to quit, wait 500ms between frames
    key = cv2.waitKey(100) & 0xFF
    if key == ord('e'):
        show_edge_visualization = not show_edge_visualization
    if key == ord('q'):
        break

cv2.destroyAllWindows()