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

# Dense Farneback optical flow parameters.
farneback_params = dict(
    pyr_scale=0.5,
    levels=3,
    winsize=21,
    iterations=3,
    poly_n=5,
    poly_sigma=1.2,
    flags=0,
)

prev_gray = None
show_flow_overlay = True


def compute_flow_divergence(flow):
    u = flow[:, :, 0]
    v = flow[:, :, 1]

    du_dx = cv2.Sobel(u, cv2.CV_32F, 1, 0, ksize=3)
    dv_dy = cv2.Sobel(v, cv2.CV_32F, 0, 1, ksize=3)
    divergence = du_dx + dv_dy
    return divergence


def visualize_divergence(divergence):
    # Robust scaling for display stability.
    low = float(np.percentile(divergence, 2.0))
    high = float(np.percentile(divergence, 98.0))
    span = max(high - low, 1e-6)
    scaled = np.clip((divergence - low) / span, 0.0, 1.0)
    div_u8 = (scaled * 255.0).astype(np.uint8)
    return cv2.applyColorMap(div_u8, cv2.COLORMAP_TURBO)


def analyze_middle_row_obstacles(divergence):
    height, width = divergence.shape
    row_h = height // 3
    col_w = width // 3

    y1 = row_h
    y2 = 2 * row_h
    middle_row = divergence[y1:y2, :]

    # Use a high percentile in the middle row as expansion threshold.
    expansion_threshold = float(np.percentile(middle_row, 85.0))

    sectors = {}
    for name, idx in (("L", 0), ("C", 1), ("R", 2)):
        x1 = idx * col_w
        x2 = width if idx == 2 else (idx + 1) * col_w
        roi = divergence[y1:y2, x1:x2]

        hot_fraction = float(np.mean(roi > expansion_threshold))
        positive_div = float(np.mean(np.maximum(roi, 0.0)))
        obstacle = hot_fraction > 0.18 and positive_div > 0.0

        sectors[name] = {
            "obstacle": obstacle,
            "hot_fraction": hot_fraction,
            "positive_div": positive_div,
            "rect": (x1, y1, x2, y2),
        }

    return sectors


def draw_grid_and_obstacles(display_image, sectors):
    height, width = display_image.shape[:2]
    row_h = height // 3
    col_w = width // 3

    # Draw 3x3 grid.
    cv2.line(display_image, (col_w, 0), (col_w, height), (255, 255, 255), 1)
    cv2.line(display_image, (2 * col_w, 0), (2 * col_w, height), (255, 255, 255), 1)
    cv2.line(display_image, (0, row_h), (width, row_h), (255, 255, 255), 1)
    cv2.line(display_image, (0, 2 * row_h), (width, 2 * row_h), (255, 255, 255), 1)

    for name in ("L", "C", "R"):
        sector = sectors[name]
        x1, y1, x2, y2 = sector["rect"]
        color = (0, 0, 255) if sector["obstacle"] else (0, 255, 0)
        label = f"{name}: {'OBS' if sector['obstacle'] else 'CLEAR'}"

        cv2.rectangle(display_image, (x1, y1), (x2, y2), color, 2)
        cv2.putText(
            display_image,
            label,
            (x1 + 10, y1 + 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
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
    mean_divergence = 0.0
    flow_magnitude = 0.0
    sector_states = {
        "L": {"obstacle": False, "hot_fraction": 0.0, "positive_div": 0.0, "rect": (0, 0, 0, 0)},
        "C": {"obstacle": False, "hot_fraction": 0.0, "positive_div": 0.0, "rect": (0, 0, 0, 0)},
        "R": {"obstacle": False, "hot_fraction": 0.0, "positive_div": 0.0, "rect": (0, 0, 0, 0)},
    }

    if prev_gray is not None:
        flow = cv2.calcOpticalFlowFarneback(prev_gray, gray_image, None, **farneback_params)
        divergence = compute_flow_divergence(flow)

        if show_flow_overlay:
            divergence_map = visualize_divergence(divergence)
            display_image = cv2.addWeighted(image, 0.45, divergence_map, 0.55, 0.0)

        mean_divergence = float(np.mean(divergence))
        flow_magnitude = float(np.mean(np.linalg.norm(flow, axis=2)))
        sector_states = analyze_middle_row_obstacles(divergence)

    draw_grid_and_obstacles(display_image, sector_states)

    cv2.putText(
        display_image,
        f"Mean divergence: {mean_divergence:+.4f}",
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        display_image,
        f"Mean flow magnitude: {flow_magnitude:.4f}",
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
            f"Middle row OBS L/C/R: "
            f"{int(sector_states['L']['obstacle'])}/"
            f"{int(sector_states['C']['obstacle'])}/"
            f"{int(sector_states['R']['obstacle'])}"
        ),
        (10, 90),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        display_image,
        f"Flow overlay: {'ON' if show_flow_overlay else 'OFF'} (press 'v')",
        (10, 120),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )

    cv2.imshow("Image Sequence", display_image)
    prev_gray = gray_image.copy()
    
    # Press 'q' to quit and 'v' to toggle divergence overlay.
    key = cv2.waitKey(100) & 0xFF
    if key == ord('v'):
        show_flow_overlay = not show_flow_overlay
    if key == ord('q'):
        break

cv2.destroyAllWindows()