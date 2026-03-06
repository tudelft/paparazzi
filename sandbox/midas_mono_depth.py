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
downsample_scale = 1.0
target_width = max(1, int(width * downsample_scale))
target_height = max(1, int(height * downsample_scale))
view_scale = 1.0 / downsample_scale

# Rotate each frame by 90 degrees counterclockwise before display.
rotation_code = cv2.ROTATE_90_COUNTERCLOCKWISE

# Depth model can be changed with DEPTH_MODEL_NAME or DEPTH_MODEL_PATH.
models_dir = "./models"
depth_model_name = os.getenv("DEPTH_MODEL_NAME", "midas_small.onnx")
depth_model_path = os.getenv("DEPTH_MODEL_PATH", os.path.join(models_dir, depth_model_name))

if not os.path.isabs(depth_model_path):
    depth_model_path = os.path.join(".", depth_model_path)

if not os.path.exists(depth_model_path):
    available_models = sorted(
        os.path.join(models_dir, file_name)
        for file_name in os.listdir(models_dir)
        if file_name.lower().endswith(".onnx")
    ) if os.path.isdir(models_dir) else []

    if available_models:
        print(
            f"Requested depth model not found at: {depth_model_path}\\n"
            f"Falling back to: {available_models[0]}"
        )
        depth_model_path = available_models[0]
    else:
        print(f"Depth model not found at: {depth_model_path}")
        print("No ONNX depth models found in ./models")
        exit()

depth_net = cv2.dnn.readNet(depth_model_path)
model_input_dim = int(os.getenv("DEPTH_INPUT_SIZE", "256"))
model_input_size = (model_input_dim, model_input_dim)
show_depth_visualization = True
enable_distance_threshold = True
close_pixel_fraction = 1.0 / 3.0
block_on_threshold = 0.22
block_off_threshold = 0.16
blocked_state = {"L": False, "F": False, "R": False}


def preprocess_with_letterbox(frame_rgb, input_size):
    target_w, target_h = input_size
    src_h, src_w = frame_rgb.shape[:2]

    scale = min(target_w / src_w, target_h / src_h)
    resized_w = max(1, int(round(src_w * scale)))
    resized_h = max(1, int(round(src_h * scale)))

    resized = cv2.resize(frame_rgb, (resized_w, resized_h), interpolation=cv2.INTER_CUBIC)
    padded = np.zeros((target_h, target_w, 3), dtype=np.float32)

    pad_x = (target_w - resized_w) // 2
    pad_y = (target_h - resized_h) // 2
    padded[pad_y:pad_y + resized_h, pad_x:pad_x + resized_w] = resized

    return padded, pad_x, pad_y, resized_w, resized_h


def infer_depth_map(frame_rgb, net, input_size):
    resized, pad_x, pad_y, resized_w, resized_h = preprocess_with_letterbox(frame_rgb, input_size)

    # Standard ImageNet normalization used by common monocular depth models.
    mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)
    std = np.array([0.229, 0.224, 0.225], dtype=np.float32)
    normalized = (resized - mean) / std

    blob = np.transpose(normalized, (2, 0, 1))[np.newaxis, ...].astype(np.float32)
    net.setInput(blob)
    depth = net.forward().squeeze()

    # Remove letterbox padding before resizing back to frame resolution.
    return depth[pad_y:pad_y + resized_h, pad_x:pad_x + resized_w]


def estimate_depth_colormap(frame_bgr, net, input_size):
    frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
    depth = infer_depth_map(frame_rgb, net, input_size)

    depth_resized = cv2.resize(
        depth,
        (frame_bgr.shape[1], frame_bgr.shape[0]),
        interpolation=cv2.INTER_CUBIC,
    )

    # Per-frame robust normalization (no temporal smoothing).
    near_val = float(np.percentile(depth_resized, 2.0))
    far_val = float(np.percentile(depth_resized, 98.0))
    depth_span = max(far_val - near_val, 1e-6)
    normalized_depth = np.clip((depth_resized - near_val) / depth_span, 0.0, 1.0)

    # MiDaS output is inverse-depth-like, so convert to a distance proxy where
    # lower values are closer and keep the closest third of pixels.
    normalized_distance = 1.0 - normalized_depth
    close_distance_threshold = float(
        np.quantile(normalized_distance, close_pixel_fraction)
    )
    close_mask = normalized_distance <= close_distance_threshold

    depth_u8 = (normalized_depth * 255.0).astype(np.uint8)
    depth_colormap = cv2.applyColorMap(depth_u8, cv2.COLORMAP_INFERNO)
    depth_colormap_filtered = np.zeros_like(depth_colormap)
    depth_colormap_filtered[close_mask] = depth_colormap[close_mask]
    return depth_colormap, depth_colormap_filtered, close_mask, normalized_distance


def get_sector_obstacle_scores(close_mask):
    height, width = close_mask.shape
    third = width // 3
    left = close_mask[:, :third]
    front = close_mask[:, third:2 * third]
    right = close_mask[:, 2 * third:]

    # Score = fraction of pixels in that sector classified as close.
    return {
        "L": float(np.mean(left)) if left.size else 0.0,
        "F": float(np.mean(front)) if front.size else 0.0,
        "R": float(np.mean(right)) if right.size else 0.0,
    }


def update_blocked_states(sector_scores):
    for sector in ("L", "F", "R"):
        score = sector_scores[sector]
        if blocked_state[sector]:
            blocked_state[sector] = score > block_off_threshold
        else:
            blocked_state[sector] = score >= block_on_threshold


def decide_frm_action(sector_scores):
    # F: move forward, R: turn right, M: turn left/manoeuvre.
    if not blocked_state["F"]:
        return "F"

    if not blocked_state["R"] and (blocked_state["L"] or sector_scores["R"] <= sector_scores["L"]):
        return "R"

    return "M"

# Play images
for image_file in image_files:
    image_path = os.path.join(folder_path, image_file)
    image = cv2.imread(image_path)
    
    if image is None:
        continue
    
    # Downsample each frame before any further processing.
    image = cv2.resize(image, (target_width, target_height), interpolation=cv2.INTER_AREA)
    
    image = cv2.rotate(image, rotation_code)

    depth_colormap, depth_colormap_filtered, close_mask, normalized_distance = estimate_depth_colormap(
        image, depth_net, model_input_size
    )
    sector_scores = get_sector_obstacle_scores(close_mask)
    update_blocked_states(sector_scores)
    frm_action = decide_frm_action(sector_scores)
    rgb_close_only = np.zeros_like(image)
    rgb_close_only[close_mask] = image[close_mask]

    if show_depth_visualization:
        display_image = depth_colormap_filtered if enable_distance_threshold else depth_colormap
    else:
        display_image = rgb_close_only if enable_distance_threshold else image.copy()

    display_for_view = cv2.resize(
        display_image,
        None,
        fx=view_scale,
        fy=view_scale,
        interpolation=cv2.INTER_LINEAR,
    )

    cv2.putText(
        display_for_view,
        (
            f"Occ L/F/R: {sector_scores['L']:.2f} / {sector_scores['F']:.2f} / {sector_scores['R']:.2f}"
        ),
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        display_for_view,
        (
            f"State L/F/R: "
            f"{'BLK' if blocked_state['L'] else 'CLR'} / "
            f"{'BLK' if blocked_state['F'] else 'CLR'} / "
            f"{'BLK' if blocked_state['R'] else 'CLR'}"
        ),
        (10, 60),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        display_for_view,
        f"Action (F/R/M): {frm_action}",
        (10, 90),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        display_for_view,
        f"Depth view: {'ON' if show_depth_visualization else 'OFF'} (press 'd')",
        (10, 120),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        display_for_view,
        f"Close filter: {'ON' if enable_distance_threshold else 'OFF'} (press 'f')",
        (10, 150),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )

    cv2.imshow("Image Sequence", display_for_view)
    
    # Press 'q' to quit, wait 500ms between frames
    key = cv2.waitKey(30) & 0xFF
    if key == ord('d'):
        show_depth_visualization = not show_depth_visualization
    if key == ord('f'):
        enable_distance_threshold = not enable_distance_threshold
    if key == ord('q'):
        break

cv2.destroyAllWindows()