import cv2
import numpy as np
import os

# Specify your folder path
folder_path = "./datasets/AE4317_2019_datasets/cyberzoo_poles/20190121-135009"  # Change this to your image folder

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

# Path to monocular depth model in ONNX format (for example MiDaS small).
model_path = "./models/midas_small.onnx"
if not os.path.exists(model_path):
    print(f"Depth model not found at: {model_path}")
    print("Place an ONNX depth model there (e.g. MiDaS small) and rerun.")
    exit()

depth_net = cv2.dnn.readNet(model_path)
model_input_size = (256, 256)
show_depth_visualization = True


def estimate_depth_colormap(frame_bgr, net, input_size):
    frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
    resized = cv2.resize(frame_rgb, input_size, interpolation=cv2.INTER_CUBIC)

    # Standard ImageNet normalization used by common monocular depth models.
    mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)
    std = np.array([0.229, 0.224, 0.225], dtype=np.float32)
    normalized = (resized - mean) / std

    blob = np.transpose(normalized, (2, 0, 1))[np.newaxis, ...].astype(np.float32)
    net.setInput(blob)
    depth = net.forward().squeeze()

    depth_resized = cv2.resize(
        depth,
        (frame_bgr.shape[1], frame_bgr.shape[0]),
        interpolation=cv2.INTER_CUBIC,
    )
    depth_u8 = cv2.normalize(depth_resized, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    depth_colormap = cv2.applyColorMap(depth_u8, cv2.COLORMAP_INFERNO)
    return depth_colormap, float(np.mean(depth_resized))

# Play images
for image_file in image_files:
    image_path = os.path.join(folder_path, image_file)
    image = cv2.imread(image_path)
    
    if image is None:
        continue
    
    # Resize to match first image dimensions
    image = cv2.resize(image, (width, height))
    
    image = cv2.rotate(image, rotation_code)

    depth_colormap, mean_depth = estimate_depth_colormap(image, depth_net, model_input_size)
    display_image = depth_colormap if show_depth_visualization else image.copy()

    cv2.putText(
        display_image,
        f"Mean relative depth: {mean_depth:.2f}",
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        display_image,
        f"Depth view: {'ON' if show_depth_visualization else 'OFF'} (press 'd')",
        (10, 60),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )

    cv2.imshow("Image Sequence", display_image)
    
    # Press 'q' to quit, wait 500ms between frames
    key = cv2.waitKey(30) & 0xFF
    if key == ord('d'):
        show_depth_visualization = not show_depth_visualization
    if key == ord('q'):
        break

cv2.destroyAllWindows()