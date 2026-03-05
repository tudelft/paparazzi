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

# Dense Farneback optical flow parameters.
farneback_params = dict(
    pyr_scale=0.5,
    levels=3,
    winsize=15,
    iterations=3,
    poly_n=5,
    poly_sigma=1.2,
    flags=0,
)

prev_gray = None
show_flow_visualization = True


def flow_to_hsv_bgr(flow):
    mag, ang = cv2.cartToPolar(flow[..., 0], flow[..., 1])
    hsv = np.zeros((flow.shape[0], flow.shape[1], 3), dtype=np.uint8)
    hsv[..., 0] = (ang * 180 / np.pi / 2).astype(np.uint8)
    hsv[..., 1] = 255
    hsv[..., 2] = cv2.normalize(mag, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)

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
    if prev_gray is None:
        display_image = image.copy()
        mean_mag = 0.0
    else:
        flow = cv2.calcOpticalFlowFarneback(prev_gray, gray_image, None, **farneback_params)
        flow_bgr = flow_to_hsv_bgr(flow)
        display_image = flow_bgr if show_flow_visualization else image.copy()
        mag, _ = cv2.cartToPolar(flow[..., 0], flow[..., 1])
        mean_mag = float(np.mean(mag))

    cv2.putText(
        display_image,
        f"Mean flow magnitude: {mean_mag:.2f}",
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        display_image,
        f"Flow view: {'ON' if show_flow_visualization else 'OFF'} (press 'f')",
        (10, 60),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )

    cv2.imshow("Image Sequence", display_image)
    prev_gray = gray_image.copy()
    
    # Press 'q' to quit, wait 500ms between frames
    key = cv2.waitKey(30) & 0xFF
    if key == ord('f'):
        show_flow_visualization = not show_flow_visualization
    if key == ord('q'):
        break

cv2.destroyAllWindows()