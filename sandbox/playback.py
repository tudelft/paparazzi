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

# Create FAST detector and keep up to 50 strongest features.
fast_detector = cv2.FastFeatureDetector_create()
max_features = 50

# Lucas-Kanade optical flow parameters.
lk_params = dict(
    winSize=(21, 21),
    maxLevel=3,
    criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01),
)

prev_gray = None
prev_points = None


def detect_fast_points(gray_image, detector, count):
    keypoints = detector.detect(gray_image, None)
    keypoints = sorted(keypoints, key=lambda kp: kp.response, reverse=True)[:count]
    if not keypoints:
        return None
    points = np.array([kp.pt for kp in keypoints], dtype=np.float32).reshape(-1, 1, 2)
    return points

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

    if prev_gray is None or prev_points is None or len(prev_points) < max_features // 2:
        prev_points = detect_fast_points(gray_image, fast_detector, max_features)
        if prev_points is not None:
            for pt in prev_points.reshape(-1, 2):
                x, y = pt.astype(int)
                cv2.circle(display_image, (x, y), 3, (0, 255, 0), -1)
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

            for new_pt, old_pt in zip(good_new, good_old):
                new_x, new_y = new_pt.astype(int)
                old_x, old_y = old_pt.astype(int)
                cv2.line(display_image, (old_x, old_y), (new_x, new_y), (0, 255, 0), 2)
                cv2.circle(display_image, (new_x, new_y), 3, (0, 0, 255), -1)

            prev_points = good_new.reshape(-1, 1, 2).astype(np.float32)
        else:
            prev_points = None

    tracked_count = 0 if prev_points is None else len(prev_points)
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

    cv2.imshow("Image Sequence", display_image)
    prev_gray = gray_image.copy()
    
    # Press 'q' to quit, wait 500ms between frames
    if cv2.waitKey(200) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()