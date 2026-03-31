import cv2
import numpy as np
import joblib
import matplotlib.pyplot as plt
import glob
import random
import os

# Load the saved model (no retraining is needed) (it is one folder up for the convenience of other scripts)
clf = joblib.load('../gate_detector.pkl')
print('Model loaded.')

# Folder of NEW images to test on (absolute path has been  because this is just used for testing, change accordingly)
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEV_DIR = os.path.dirname(os.path.dirname(os.path.dirname(SCRIPT_DIR)))
NEW_IMAGES_FOLDER = os.path.join(DEV_DIR, 'downloads from drone', '20260320')

# Options / Settings
NUM_IMAGES  = 16  # how many random images to pick, must be a perfect square (4, 9, 16, 25, etc)
SAVE_FIGURE = True # set to False to just display without saving
SAVE_PATH   = '../results_figures/gate_detection_results.png'

# Same feature extractor as the training
def extract_features(yuv, hsv, lab, y, x):
    """
    Extract a feature vector for a single pixel using multiple color spaces
    and local neighborhood statistics.

    @param yuv: Image in YUV color space
    @param hsv: Image in HSV color space
    @param lab: Image in LAB color space
    @param y: Pixel row index
    @param x: Pixel column index

    @return: List representing the feature vector for the pixel
    """
    p_yuv = yuv[y, x]
    p_hsv = hsv[y, x]
    p_lab = lab[y, x]
    patch = yuv[y-1:y+2, x-1:x+2].reshape(-1, 3)
    mean  = patch.mean(axis=0)
    std   = patch.std(axis=0)
    grad  = np.abs(yuv[y, x+1].astype(int) - yuv[y, x-1].astype(int))
    return [
        int(p_yuv[0]), int(p_yuv[1]), int(p_yuv[2]),
        int(p_hsv[0]), int(p_hsv[1]), int(p_hsv[2]),
        int(p_lab[0]), int(p_lab[1]), int(p_lab[2]),
        *mean.tolist(), *std.tolist(), *grad.tolist()
    ]

# Pick random images from the folder
all_images = glob.glob(NEW_IMAGES_FOLDER + '\\*.jpg')
all_images = [i for i in all_images if '_mask' not in i]

if len(all_images) < NUM_IMAGES:
    print(f'Warning: only {len(all_images)} images found, using all of them.')
    selected = all_images
else:
    selected = random.sample(all_images, NUM_IMAGES)

print(f'Selected {len(selected)} random images.')

# Build the figurre
# Each image gets 2 columns (original and detection), rows are the  NUM_IMAGES divided by 2
cols_per_image = 2
grid_cols = int(np.sqrt(NUM_IMAGES)) * cols_per_image
grid_rows = int(np.sqrt(NUM_IMAGES))

fig, axes = plt.subplots(grid_rows, grid_cols, figsize=(grid_cols * 3, grid_rows * 3))
fig.suptitle('Pole Detection Results — Original (left) vs Detection (right)', fontsize=12)

# Run detection on each selected image and fill the grid
for idx, f in enumerate(selected):
    img = cv2.imread(f)
    h, w, d = img.shape
    print(f'[{idx+1}/{len(selected)}] Running on {f} ...')

    yuv = cv2.cvtColor(img, cv2.COLOR_BGR2YUV)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)

    pixels = np.array([
        extract_features(yuv, hsv, lab, y, x)
        for y in range(4, h-2)
        for x in range(2, w-3)
    ])

    y_pred_flat = clf.predict(pixels)

    msk_cropped = y_pred_flat.reshape(h-6, w-5)
    msk = np.zeros((h, w), dtype=np.uint8)
    msk[4:h-2, 2:w-3] = msk_cropped

    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    overlay = img.copy()
    overlay[:, :, 1] = msk[:, :]
    overlay_rgb = cv2.cvtColor(overlay, cv2.COLOR_BGR2RGB)

    # Work out which row and column pair this image goes in
    row      = idx // int(np.sqrt(NUM_IMAGES))
    col_pair = idx %  int(np.sqrt(NUM_IMAGES))
    col_orig = col_pair * cols_per_image        # original image column
    col_det  = col_pair * cols_per_image + 1    # detection image column

    axes[row, col_orig].imshow(img_rgb)
    axes[row, col_orig].set_title(f'#{idx+1} Original', fontsize=7)
    axes[row, col_orig].axis('off')

    axes[row, col_det].imshow(overlay_rgb)
    axes[row, col_det].set_title(f'#{idx+1} Detection', fontsize=7)
    axes[row, col_det].axis('off')

plt.tight_layout()

# Save + showing
if SAVE_FIGURE:
    plt.savefig(SAVE_PATH, dpi=150, bbox_inches='tight')
    print(f'Figure saved to {SAVE_PATH}')

plt.show()