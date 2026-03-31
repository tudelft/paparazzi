"""
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
!! WARNING: To set up the PARALLELISM in this tool which is only needed to speed up the mask generation process the   !!
!! assistance of the LLM ChatBot Claude has been used.                                                                !!
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

Generates binary masks (white = detected object or black = background) for every
image in a image folder and saves them into a set of output folders in a main folder.

Output folder structure

note this is just for poles, trees and the ground

<MASKS_OUTPUT_ROOT>/
    ground_masks/
        1_mask_ground.png
        2_mask_ground.png
        ...
    pole_masks/
        1_mask_pole.png
        2_mask_pole.png
        ...
    tree_masks/
        1_mask_tree.png
        2_mask_tree.png
        ...
"""

import cv2
import numpy as np
import joblib
import glob
import os
import multiprocessing as mp

# Settings

# Folder containing the images to process
# IMAGE_FOLDER = r'C:\Users\neytc\Documents\TU_Delft\lecture_notes\mav\MAV_CW\DEVELOPMENT\downloads from drone\20260320'
#
# # Where the three mask sub folders will be created
# MASKS_OUTPUT_ROOT = r'C:\Users\neytc\Documents\TU_Delft\lecture_notes\mav\MAV_CW\DEVELOPMENT\Python\neural_network\latest_flight_all_generated_masks'
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# Go up 2 levels: neural_network → Python → DEVELOPMENT
DEV_DIR = os.path.dirname(os.path.dirname(SCRIPT_DIR))

IMAGE_FOLDER = os.path.join(DEV_DIR, 'downloads from drone', '20260320')

MASKS_OUTPUT_ROOT = os.path.join(SCRIPT_DIR, 'latest_flight_all_generated_masks')

# .pkl model files location
GROUND_MODEL_PATH = 'ground_detector.pkl'
POLE_MODEL_PATH   = 'pole_detector.pkl'
TREE_MODEL_PATH   = 'tree_detector.pkl'

# Test mode
TEST_MODE       = True  # leave to false unless you test stuff
TEST_NUM_IMAGES = 30

# Parallel stuff
NUM_WORKERS = 10

# Tree edge-filter settings
EDGE_FILTER_ON         = True
MIN_BLOB_AREA          = 35
SMALL_BLOB_THRESHOLD   = 150
EDGE_DENSITY_THRESHOLD = 0.04
CANNY_LOW              = 50
CANNY_HIGH             = 150
CANNY_BLUR             = 5
CANNY_SIGMA            = 1e-18


# HELPERS Functions

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


def run_classifier(clf, img):
    """
    Apply a trained classifier to an image to produce a segmentation mask.

    @param clf: Trained classifier (e.g., RandomForest)
    @param img: Input image in BGR format

    @return: Predicted mask as a NumPy array (uint8)
    """
    h, w, _ = img.shape
    yuv = cv2.cvtColor(img, cv2.COLOR_BGR2YUV)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)

    y0, y1 = 4, h - 2
    x0, x1 = 2, w - 3

    # values for pixel
    yuv_px = yuv[y0:y1, x0:x1].astype(np.float64)
    hsv_px = hsv[y0:y1, x0:x1].astype(np.float64)
    lab_px = lab[y0:y1, x0:x1].astype(np.float64)

    # 3x3 patch mean and std over YUV
    neighbours = np.stack([
        yuv[y0-1:y1-1, x0-1:x1-1], yuv[y0-1:y1-1, x0:x1], yuv[y0-1:y1-1, x0+1:x1+1],
        yuv[y0:y1,     x0-1:x1-1], yuv[y0:y1,     x0:x1],  yuv[y0:y1,     x0+1:x1+1],
        yuv[y0+1:y1+1, x0-1:x1-1], yuv[y0+1:y1+1, x0:x1],  yuv[y0+1:y1+1, x0+1:x1+1],
    ], axis=0).astype(np.float64)

    patch_mean = neighbours.mean(axis=0)
    patch_std  = neighbours.std(axis=0)

    # Gradient (Make it float64 first to avoid uint8 underflow)
    grad = np.abs(yuv[y0:y1, x0+1:x1+1].astype(np.float64) -
                  yuv[y0:y1, x0-1:x1-1].astype(np.float64))

    # Concatenate features
    pixels = np.concatenate([
        yuv_px, hsv_px, lab_px,
        patch_mean, patch_std, grad
    ], axis=2).reshape(-1, 18)

    pred_flat   = clf.predict(pixels)
    msk_cropped = pred_flat.reshape(y1 - y0, x1 - x0)
    msk         = np.zeros((h, w), dtype=np.uint8)
    msk[y0:y1, x0:x1] = msk_cropped
    return msk

def binary_to_bw(mask):
    """
    Convert a mask to a strict black-and-white format (0 or 255).

    @param mask: Input mask array

    @return: Binary mask (uint8)
    """
    return np.where(mask > 0, 255, 0).astype(np.uint8)


def filter_tree_by_edge_density(raw_mask, img):
    """
    Filter a tree mask based on contour size and internal edge density.

    @param raw_mask: Initial predicted mask
    @param img: Original input image

    @return: Refined mask after edge-based filtering
    """
    gray    = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (CANNY_BLUR, CANNY_BLUR), CANNY_SIGMA)
    edges   = cv2.Canny(blurred, CANNY_LOW, CANNY_HIGH)

    kernel  = np.ones((3, 3), np.uint8)
    cleaned = cv2.morphologyEx(raw_mask.astype(np.uint8), cv2.MORPH_OPEN,  kernel)
    cleaned = cv2.morphologyEx(cleaned,                   cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(cleaned, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    output_mask = np.zeros_like(raw_mask, dtype=np.uint8)

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < MIN_BLOB_AREA:
            continue

        # Small blobs edge check skipping stuff
        if area < SMALL_BLOB_THRESHOLD:
            cv2.drawContours(output_mask, [cnt], -1, 255, -1)
            continue

        # Check internal edge density
        blob_mask        = np.zeros_like(raw_mask, dtype=np.uint8)
        cv2.drawContours(blob_mask, [cnt], -1, 255, -1)
        edges_inside     = cv2.bitwise_and(edges, blob_mask)
        edge_pixel_count = cv2.countNonZero(edges_inside)
        blob_pixel_count = cv2.countNonZero(blob_mask)
        edge_density     = edge_pixel_count / blob_pixel_count if blob_pixel_count > 0 else 0

        if edge_density >= EDGE_DENSITY_THRESHOLD:
            cv2.drawContours(output_mask, [cnt], -1, 255, -1)

    return output_mask


def make_tree_mask(tree_clf, img):
    """
    Generate a tree mask using a classifier and optional edge filtering.

    @param tree_clf: Trained tree detection model
    @param img: Input image

    @return: Binary tree mask
    """
    raw_mask = run_classifier(tree_clf, img)
    if EDGE_FILTER_ON:
        filtered = filter_tree_by_edge_density(raw_mask, img)
    else:
        filtered = raw_mask
    return binary_to_bw(filtered)


def save_image(path, img):

    _, buf = cv2.imencode('.png', img)
    with open(path, 'wb') as f:
        f.write(buf)
        f.flush()
        os.fsync(f.fileno())


# Worker Process  to allow for parallelism stuff

def process_image(args):
    """
    Worker function for parallel processing of images.

    Loads an image, applies classifiers (ground, pole, tree),
    and saves the generated masks.

    @param args: Tuple containing image path, output directories,
                 and model file paths

    @return: Image name or warning message
    """
    import os
    os.environ['LOKY_MAX_CPU_COUNT'] = '1'  # suppress annoying sklearn warning

    path, ground_dir, pole_dir, tree_dir, \
    ground_model_path, pole_model_path, tree_model_path = args

    stem = os.path.splitext(os.path.basename(path))[0]

    # Each worker loads its own copy of the models caause cant sahred job lib stuff
    ground_clf = joblib.load(ground_model_path)
    pole_clf   = joblib.load(pole_model_path)
    tree_clf   = joblib.load(tree_model_path)

    img = cv2.imread(path)
    if img is None:
        return f'WARNING: could not read {path}, skipping.'

    # Ground
    ground_bw = binary_to_bw(run_classifier(ground_clf, img))
    save_image(os.path.join(ground_dir, f'{stem}_mask_ground.png'), ground_bw)

    # Pole
    pole_bw = binary_to_bw(run_classifier(pole_clf, img))
    save_image(os.path.join(pole_dir, f'{stem}_mask_pole.png'), pole_bw)

    # Tree
    tree_bw = make_tree_mask(tree_clf, img)
    save_image(os.path.join(tree_dir, f'{stem}_mask_tree.png'), tree_bw)

    return stem

# Main function to execute the process

def main():
    """
    Execute the full mask generation pipeline.

    Loads images, processes them in parallel, and saves
    output masks into structured folders.

    @return: None
    """
    # Create output sub-folders
    ground_dir = os.path.join(MASKS_OUTPUT_ROOT, 'ground_masks')
    pole_dir   = os.path.join(MASKS_OUTPUT_ROOT, 'pole_masks')
    tree_dir   = os.path.join(MASKS_OUTPUT_ROOT, 'tree_masks')

    for d in (ground_dir, pole_dir, tree_dir):
        os.makedirs(d, exist_ok=True)
        print(f'Output folder ready: {d}')

    # Collecting images
    all_images = sorted(glob.glob(os.path.join(IMAGE_FOLDER, '*.jpg')) +
                        glob.glob(os.path.join(IMAGE_FOLDER, '*.png')))
    all_images = [p for p in all_images if '_mask' not in os.path.basename(p).lower()]

    if not all_images:
        print(f'\nNo images found in: {IMAGE_FOLDER}')
        return

    if TEST_MODE:
        import random
        all_images = random.sample(all_images, min(TEST_NUM_IMAGES, len(all_images)))
        print(f'\nTEST MODE — processing {len(all_images)} random images.\n')
    else:
        print(f'\nFound {len(all_images)} images to process.\n')

    # Build args list for workers
    args_list = [
        (path, ground_dir, pole_dir, tree_dir,
         GROUND_MODEL_PATH, POLE_MODEL_PATH, TREE_MODEL_PATH)
        for path in all_images
    ]

    # Run in parallel
    n_workers = NUM_WORKERS or mp.cpu_count()
    print(f'Using {n_workers} worker processes.\n')

    total = len(all_images)
    with mp.Pool(processes=n_workers) as pool:
        for i, result in enumerate(pool.imap_unordered(process_image, args_list), 1):
            print(f'[{i}/{total}]  {result}', flush=True)

    print(f'\nDone. All masks saved to: {MASKS_OUTPUT_ROOT}')


if __name__ == '__main__':
    mp.freeze_support()   # needed on Windows (linux people be careful)
    main()