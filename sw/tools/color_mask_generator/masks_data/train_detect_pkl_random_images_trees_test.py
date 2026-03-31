import cv2
import numpy as np
import joblib
import matplotlib.pyplot as plt
import glob
import random

# Load models — instant, no retraining
tree_clf   = joblib.load('../tree_detector.pkl')
ground_clf = joblib.load('../ground_detector.pkl')
print('Models loaded.')

# Folder forimages to test on
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEV_DIR = os.path.dirname(os.path.dirname(os.path.dirname(SCRIPT_DIR)))
NEW_IMAGES_FOLDER = os.path.join(DEV_DIR, 'downloads from drone', '20260320')

# Options and settings
NUM_IMAGES  = 2 ** 2  # must be a perfect square (eg 4 or 16)
SAVE_FIGURE = True
SAVE_PATH   = '../results_figures/tree_filter_detection_extra_results.png'

# Edge filter options
EDGE_FILTER_ON         = True
MIN_BLOB_AREA          = 35  # minimum blob size
SMALL_BLOB_THRESHOLD   = 150  # blobs smaller than this skip edge density check
EDGE_DENSITY_THRESHOLD = 0.04    # fraction of blob pixels that must be edges
CANNY_LOW              = 50# Canny lower threshold
CANNY_HIGH             = 150 # Canny upper threshold
CANNY_BLUR             = 5  # kernel size (must be odd)
CANNY_SIGMA            = 0.000000000000000001   # blur strength

# Bounding box options
BBOX_ON              = True
SHOW_GROUND          = False
CLUSTER_DILATION     = 20
PAD_W_FRAC           = 0.15
PAD_TOP_FRAC         = 0.15
PAD_BOT_FRAC         = 0.15
MAX_ASPECT_RATIO     = 4.0
MIN_ASPECT_RATIO     = 0.2
TOO_CLOSE_THRESHOLD  = 0.25   # if detected pixels cover more than this fraction of the image we are too close


# Feature extractor (must remain same as training !!!!!!!!!!!!!!!!!!!)
def extract_features(yuv, hsv, lab, y, x):
    p_yuv = yuv[y, x]
    p_hsv = hsv[y, x]
    p_lab = lab[y, x]
    patch = yuv[y - 1:y + 2, x - 1:x + 2].reshape(-1, 3)
    mean  = patch.mean(axis=0)
    std   = patch.std(axis=0)
    grad  = np.abs(yuv[y, x + 1].astype(int) - yuv[y, x - 1].astype(int))
    return [
        int(p_yuv[0]), int(p_yuv[1]), int(p_yuv[2]),
        int(p_hsv[0]), int(p_hsv[1]), int(p_hsv[2]),
        int(p_lab[0]), int(p_lab[1]), int(p_lab[2]),
        *mean.tolist(), *std.tolist(), *grad.tolist()
    ]


# Run classifier on full image
def run_classifier(clf, img):
    h, w, d = img.shape
    yuv = cv2.cvtColor(img, cv2.COLOR_BGR2YUV)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    pixels = np.array([
        extract_features(yuv, hsv, lab, y, x)
        for y in range(4, h - 2)
        for x in range(2, w - 3)
    ])
    pred_flat   = clf.predict(pixels)
    msk_cropped = pred_flat.reshape(h - 6, w - 5)
    msk         = np.zeros((h, w), dtype=np.uint8)
    msk[4:h - 2, 2:w - 3] = msk_cropped
    return msk


# Edge density filter
"""Keeps blobs with leafy/textured internal edges (trees); Rejects flat surface false positives (floor, walls, panels)"""
def filter_by_edge_density(raw_mask, img):
    gray    = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (CANNY_BLUR, CANNY_BLUR), CANNY_SIGMA)
    edges   = cv2.Canny(blurred, CANNY_LOW, CANNY_HIGH)

    # Morphological cleanup (removing isolated pixels before contour analysis)
    kernel  = np.ones((3, 3), np.uint8)
    cleaned = cv2.morphologyEx(raw_mask.astype(np.uint8), cv2.MORPH_OPEN,  kernel)
    cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(cleaned, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    output_mask = np.zeros_like(raw_mask, dtype=np.uint8)

    for cnt in contours:
        area = cv2.contourArea(cnt)

        if area < MIN_BLOB_AREA:
            continue  # too small, than get rid of asap

        # Small but important blobs skip the edge check
        if area < SMALL_BLOB_THRESHOLD:
            cv2.drawContours(output_mask, [cnt], -1, 255, -1)
            print(f'  blob area={int(area):5d}px  → KEPT (small, skipping edge check)')
            continue

        # Larger blobs, well than check the internal edge density
        blob_mask        = np.zeros_like(raw_mask, dtype=np.uint8)
        cv2.drawContours(blob_mask, [cnt], -1, 255, -1)
        edges_inside     = cv2.bitwise_and(edges, blob_mask)
        edge_pixel_count = cv2.countNonZero(edges_inside)
        blob_pixel_count = cv2.countNonZero(blob_mask)
        edge_density     = edge_pixel_count / blob_pixel_count if blob_pixel_count > 0 else 0

        print(f'  blob area={int(area):5d}px  edge_density={edge_density:.3f}', end='  →  ')

        if edge_density >= EDGE_DENSITY_THRESHOLD:
            cv2.drawContours(output_mask, [cnt], -1, 255, -1)
            print('KEPT LEKKER ')
        else:
            print('REJECTED: IMMEDIATELY DESTROY AND GET RID OF NOWWWWWWWWWWWWWW X')

    return output_mask


# Bounding box generator with clustering included
"""Dilates tree mask merging nearby tree blobs into clusters then fits one padded bounding box per cluster also the 
Aspect ratio filter destroys thin false positives"""
def generate_tree_bounding_boxes(tree_mask, img_debug=None):
    h, w  = tree_mask.shape
    debug = img_debug.copy() if img_debug is not None else None

    # TOO CLOSE checking
    # If detected pixels cover a large fraction of the image the drone is so close the tree is taking the whole picture
    total_pixels    = h * w
    detected_pixels = cv2.countNonZero(tree_mask)
    coverage        = detected_pixels / total_pixels

    if coverage > TOO_CLOSE_THRESHOLD:
        print(f' WAY TOO CLOSE BRO: {coverage:.1%} of image is tree ')
        warning_box = {
            'x1'      : 10,
            'y1'      : 10,
            'x2'      : w - 10,
            'y2'      : h - 10,
            'too_close': True
        }
        if debug is not None:
            # Draw red border around entire image
            cv2.rectangle(debug, (10, 10), (w-10, h-10), (0, 0, 255), 3)
            cv2.putText(debug, f'TOO CLOSE ({coverage:.0%})',
                        (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        return [warning_box], debug

    # Cluster nearby blobs by the dilation
    cluster_kernel = np.ones((CLUSTER_DILATION * 2 + 1,
                              CLUSTER_DILATION * 2 + 1), np.uint8)
    dilated_mask   = cv2.dilate(tree_mask, cluster_kernel, iterations=1)

    contours, _ = cv2.findContours(dilated_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    boxes = []

    for cnt in contours:
        if cv2.contourArea(cnt) < MIN_BLOB_AREA:
            continue

        # Get the bounding rectangle from the dilated contour
        x, y, w_box, h_box = cv2.boundingRect(cnt)
        x     = max(0, x)
        y     = max(0, y)
        w_box = min(w - x, w_box)
        h_box = min(h - y, h_box)

        # Check actual tree pixels using the original undilated mask
        region        = tree_mask[y:y + h_box, x:x + w_box]
        actual_pixels = cv2.countNonZero(region)
        if actual_pixels < MIN_BLOB_AREA:
            continue

        # Add padding
        pad_w   = int(PAD_W_FRAC   * w_box)
        pad_top = int(PAD_TOP_FRAC * h_box)
        pad_bot = int(PAD_BOT_FRAC * h_box)

        x1 = max(0, x - pad_w)
        x2 = min(w, x + w_box + pad_w)
        y1 = max(0, y - pad_top)
        y2 = min(h, y + h_box + pad_bot)

        # AR filter
        box_h  = y2 - y1
        box_w  = x2 - x1
        aspect = box_h / max(1, box_w)

        if aspect > MAX_ASPECT_RATIO or aspect < MIN_ASPECT_RATIO:
            print(f'  Cluster at({x},{y}) aspect={aspect:.2f} → rejected (baaaaaaaaaaad aspect) X')
            continue

        print(f'  Cluster at ({x},{y}) size= ({w_box}x{h_box}) actual = {actual_pixels}px aspect={aspect:.2f} → KEPT SUPER STUFF')

        boxes.append({'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2, 'too_close': False})

        if debug is not None:
            cv2.rectangle(debug, (x1, y1), (x2, y2), (0, 255, 0), 2)

    return boxes, debug


# Drawing bounding boxes on the image
def draw_boxes(img, boxes):
    visual = img.copy()
    for box in boxes:
        if box.get('too_close', False):
            # Red border is because too close warning
            colour = (0, 0, 255)
            label  = 'TOO CLOSE'
        else:
            # Green box is box there is a normal tree detection
            colour = (0, 255, 0)
            label  = 'TREE'
        cv2.rectangle(visual, (box['x1'], box['y1']), (box['x2'], box['y2']), colour, 2)
        cv2.putText(visual, label,
                    (box['x1'], max(0, box['y1'] - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, colour, 1)
    return visual


# Pick random images from the folder
all_images = glob.glob(NEW_IMAGES_FOLDER + '\\*.jpg')
all_images = [i for i in all_images if '_mask' not in i]

if len(all_images) < NUM_IMAGES:
    print(f'Warning: only {len(all_images)} images found, using all of them.')
    selected = all_images
else:
    selected = random.sample(all_images, NUM_IMAGES)

print(f'Selected {len(selected)} random images.')

# Build the figure grid (original | raw | edge filtered | ground | bounding boxes)
num_panels = 2 # always: original + raw
if EDGE_FILTER_ON: num_panels += 1 # + edge filtered
if SHOW_GROUND: num_panels += 1 # + ground mask
if BBOX_ON: num_panels += 1 # + bounding boxes

cols_per_image = num_panels
grid_cols      = int(np.sqrt(NUM_IMAGES)) * cols_per_image
grid_rows      = int(np.sqrt(NUM_IMAGES))

fig, axes = plt.subplots(grid_rows, grid_cols, figsize=(grid_cols * 3, grid_rows * 3))
fig.suptitle('Tree Detection — Original | Raw | Edge Filtered | Ground | Bounding Box', fontsize=12)

# Run detection on each image
for idx, f in enumerate(selected):
    img     = cv2.imread(f)
    h, w, d = img.shape
    print(f'\n[{idx + 1}/{len(selected)}] {f}')

    # Run both classifiers
    raw_mask    = run_classifier(tree_clf,   img)
    ground_mask = run_classifier(ground_clf, img) if (BBOX_ON or SHOW_GROUND) else None

    # Apply edge density filter
    filtered_mask = filter_by_edge_density(raw_mask, img) if EDGE_FILTER_ON else raw_mask

    # Generate clustered bounding boxes
    boxes, _ = generate_tree_bounding_boxes(filtered_mask, img_debug=img) if BBOX_ON else ([], None)

    # Build the display panels
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    raw_overlay        = img.copy()
    raw_overlay[:,:,1] = raw_mask
    raw_rgb            = cv2.cvtColor(raw_overlay, cv2.COLOR_BGR2RGB)

    row      = idx // int(np.sqrt(NUM_IMAGES))
    col_pair = idx %  int(np.sqrt(NUM_IMAGES))
    col_base = col_pair * cols_per_image

    # Panel fro original image
    axes[row, col_base].imshow(img_rgb)
    axes[row, col_base].set_title(f'#{idx+1} Original', fontsize=7)
    axes[row, col_base].axis('off')

    # Panel for raw tree classifier output
    axes[row, col_base + 1].imshow(raw_rgb)
    axes[row, col_base + 1].set_title(f'#{idx+1} Raw', fontsize=7)
    axes[row, col_base + 1].axis('off')

    panel = 2  # next available panel slot

    # Panel for edge filtered (if on)
    if EDGE_FILTER_ON:
        filt_overlay        = img.copy()
        filt_overlay[:,:,1] = filtered_mask
        filt_rgb            = cv2.cvtColor(filt_overlay, cv2.COLOR_BGR2RGB)
        axes[row, col_base + panel].imshow(filt_rgb)
        axes[row, col_base + panel].set_title(f'#{idx+1} Edge Filtered', fontsize=7)
        axes[row, col_base + panel].axis('off')
        panel += 1

    # Panel for ground mask (if on)
    if SHOW_GROUND:
        ground_overlay        = img.copy()
        ground_overlay[:,:,1] = ground_mask
        ground_rgb            = cv2.cvtColor(ground_overlay, cv2.COLOR_BGR2RGB)
        axes[row, col_base + panel].imshow(ground_rgb)
        axes[row, col_base + panel].set_title(f'#{idx+1} Ground', fontsize=7)
        axes[row, col_base + panel].axis('off')
        panel += 1

    # Panel for bounding boxes overlaid on edge filtered tree mask (if on)
    if BBOX_ON:
        bbox_img        = img.copy()
        bbox_img[:,:,1] = filtered_mask
        bbox_rgb        = cv2.cvtColor(bbox_img, cv2.COLOR_BGR2RGB)
        bbox_with_boxes = draw_boxes(bbox_rgb, boxes)
        axes[row, col_base + panel].imshow(bbox_with_boxes)
        axes[row, col_base + panel].set_title(f'#{idx+1} Boxes', fontsize=7)
        axes[row, col_base + panel].axis('off')

plt.tight_layout()

if SAVE_FIGURE:
    plt.savefig(SAVE_PATH, dpi=150, bbox_inches='tight')
    print(f'\nFigure saved to {SAVE_PATH}')

plt.show()