#=========================================================
#GATE SYNTHETIC DATA GENERATOR - DOCUMENTATION
#=========================================================
#
#1. OVERVIEW
#This script generates a synthetic dataset of "gate" images
#used for training computer vision models to detect gates
#and estimate vertical heading angles.
#
#2. REQUIREMENTS
#- Python 3.x
#- OpenCV (cv2)
#- NumPy
#
#Install dependencies via:
#pip install opencv-python numpy
#
#3. SETUP
#- Place 'gate_template.png' in the same folder as the script.
#- The template should have a transparent background (alpha channel).
#
#4. HOW IT WORKS
#The script follows a multi-stage augmentation pipeline:
#- Local Warp: Simulates lens or physical distortion.
#- Perspective: Applies sideways tapering (Yaw) to the gate.
#- Random Overlay: Places the gate on a random-gray background.
#- Interference: Adds blue "dead pixel" noise and Gaussian noise.
#- Motion Blur: Simulates camera movement.
#
#5. OUTPUTS
#All files are saved to the 'synthetic_data/' directory:
#- /img: Contains .jpg files (sf_0000.jpg, etc.)
#- labels.json: Contains metadata for each image.
#
#6. LABEL DATA FORMAT
#- img_name: Filename.
#- heading_angle: Normalized Y-coordinate (-1.0 to 1.0).
#- confidence: 1.0 (gate present) or 0.0 (no gate).
#
#7. CONFIGURATION
#You can adjust the constants at the top of the script:
#- NUM_IMAGES: Total files to generate.
#- IMG_H / IMG_W: Output resolution.
#- YAW_DISTORT_MAX: Intensity of perspective warping.
#
#=========================================================

import cv2
import numpy as np
import os
import json
import random

# ============================================================================
# CONFIGURATION
# ============================================================================
NUM_IMAGES = 5000                   
OUTPUT_DIR = "synthetic_data/img"   
IMG_H, IMG_W = 520, 240             
TEMPLATE_PATH = "gate_template.png" 

YAW_DISTORT_MAX = 0.20  

# ============================================================================
# AUGMENTATION FILTERS
# ============================================================================

def distort_template(image):
    """Slightly warps the template to simulate sagging or lens distortion."""
    h, w = image.shape[:2]
    # Create a mesh grid
    flex = random.uniform(2.0, 5.0)
    map_x = np.zeros((h, w), np.float32)
    map_y = np.zeros((h, w), np.float32)
    
    # Apply a wavy/sinusoidal distortion
    for i in range(h):
        for j in range(w):
            map_x[i, j] = j + flex * np.sin(i / 50.0)
            map_y[i, j] = i + flex * np.cos(j / 50.0)
            
    return cv2.remap(image, map_x, map_y, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=(0,0,0,0))

def add_blue_dots(image, count=15):
    """Adds small blue 'dead pixels' or interference noise."""
    h, w = image.shape[:2]
    for _ in range(count):
        # Random position
        ty = random.randint(0, h-1)
        tx = random.randint(0, w-1)
        # Random size (1 to 2 pixels)
        size = random.randint(1, 3)
        # Blue color (BGR: Blue, Green, Red)
        cv2.circle(image, (tx, ty), size, (random.randint(200, 255), random.randint(0, 50), random.randint(0, 50)), -1)
    return image

def add_gaussian_noise(image):
    row, col, ch = image.shape
    var = random.uniform(10, 30)
    sigma = var**0.5
    gauss = np.random.normal(0, sigma, (row, col, ch))
    noisy = image + gauss
    return np.clip(noisy, 0, 255).astype(np.uint8)

def apply_color_distort(image):
    img = image.astype(np.float32)
    brightness = random.uniform(-30, 30)
    contrast = random.uniform(0.8, 1.2)
    img = img * contrast + brightness
    return np.clip(img, 0, 255).astype(np.uint8)

def apply_motion_blur(image):
    if random.random() > 0.7:
        size = random.choice([3, 5])
        kernel = np.zeros((size, size))
        kernel[int((size-1)/2), :] = np.ones(size)
        kernel = kernel / size
        return cv2.filter2D(image, -1, kernel)
    return image

# ============================================================================
# TRANSFORMATION FUNCTIONS
# ============================================================================

def apply_sideways_taper(image, distort_limit=0.2):
    h, w = image.shape[:2]
    src_pts = np.float32([[0, 0], [w, 0], [w, h], [0, h]])
    taper = random.uniform(distort_limit * 0.5, distort_limit)
    if random.random() > 0.5:
        dst_pts = np.float32([[0 + (w * taper / 2), 0], [w - (w * taper / 2), 0], [w, h], [0, h]])
    else:
        dst_pts = np.float32([[0, 0], [w, 0], [w - (w * taper / 2), h], [0 + (w * taper / 2), h]])
    matrix = cv2.getPerspectiveTransform(src_pts, dst_pts)
    return cv2.warpPerspective(image, matrix, (w, h), borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0, 0))

def rotate_2d(image, angle):
    (h, w) = image.shape[:2]
    M = cv2.getRotationMatrix2D((w // 2, h // 2), angle, 1.0)
    cos, sin = np.abs(M[0, 0]), np.abs(M[0, 1])
    nW, nH = int((h * sin) + (w * cos)), int((h * cos) + (w * sin))
    M[0, 2] += (nW / 2) - (w // 2); M[1, 2] += (nH / 2) - (h // 2)
    return cv2.warpAffine(image, M, (nW, nH), borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0, 0))

def overlay_transparent(bg, ov, x, y):
    h, w = ov.shape[:2]; bh, bw = bg.shape[:2]
    x1, x2, y1, y2 = max(0, x), min(bw, x + w), max(0, y), min(bh, y + h)
    if x1 >= x2 or y1 >= y2: return bg
    ov_crop = ov[max(0, -y):max(0, -y)+(y2-y1), max(0, -x):max(0, -x)+(x2-x1)]
    alpha = ov_crop[:, :, 3] / 255.0
    for c in range(3):
        bg[y1:y2, x1:x2, c] = alpha * ov_crop[:, :, c] + (1 - alpha) * bg[y1:y2, x1:x2, c]
    return bg

# ============================================================================
# MAIN GENERATOR
# ============================================================================

def generate_dataset():
    print(f"START Generation: {NUM_IMAGES} images with Blue Noise & Warp")
    gate_template = cv2.imread(TEMPLATE_PATH, cv2.IMREAD_UNCHANGED)
    if gate_template is None:
        print("ERROR: Template not found!"); return
        
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    labels, headings_list = [], []

    for i in range(NUM_IMAGES):
        bg_val = random.randint(30, 80)
        img = np.full((IMG_H, IMG_W, 3), bg_val, dtype=np.uint8)
        
        has_gate = random.random() > 0.20 # Higher "no-gate" chance to help confidence
        
        if has_gate:
            scale = random.uniform(0.12, 0.50)
            resized = cv2.resize(gate_template, (int(gate_template.shape[1]*scale), int(gate_template.shape[0]*scale)))
            
            # --- 1. LOCAL TEMPLATE DISTORTION ---
            resized = distort_template(resized)
            
            p_gate = apply_sideways_taper(resized, YAW_DISTORT_MAX)
            final_gate = rotate_2d(p_gate, random.uniform(-4, 4))
            gh, gw = final_gate.shape[:2]
            
            cx = random.randint(min(gw//2, IMG_W-gw//2), max(gw//2, IMG_W-gw//2))
            side = 1 if random.random() > 0.5 else -1
            push = random.random() ** 0.6
            cy = int((IMG_H/2) + (side * push * (IMG_H/2 - gh/2 - 10)))

            img = overlay_transparent(img, final_gate, cx - gw // 2, cy - gh // 2)
            heading = np.clip((cy / IMG_H) * 2.0 - 1.0, -1.0, 1.0)
            confidence = 1.0
            headings_list.append(heading)
        else:
            heading, confidence = 0.0, 0.0

        # --- 2. GLOBAL FILTERS & BLUE NOISE ---
        img = apply_color_distort(img)
        img = add_gaussian_noise(img)
        img = add_blue_dots(img, count=random.randint(5, 20)) # Blue interference
        img = apply_motion_blur(img)
        
        img = cv2.GaussianBlur(img, (3,3), 0)

        img_filename = f"sf_{i:04d}.jpg"
        cv2.imwrite(os.path.join(OUTPUT_DIR, img_filename), img)
        labels.append({"img_name": img_filename, "heading_angle": heading, "confidence": confidence})

    with open("synthetic_data/labels.json", "w") as f:
        json.dump(labels, f, indent=4)
    
    print(f"\n✓ Done! Usable gates: {len(headings_list)}")

if __name__ == "__main__":
    generate_dataset()
