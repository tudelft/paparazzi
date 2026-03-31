"""
Pixel-wise Object Detector Training Script
------------------------------------------

This script trains a RandomForest-based pixel classifier using paired image
and mask datasets. Each pixel is classified as either foreground (object)
or background based on hand-labeled mask images.

The model uses handcrafted features derived from multiple color spaces
(YUV, HSV, LAB) along with local neighborhood statistics and gradient
information to improve detection robustness.

Main functionality:
- Loads image/mask pairs from specified folders
- Randomly samples pixels to build a training dataset
- Extracts per-pixel feature vectors (21 features)
- Trains a RandomForestClassifier
- Evaluates accuracy on a held-out test set
- Saves the trained model to disk
- Optionally visualizes predictions on training images

Originally designed for ground detection, but can be adapted for any
binary segmentation task with appropriate masks.

Dependencies:
    - OpenCV (cv2)
    - NumPy
    - scikit-learn
    - matplotlib
    - joblib

Author: [Your Name]
Date: [Submission Date]
"""

import cv2
import numpy as np
import glob
import os
from random import randrange
import joblib

from sklearn.model_selection import train_test_split
from sklearn.ensemble import RandomForestClassifier
from sklearn.metrics import accuracy_score
import matplotlib.pyplot as plt

# script was originally made to detect the ground and therefore may have references to the ground

def train_detector(
    images_folder, # path to folder with  raw images
    masks_folder, # path to folder containing ......_mask.jpg ground truth files that were hand labeled
    model_save_path  = 'detector.pkl', # where to save the trained model
    show_figures = False, # switch: set True to show results on training images after training
    samples_per_image = 300000, # number of random pixels sampled for each image
    maxfiles = 500, # safety cap on the number of images that have to be processed
):
    """
    Train a pixel-wise object detector using a Random Forest classifier.

    This function builds a dataset by randomly sampling pixels from input
    images and extracting feature vectors for each pixel. Labels are obtained
    from corresponding mask images, where pixel intensity indicates foreground
    or background. The trained model is saved to disk and optionally visualized.

    Parameters
    ----------
    images_folder : str
        Path to the folder containing input images.

    masks_folder : str
        Path to the folder containing ground truth mask images.
        Mask filenames must follow the pattern: <image_name>_mask.jpg.

    model_save_path : str, optional (default='detector.pkl')
        File path where the trained model will be saved.

    show_figures : bool, optional (default=False)
        If True, displays side-by-side visualizations of original images
        and predicted masks after training.

    samples_per_image : int, optional (default=300000)
        Number of random pixels sampled per image to build the dataset.

    maxfiles : int, optional (default=500)
        Maximum number of image/mask pairs to process (safety limit).

    Returns
    -------
    clf : sklearn.ensemble.RandomForestClassifier
        The trained Random Forest classifier.

    Notes
    -----
    - Features include color values in YUV, HSV, and LAB color spaces,
      as well as local neighborhood statistics and gradient information.
    - Mask pixels are binarized using a threshold of 127.
    - Stratified train/test split is used to preserve class balance.
    - The function assumes masks align perfectly with input images.
    """

    # NOTE: modify this function only if you know what you're doing. It is finely hand-tuned.
    def extract_features(yuv, hsv, lab, y, x):
        """
        Extract a feature vector for a single pixel.

        Features include:
        - Pixel color values in YUV, HSV, and LAB color spaces
        - Mean and standard deviation of a 3x3 neighborhood (YUV)
        - Horizontal gradient in YUV space

        Parameters
        ----------
        yuv, hsv, lab : np.ndarray
            Image representations in different color spaces.

        y : int
            Vertical pixel coordinate.

        x : int
            Horizontal pixel coordinate.

        Returns
        -------
        list of int/float
            A 21-dimensional feature vector describing the pixel.
        """
        p_yuv = yuv[y, x] # pixel values in YUV
        p_hsv = hsv[y, x] # pixel values in HSV
        p_lab = lab[y, x] # pixel values in LAB

        patch = yuv[y-1:y+2, x-1:x+2].reshape(-1, 3)  # 3x3 neighbourhood around pixel in YUV
        mean  = patch.mean(axis=0) # mean color of neighbourhood (smooths noise)
        std   = patch.std(axis=0)  # standard deviation of neighbourhood (high is edge and low if flatish)
        grad  = np.abs(yuv[y, x+1].astype(int) - yuv[y, x-1].astype(int))  # horizontal gradient

        # Return all features into one flat list:
        return [
            int(p_yuv[0]), int(p_yuv[1]), int(p_yuv[2]), # Y, U, V
            int(p_hsv[0]), int(p_hsv[1]), int(p_hsv[2]), # H, S, V
            int(p_lab[0]), int(p_lab[1]), int(p_lab[2]), # L, A, B
            *mean.tolist(), *std.tolist(), *grad.tolist()  # neighbourhood stats + gradient info stuff
        ]

    # Load image/mask pairs (scans the masks folder for all *_mask.jpg then finds correct images in the images fodler)
    # labels = glob.glob(masks_folder + '\\*_mask.jpg', recursive=True)
    labels = glob.glob(masks_folder + '\\*_mask.jpg', recursive=True)

    images = []
    for lf in labels:
        basename = os.path.basename(lf).replace('_mask.jpg', '')
        # matches  = glob.glob(images_folder + '\\' + basename + '.*')
        matches = glob.glob(os.path.join(images_folder, basename + '.*'))
        matches  = [m for m in matches if '_mask' not in m]
        if matches:
            images.append(matches[0])

    print(f'Found {len(images)} image/mask pairs.')
    print(images)

    # Build dataset

    # randomly samples pixels in images and records their feature vector (X_vec) and if they are objects or not lables (y_vec).
    X_vec = []  # feature vectors for a sampled pixel
    y_vec = []  # labels (255 = target object or for 0 = background)
    files_remaining = maxfiles

    for f in images:
        basename = os.path.splitext(os.path.basename(f))[0]  # filename without extension
        lf       = os.path.join(masks_folder, basename + '_mask.jpg')  # expected mask path

        if os.path.exists(lf):
            files_remaining -= 1
            if files_remaining <= 0:
                break  # max files stuff

            img = cv2.imread(f)   # load original image in BGR
            if img is None: continue

            msk = cv2.imread(lf)  # load mask image in BGR
            h, w, d = img.shape

            print('img=', f, 'lbl=', lf, w, 'x', h)

            # Convert to all three color spaces
            yuv = cv2.cvtColor(img, cv2.COLOR_BGR2YUV)
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)

            img[:, :, 1] = msk[:, :, 0]  # overlay mask on green channel for a visual reference

            # Randomly sampling pixels from this image
            for i in range(samples_per_image):
                # to make sure the borders of the image dont make the process get ruined
                x = randrange(2, w-3)  # x margin of 2 keeps 3x3 in image bounds
                y = randrange(4, h-2)  # y margin of 4 keeps 3x3 in image bounds

                # Binary mask stuff
                m = int(msk[y, x, 0])
                m = 255 if m >= 127 else 0

                X_vec.append(extract_features(yuv, hsv, lab, y, x)) # features
                y_vec.append([m]) #  label

    print('Dataset', len(X_vec), len(y_vec))

    # Train/test split (80% training, 20% testing), also stratify used to make sure 80% and 20% have the same background ratio
    X_train, X_test, y_train, y_test = train_test_split(
        X_vec, y_vec, test_size=0.2, stratify=y_vec, random_state=1
    )

    print('Train', len(X_train), '| Test', len(X_test))

    # Train
    clf = RandomForestClassifier(
        n_estimators=200, # number of trees
        max_depth=None, # trees grow until leaves are pure
        class_weight='balanced', # upweights rare target class to prevent bias toward background
        n_jobs=-1,
        random_state=0  # seed
    )
    clf.fit(X_train, np.array(y_train).ravel())

    # Evaluating
    y_pred = clf.predict(X_test)
    score  = accuracy_score(y_test, y_pred)
    print('Sensitivity:', round(score, 3))

    # Save model as *.pkl
    joblib.dump(clf, model_save_path)
    print(f'Model saved to {model_save_path}')

    # Visualise  training
    if show_figures:
        for f in images:
            img      = cv2.imread(f)
            h, w, d  = img.shape

            yuv = cv2.cvtColor(img, cv2.COLOR_BGR2YUV)
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)

            pixels = np.array([
                extract_features(yuv, hsv, lab, y, x)
                for y in range(4, h-2)
                for x in range(2, w-3)
            ])

            # Run classifier on all pixels
            y_pred_flat = clf.predict(pixels)

            # Remake prediction as a 2D mask
            msk_cropped = y_pred_flat.reshape(h-6, w-5)
            msk = np.zeros((h, w), dtype=np.uint8)  # full canvas and borders stay black
            msk[4:h-2, 2:w-3] = msk_cropped # predictions

            # Overlay
            img_rgb          = cv2.cvtColor(img, cv2.COLOR_BGR2RGB) # original
            overlay          = img.copy()
            overlay[:, :, 1] = msk[:, :] # green = detections
            overlay_rgb      = cv2.cvtColor(overlay, cv2.COLOR_BGR2RGB)

            # Side by side dispaly
            fig, axes = plt.subplots(1, 2, figsize=(12, 5))
            axes[0].imshow(img_rgb);     axes[0].set_title('Original');   axes[0].axis('off')
            axes[1].imshow(overlay_rgb); axes[1].set_title('Detection');  axes[1].axis('off')
            plt.suptitle(f, fontsize=8)
            plt.tight_layout()
            plt.show()

    return clf


# make sure ot se the correct paths here: currenlty set to gate detection
if __name__ == '__main__':
    base = os.path.dirname(os.path.abspath(__file__))
    train_detector(
        images_folder   = os.path.join(base, 'downloads from drone', '20260320'),
        masks_folder    = os.path.join(base, 'masks_data', 'gate_detection_masks'),
        model_save_path = os.path.join(base, 'gate_detector.pkl'),
        show_figures    = True,  # flip to True to see visualisations stuff
    )

