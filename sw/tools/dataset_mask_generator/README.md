# Mask Generation & Detector Training Pipeline

## Overview

This module provides a pipeline for generating high-quality binary masks for different object classes (e.g. ground, poles, trees). These masks can then be used to train machine learning or neural network models for image segmentation tasks.

The workflow consists of two main steps:

1. Train pixel-wise classifiers → produces `.pkl` model files  
2. Generate masks using trained models → produces binary mask images  

Note:  
This pipeline is not intended for real-time use on the drone due to its high computational cost. It is designed for offline data labeling and dataset preparation.

---

## Pipeline Summary

### Step 1 — Train Detector

Use the `train_detector` function to train a model for a specific object class.

Input:
- Folder with raw images
- Folder with corresponding ground-truth masks (`*_mask.jpg`)

Output:
- A trained classifier saved as a `.pkl` file

You need to train one model per object class, for example:
- `ground_detector.pkl`
- `pole_detector.pkl`
- `tree_detector.pkl`

---

### Step 2 — Generate Masks

Once the models are trained, use the mask generation script to process images.

This script will:
- Load each trained model
- Apply it pixel-wise to every image
- Generate binary masks for each class
- Save them into structured output folders

Output folder structure (name chosen):
```
<MASKS_OUTPUT_ROOT_FOLDER>/
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
```

---

## Features

- Pixel-wise classification using handcrafted features
- Supports multiple object classes
- Parallel processing for faster mask generation
- Optional post-processing for improved tree detection (edge-based filtering)

---

## Testing

- Example data and scripts are provided in the `masks_data` folder
- The pipeline can be tested on different object types
- A dedicated tree pipeline is included for improved accuracy using edge filtering

---

## Additional Tools

The repository includes tools for rapid mask creation, which can help accelerate the labeling process. These tools can also be adapted for use with YUV-based masks.

---

## Notes

- The quality of the generated masks depends on the quality of the training data
- Ensure that image and mask pairs are correctly aligned
- Large datasets and high sampling rates will increase processing time significantly

---

## Typical Usage Workflow

1. Prepare a labeled dataset (images + masks)
2. Train detector(s) using `train_detector`
3. Run the mask generation script
4. Use generated masks for:
   - Neural network training
   - Dataset augmentation
   - Evaluation

---

## Summary

This pipeline is a data preparation tool designed to:
- Automate mask generation
- Improve labeling efficiency
- Support downstream machine learning workflows