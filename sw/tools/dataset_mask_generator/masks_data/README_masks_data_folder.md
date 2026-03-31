Object Detection Training & Testing Suite
This repository contains tools and datasets designed to facilitate manual labeling of training data and the subsequent validation of object detection models.

** Project Overview **
The primary purpose of this suite is to streamline the laborious process of creating "ground truth" masks for neural network training. It includes a custom-built GUI for labeling and various scripts to test the accuracy of generated models on real-world drone imagery.

**Important Notes on Training**
Vase Detection: An attempt was made to train a detector for vases. However, due to the small pixel size of the objects, training was unsuccessful. The masks are preserved in this repository for archival or future optimization.

Gate Detection: An improved detection process was tested for gates but did not yield successful results. The training data for gates remains available for further experimentation.

Model Generation: The logic for creating the detector (the actual training functions) is located one level up in the /neural_network directory.

**Repository Contents**
1. Training Data (Hand-Made Masks)
A collection of manually drawn masks used to train the software to recognize specific objects. These define the exact pixel areas the neural network should identify.

2. Mask Painter GUI (MaskPainter.py)
An AI-assisted graphical tool developed to simplify the manual labeling process.

Function: Allows users to "paint" over images to generate binary masks quickly.

Controls: * LMB: Paint | RMB: Erase

Scroll: Zoom | MMB Drag: Pan

Sliders: Adjust brush size and overlay opacity.

3. Validation Scripts
Scripts designed to run the trained detectors (typically .pkl models) on random batches of new images.

Visual Feedback: These scripts generate multi-panel figures comparing original images, raw classifier outputs, and filtered results (e.g., edge density filtering and bounding box clustering).

Randomized Testing: Automatically selects samples from drone download folders (e.g., 20260320) to verify real-world performance.

**Workflow Summary**
Labeling: Use MaskPainter.py to create masks for new training images.

Training: Utilize the scripts in the /neural_network folder to generate model files.

Testing: Run the detection scripts in this folder to visualize and evaluate the model's performance on unseen data.

**Technical Requirements**

Language: Python 3.x
Libraries: OpenCV, NumPy, Joblib, Matplotlib, Pillow (PIL), Tkinter