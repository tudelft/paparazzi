# MAV 2026 Group 9 - CNN-Based MAVLab Gate Detection and Initial Navigation Integration on Paparazzi

This repository contains the implementation developed by **Group 9** for the **TU Delft Autonomous Flight of MAV course (AE4317 / MAVLab 2026)**.
The project extends **Paparazzi UAS** with an onboard gate detection pipeline based on a small CNN trained offline on images of TU Delft MAVLab gates, plus an initial navigation module that reacts to the detector output.

## Contributors

**Group:** 9  
**Submission date:** 31/03/2026

| Name              | NetID       | Student Number |
|-------------------|-------------|----------------|
| M. Sanz Piña      | msanzpina   | 6557368        |
| Tommaso Calzolari | tcalzolari  | 6430600        |
| Leonardo Pedretti | lpedretti   | 6432891        |
| D. Townsend       | dtownsed    | 6315577        |
| E. Bester         | ebester     | 6534899        |
| H. Kovács         | hkovacs     | 6549608        |

---

## PULL REQUEST
For pull request description look at PULL_REQUEST_README.md file.

## Project overview

This project implements a Bebop-based gate detection and gate-oriented navigation pipeline inside Paparazzi.
The system is centered around two custom modules:

- `gate_cnn_detector`, the computer vision module that receives images from the front camera, preprocesses them onboard, and runs a lightweight CNN inference pass fully in C
- `custom_avoider`, the navigation module that subscribes to `VISUAL_DETECTION` messages, searches for the gate, aligns the vehicle laterally, and then commits to a simple forward pass-through behavior

The CNN itself is not trained onboard. It was trained offline on images of MAVLab gates from TU Delft, then validated and tested outside the flight stack. The resulting trained weights were embedded directly in the Paparazzi module, so runtime execution only consists of image preprocessing and a forward pass through the network. This avoids external machine-learning libraries in the onboard path and keeps the detector lightweight and efficient.

The detector publishes its output through the standard `VISUAL_DETECTION` ABI message. `custom_avoider` consumes that message stream and uses the gate center together with fresh/no-detection semantics to decide whether to keep searching, align horizontally, move toward the gate, or continue a short blind pass-through. This navigation logic is intentionally an initial integration concept for demonstrating closed-loop use of the CNN detector. It is useful as a starting point, but it still needs further refinement and flight tuning before it should be treated as a robust autonomous gate navigation solution.

## Relevant files

Relevant XML integration files:

- `conf/userconf/tudelft/course_conf.xml`: registers the `bebop_custom_avoid` aircraft entry used from Paparazzi Center
- `conf/airframes/tudelft/custom_airframe.xml`: airframe wiring for the Bebop platform, camera modules, `gate_cnn_detector`, and `custom_avoider`
- `conf/modules/gate_cnn_detector.xml`: module definition for the CNN gate detector
- `conf/modules/custom_avoider.xml`: module definition for the navigation module that consumes `VISUAL_DETECTION`

Relevant source files:

- `sw/airborne/modules/computer_vision/gate_cnn_detector/gate_cnn_detector.c`: CNN inference, image preprocessing, prediction packaging, and message publication
- `sw/airborne/modules/computer_vision/gate_cnn_detector/gate_cnn_detector.h`: detector interface and prediction structure
- `sw/airborne/modules/computer_vision/gate_cnn_detector/gate_cnn_weights.c`: embedded trained CNN weights
- `sw/airborne/modules/computer_vision/gate_cnn_detector/gate_cnn_weights.h`: declarations for the embedded weights
- `sw/airborne/modules/custom_avoider/custom_avoider.c`: initial gate-oriented navigation logic built on top of detector output

## How it works

1. `video_capture` and the Bebop camera stack provide frames from the front camera.
2. `gate_cnn_detector` receives the image in the Paparazzi computer-vision pipeline, rescales it internally to the CNN input size, and runs a forward pass using embedded weights.
3. The detector converts the prediction into a `VISUAL_DETECTION` message containing gate presence, image coordinates, bounding-box values, confidence semantics, and image-width metadata.
4. `custom_avoider` subscribes to those messages and uses them to search for the gate, align the vehicle with the gate center, move forward, and perform a short blind continuation once the detection remains stable long enough.

This structure makes the detector usable as a standalone vision demo as well as part of a simple perception-to-navigation loop.


# MAIN README

Paparazzi UAS
=============
[![Build Status](https://paparazziuav.semaphoreci.com/badges/paparazzi/branches/master.svg?style=shields&key=d3a59143-a357-434e-89b8-057f15ed8dd2)](https://paparazziuav.semaphoreci.com/projects/paparazzi) [![Gitter chat](https://badges.gitter.im/paparazzi/discuss.svg)](https://gitter.im/paparazzi/discuss)
<a href="https://scan.coverity.com/projects/paparazzi-paparazzi">
  <img alt="Coverity Scan Build Status"
       src="https://scan.coverity.com/projects/4928/badge.svg"/>
</a>

Paparazzi is a free open source software package for Unmanned (Air) Vehicle Systems.
For many years, the system has been used successfuly by hobbyists, universities and companies all over the world, on vehicles of various sizes (11.9g to 25kg).
Paparazzi supports fixed wing, rotorcraft, hybrids, flapping vehicles and it is even possible to use it for boats and surface vehicles.

Documentation is available here https://paparazzi-uav.readthedocs.io/en/latest/

More docs is also available on the wiki http://wiki.paparazziuav.org

To get in touch, subscribe to the mailing list [paparazzi-devel@nongnu.org] (http://savannah.nongnu.org/mail/?group=paparazzi), the IRC channel (freenode, #paparazzi) and Gitter (https://gitter.im/paparazzi/discuss).

Required software
-----------------

Instructions for installation can be found on the wiki (http://wiki.paparazziuav.org/wiki/Installation).

Quick start:

```
git clone https://github.com/paparazzi/paparazzi.git
cd ./paparazzi
./install.sh
```



For Ubuntu users, required packages are available in the [paparazzi-uav PPA] (https://launchpad.net/~paparazzi-uav/+archive/ppa),
Debian users can use the [OpenSUSE Build Service repository] (http://download.opensuse.org/repositories/home:/flixr:/paparazzi-uav/Debian_7.0/)

Debian/Ubuntu packages:
- **paparazzi-dev** is the meta-package on which the Paparazzi software depends to compile and run the ground segment and simulator.
- **paparazzi-jsbsim** is needed for using JSBSim as flight dynamics model for the simulator.

Recommended cross compiling toolchain: https://launchpad.net/gcc-arm-embedded


Directories quick and dirty description:
----------------------------------------

_conf_: the configuration directory (airframe, radio, ... descriptions).

_data_: where to put read-only data (e.g. maps, terrain elevation files, icons)

_doc_: documentation (diagrams, manual source files, ...)

_sw_: software (onboard, ground station, simulation, ...)

_var_: products of compilation, cache for the map tiles, ...


Compilation and demo simulation
-------------------------------

1. type "make" in the top directory to compile all the libraries and tools.

2. "./paparazzi" to run the Paparazzi Center

3. Select the "Bixler" aircraft in the upper-left A/C combo box.
  Select "sim" from upper-middle "target" combo box. Click "Build".
  When the compilation is finished, select "Simulation" in Operation tab and click "Start Session".

4. In the GCS, wait about 10s for the aircraft to be in the "Holding point" navigation block.
  Switch to the "Takeoff" block (lower-left blue airway button in the strip).
  Takeoff with the green launch button.

Uploading the embedded software
----------------------------------

1. Power the flight controller board while it is connected to the PC with the USB cable.

2. From the Paparazzi center, select the "ap" target, and click "Upload".


Flight
------

1.  From the Paparazzi Center, select the flight session and ... do the same as in simulation !
