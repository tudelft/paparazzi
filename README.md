# Green Attractor

## Demo
The testing location has different obstacles.
![Obstacles in the CyberZoo](/images/obstacles.png)
The navigation for the drone is decided using this state diagram.
![Flowchart](/images/flowchart.png)
The detection for competition is done using a vertical green colorfilter that navigates on the availablity of green in the detection box.
![Horizontal colorfilter box](/images/horizontal_bar.png)
The extension for detecting a mat is a vertical bar with the same green filter, that looks over the mat in order keep flying. It looks ahead and takes over when the horizontal bar fails.
![Vertical colorfilter box](/images/vertical_bar.png)
## Videos showing the flight in simulation

[![Standaard Video](/imagegs/standaard.png)](https://youtu.be/LPpJ-rpNmqc "Mat avoidance")
[![Mat avoidance ](/imagegs/mat.png)](https://youtu.be/DE0FiWiZbzM "Mat avoidance")
[![Orange Maze ](/imagegs/orange_maze.png)](https://youtu.be/Ya5rIKcP2Ik "Orange Maze")



## Setup for execution

### Clone the code
```console
mkdir paparazzi
cd ~/paparazzi
git remote add origin https://github.com/dirkdefuijk/paparazzi
git fetch tudelft green_attractor_refactored
git checkout green_attractor_refactored
```

### Sync the modules
```console
git submodule init
git submodule sync
git submodule update
```

### Build Paparazzi by using:
```console
make clean
make
```

### Starting paparazzi
```console
python start.py
```
Select:
Conf: userconf/tudelft/course conf.xml
Controlpanel: userconf/tudelft/course control panel.xml .
Click ‘Set active’ and close the dialog.

```console
sudo apt install ffmpeg vlc cmake jstest-gtk default-jre
sudo apt install gazebo9 libgazebo9-dev
```

Build OpenCV for the Bebop and install requirements
```console
cd ~/paparazzi/sw/ext/opencv_bebop
sudo apt install libjpeg-turbo8-dev libpng-dev libtiff-dev zlib1g-dev libdc1394-22-dev
make
```

Run paparazzi
```console
cd ~/paparazzi
./paparazzi
```

Paparazzi UAS
=============

[![Build Status](https://travis-ci.org/paparazzi/paparazzi.png?branch=master)](https://travis-ci.org/paparazzi/paparazzi)


Paparazzi is a free open source software package for Unmanned (Air) Vehicle Systems.
For many years, the system has been used successfuly by hobbyists, universities and companies all over the world, on vehicles of various sizes (11.9g to 25kg).
Paparazzi supports fixed wing, rotorcraft, hybrids, flapping vehicles and it is even possible to use it for boats and surface vehicles.

Up to date information is available on the wiki http://wiki.paparazziuav.org

To get in touch, subscribe to the mailing list [paparazzi-devel@nongnu.org] (http://savannah.nongnu.org/mail/?group=paparazzi), the IRC channel (freenode, #paparazzi) and Gitter (https://gitter.im/paparazzi/discuss).

Required software
-----------------

Instructions for installation can be found on the wiki (http://wiki.paparazziuav.org/wiki/Installation).

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

3. Select the "Microjet" aircraft in the upper-left A/C combo box.
  Select "sim" from upper-middle "target" combo box. Click "Build".
  When the compilation is finished, select "Simulation" from
  the upper-right session combo box and click "Execute".

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
