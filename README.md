# MAIN README

Paparazzi UAS
=============

[![Build Status](https://travis-ci.org/paparazzi/paparazzi.png?branch=master)](https://travis-ci.org/paparazzi/paparazzi) [![Gitter chat](https://badges.gitter.im/paparazzi/discuss.svg)](https://gitter.im/paparazzi/discuss)
[![Codacy Badge](https://api.codacy.com/project/badge/Grade/811c4398588f435fa8bc926f53d40e9f)](https://app.codacy.com/app/gautierhattenberger/paparazzi?utm_source=github.com&utm_medium=referral&utm_content=paparazzi/paparazzi&utm_campaign=Badge_Grade_Dashboard)
<a href="https://scan.coverity.com/projects/paparazzi-paparazzi">
  <img alt="Coverity Scan Build Status"
       src="https://scan.coverity.com/projects/4928/badge.svg"/>
</a>

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

# Bang-Bang Controller
This MPC approaches time-optimal principles by saturating the roll or pitch angle and optimizing the time at which it switches from acceleration to braking such that it reaches the positional target at a predefined desired speed. 

##Define quadcopter model parameters 
In [bangbang.c](sw/airborne/modules/ctrl/dronerace/bangbang.c):
- Drag coefficient: `Cd`
- Mass : `mass` 

### Compensation Estimators
In [compensation.c](sw/airborne/modules/ctrl/dronerace/compensation.c) define estimator values for pitch forward, pitch backward and roll.
Currently configured for first order polynomials to estimate time, position and velocity changes during the transition from accelerating to braking. 

## Define flightplan
The native paparazzi flightplan functionality is not used. Instead, the flightplan is defined in [flightplan_Bang.c](sw/airborne/modules/ctrl/dronerace/flightplan_Bang.c).
The elements of `bangbang_fp_struct` are indicated below. Each row of this struct corresponds with a single waypoint.
1. gate number 
2. x position
3. y position
4. z position
5. gate heading 
   * Commanded heading while flying to this gate
6. desired speed at gate
7. gate type
   * Choose `STARTGATE`,`GATE`or `ENDGATE` 
8.  Controller type
     * Select which controller must be used `BANGBANG`, `PID` or  `HIGHPID` (pid with higher gain values)
9. turning 
    * This variable indicates whether the drone is turning, which puts the bangbang controller temporarily on pause. This variable is assigned automatically.
10. Forced heading
     * give a forced heading. Only used when `overwrite_psi` is active (next element)
11. `overwrite_psi` 
     * boolean that enables the use of a forced heading. If `false` the heading will automatically be calculated such the the nose faces the target
12.  Saturation angle
       * Set maximum absolute angle for the critical axis