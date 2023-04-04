# AE4317: Group 3

## Farneback Dense Optical Flow

In this project, we used dense optical flow to make an obstacle avoidance algorithm for a bebop2 drone.

## Contents

This project consists of 3 separate parts:

1. `farneback_navigator`
2. `farneback_calculator`
3. `finite_state_machine`

These three parts are together used to go from camera images to a navigation instruction.

The code uses the airframe `bebop_opticflow_farneback.xml`, which calls `cv_farneback.xml`. From `cv_farneback.xml`, our program is compiled.

### `farneback_navigator`

This code forms the main part of the program. This is where the callback function to the camera_images is called. From here we give the instructions to the `farneback_calculator` and the `finite_state_machine`. Furthermore, some debugging tools are provided here.

### `farneback_calculator`

Here we determine the amount of flow in the left, middle and the right area.

### `finite_state_machine`

This is where the decision-making happens. Here a finite state machine is fed with the optical flow in the three respective area's and based on that, we make a decision on wheter to turn right or left or move forward.