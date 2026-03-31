# Pull Request Description - Group 9 MAVLab Gate CNN Integration

## Summary

This pull request integrates a lightweight CNN-based gate detector for TU Delft MAVLab gates into Paparazzi and connects it to an initial navigation module for Bebop-based experiments.

The detector is implemented as a Paparazzi computer-vision module and runs fully onboard in C, without relying on external machine-learning libraries during inference. The network was trained offline on images of MAVLab gates from TU Delft, then validated and tested before deployment. The trained weights were embedded directly in the source tree so that onboard computation is limited to image preprocessing and forward inference.

In addition to the detector, this pull request wires the module into a dedicated custom airframe and a course configuration entry, and it connects the detector output to `custom_avoider`, which acts as an initial gate-oriented navigation concept. That navigation module is intentionally a first integration step and still requires further refinement, tuning, and validation before being considered a robust autonomous solution.

## Why this is valuable

- It adds a self-contained example of embedded CNN inference inside Paparazzi for a real MAVLab use case.
- It demonstrates how trained neural-network weights can be integrated directly into an onboard module without external runtime dependencies.
- It provides a practical bridge between perception and guidance by showing how `VISUAL_DETECTION` can drive a custom navigation behavior.
- It can be used as a standalone gate-detection demo even if the navigation part is later replaced or improved.

## How it works

1. The Bebop front camera and video pipeline provide image frames to the Paparazzi computer-vision stack.
2. `gate_cnn_detector` preprocesses the incoming frame, reduces it to the expected CNN input size, and evaluates the embedded network in pure C.
3. The detector produces a gate prediction and publishes it through the standard `VISUAL_DETECTION` ABI message.
4. `custom_avoider` subscribes to those messages and performs a simple gate-oriented behavior: search, align, fly forward, then briefly continue blind when the gate estimate is stable enough.

The key architectural point is that training is done offline, while onboard runtime is just deterministic preprocessing plus inference.

## Files changed in this contribution

Core XML and configuration changes:

- `conf/airframes/tudelft/custom_airframe.xml`
- `conf/modules/gate_cnn_detector.xml`
- `README.md`

Core source changes for the feature:

- `sw/airborne/modules/computer_vision/gate_cnn_detector/gate_cnn_detector.c`
- `sw/airborne/modules/computer_vision/gate_cnn_detector/gate_cnn_detector.h`
- `sw/airborne/modules/computer_vision/gate_cnn_detector/gate_cnn_weights.c`
- `sw/airborne/modules/computer_vision/gate_cnn_detector/gate_cnn_weights.h`
- `sw/airborne/modules/custom_avoider/custom_avoider.c`

If kept in the final PR, there is also cleanup of an older duplicate module definition:

- `conf/modules/custom_gate_detector.xml`

## Relevant XML files and their role

- `conf/userconf/tudelft/course_conf.xml`
  Registers the `bebop_custom_avoid` aircraft entry that selects the custom airframe from Paparazzi Center.

- `conf/airframes/tudelft/custom_airframe.xml`
  Main airframe wiring for the Bebop setup. It enables camera capture, the CNN gate detector, the initial navigation module, and the related video/telemetry pipeline.

- `conf/modules/gate_cnn_detector.xml`
  Module definition for the CNN detector. This file is important because it defines the module directory, headers, sources, and runtime hooks. The correct module directory must point to `computer_vision/gate_cnn_detector`.

- `conf/modules/custom_avoider.xml`
  Module definition for the navigation module. This module depends on `gate_cnn_detector` and consumes `VISUAL_DETECTION` messages.

- `conf/modules/custom_gate_detector.xml`
  Older duplicate or alternate module definition. If this file remains in the branch, it should be clearly treated as legacy or removed to avoid confusion with `gate_cnn_detector.xml`.

## Important source files and their role

- `sw/airborne/modules/computer_vision/gate_cnn_detector/gate_cnn_detector.c`
  Main prediction implementation. Handles frame preprocessing, forward inference, postprocessing, and ABI publication.

- `sw/airborne/modules/computer_vision/gate_cnn_detector/gate_cnn_detector.h`
  Public interface for the detector and the prediction structure used internally by the module.

- `sw/airborne/modules/computer_vision/gate_cnn_detector/gate_cnn_weights.c`
  Embedded trained weights for the gate detector network.

- `sw/airborne/modules/computer_vision/gate_cnn_detector/gate_cnn_weights.h`
  External declarations for the embedded weights arrays.

- `sw/airborne/modules/custom_avoider/custom_avoider.c`
  Initial navigation module that consumes the detector output. It demonstrates how the detector can be used in a closed loop, but it should be regarded as an initial concept rather than a finished controller.

## Integration and standalone demo usage

To integrate and run this feature inside Paparazzi:

1. Ensure the aircraft entry `bebop_custom_avoid` is available through `conf/userconf/tudelft/course_conf.xml`.
2. In Paparazzi Center, select the `bebop_custom_avoid` aircraft.
3. Use the airframe `conf/airframes/tudelft/custom_airframe.xml`.
4. Build the `ap` target for onboard use on the Bebop platform.

For a standalone gate-detection demo, the essential chain is:

- Bebop camera
- `video_capture`
- `gate_cnn_detector`
- telemetry or logging of `VISUAL_DETECTION`

`custom_avoider` is not required to demonstrate gate detection itself. It is included to show how the detector can already be connected to a first navigation behavior.

## Build notes

- `ap` is the main path for the onboard demo.
- The current `nps` target in `custom_airframe.xml` uses `fdm type="gazebo"`, so Gazebo development headers and libraries are required if `nps` is built.
- If Gazebo is not installed, the detector and integration can still be demonstrated on the `ap` target, or the `nps` FDM configuration can be changed separately.

## Known limitations

- The detector is the mature part of this contribution relative to the navigation layer.
- `custom_avoider` is intentionally an initial idea for making the detector usable in a closed-loop demo and needs further refinement, tuning, and testing.
- The navigation logic should not yet be presented as a final autonomous gate-navigation solution.

## Suggested reviewer focus

- Confirm the module integration path in `conf/modules/gate_cnn_detector.xml`.
- Review whether the airframe wiring in `conf/airframes/tudelft/custom_airframe.xml` is the right integration point for the Bebop demo.
- Verify the detector-to-navigation ABI contract through `VISUAL_DETECTION`.
- Check whether legacy duplicate module definitions should remain in the branch.