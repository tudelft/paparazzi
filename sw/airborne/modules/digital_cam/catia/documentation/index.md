# CATIA & Companion Systems Documentation Hub

Welcome to the documentation portal for **CATIA** (Camera Application
Triggering Image Analysis) and associated onboard vision, acoustic, and thermal
subsystems for Paparazzi UAV.

[Camera Pipeline](catia_camera_pipeline.html) |
[AI Camera](raspberry_pi_ai_camera.html) |
[LWIR Calibration](lwir-calibration.html) |
[EARcam Guide](earcam-loudest-spot-explained.html) |
[EARcam Data Flow](earcam-dataflow.html) |
[Mission 2 Plan](mission2-score-first.html) |
[Precision Landing](precision-landing-flight-test.html)

---

## Documentation Guides

Explore the detailed technical guides for each subsystem:

### Core Architecture & Deployment

- **[CATIA Camera Pipeline](catia_camera_pipeline.html)**
  The comprehensive primary manual. Details the end-to-end capture pipeline,
  serial communication protocol, trigger dispatching, EXIF metadata geotagging,
  SODA image analysis, and automated one-command deployment to MORA.
  - [Automated Deployment to MORA](catia_camera_pipeline.html#deploy-to-mora)
  - [Physical Desk Test Setup](catia_camera_pipeline.html#start-here-physical-desk-test-with-mora)
  - [Local Simulation & Development](catia_camera_pipeline.html#start-here-local-simulation)
  - [Camera Selection & 8-Bit Masks](catia_camera_pipeline.html#choose-a-camera)
  - [Pose Streaming & Clock Handshake](catia_camera_pipeline.html#pose-and-image-timing-evidence)

### Optical & Thermal Imaging

- **[Raspberry Pi AI Camera (Sony IMX500)](raspberry_pi_ai_camera.html)**
  Hardware connections, software installation (`imx500-all`), `rpicam-*` tools,
  on-sensor neural network inference demos (MobileNet SSD, PoseNet), and
  integration with CATIA's `rpicam-still` backend on MORA.
  - [CSI Ribbon Wiring & Orientation](raspberry_pi_ai_camera.html#hardware-and-connections)
  - [Testing Commands & Demos](raspberry_pi_ai_camera.html#run-imx500-inference-demos)
  - [Camera Parameter Reference](raspberry_pi_ai_camera.html#parameters-and-why-they-matter)
  - [OS Image Backup & Restore Tools](raspberry_pi_ai_camera.html#backup)

- **[LWIR Thermal Camera Calibration](lwir-calibration.html)**
  Step-by-step workshop procedure for calibrating the Tiny 1-C thermal camera.
  Covers thermal target construction with foil/matte tape, multi-pose lens
  distortion fitting with OpenCV, body mounting alignment, and verification.
  - [Thermal Target Construction](lwir-calibration.html#3-make-a-target-the-camera-can-see)
  - [Multi-View Lens Calibration](lwir-calibration.html#6-fit-the-lens-on-the-laptop)
  - [Aircraft Mounting Alignment](lwir-calibration.html#7-measure-the-mounting-without-guessing)
  - [Verification & Error Checks](lwir-calibration.html#8-check-distances-before-checking-gps)

### Acoustic Search & Localization

- **[EARcam Explained: Acoustic Localization](earcam-loudest-spot-explained.html)**
  Conceptual and algorithmic guide on finding the loudest spot on the ground
  using a single USB microphone on a moving fixed-wing aircraft. Explains
  motionSCOUT alarm detection, inverse-square loudness modeling, and flight search
  patterns (lawnmower survey + star refinement).
  - [Why One Simple USB Microphone](earcam-loudest-spot-explained.html#why-one-simple-usb-microphone-was-chosen)
  - [Acoustic Detection Physics](earcam-loudest-spot-explained.html#how-sound-behaves-in-plain-terms)
  - [Loudest-Spot Solver Algorithm](earcam-loudest-spot-explained.html#turning-readings-into-one-position)
  - [Autonomous Flight Search Patterns](earcam-loudest-spot-explained.html#the-flight-pattern)

- **[EARcam Position & Sound Data Flow](earcam-dataflow.html)**
  Deep dive into the real-time data flow between the flight controller and
  MORA. Covers UART framing, 50 ms loudness window sampling, geotagged ring
  buffering, and `< 1 ms` solver execution.
  - [Synchronization Overview](earcam-dataflow.html#synchronization-overview)
  - [Data Flow Pipeline](earcam-dataflow.html#data-flow-pipeline)
  - [Sample Field Origins & Logging](earcam-dataflow.html#sample-field-origins)
  - [Ground Replay Tools](earcam-dataflow.html#ground-tools-and-replay)

### Flight Operations & Scoring

- **[Mission 2: Maximize Points, Keep The Airframe](mission2-score-first.html)**
  Competition flight strategy and software-first accuracy assessment for IMAV
  2026 Mission 2. Focuses on safe fixed-wing operations, thermal candidate
  scoring, WGS84 ground-truth verification, and offline assessment scripts.
  - [Quick 'n Dirty Checklist](mission2-score-first.html#quick-n-dirty-checklist)
  - [Mission 2 Scoring Breakdown](mission2-score-first.html#what-earns-points)
  - [Hotspot Verification Protocol](mission2-score-first.html#first-accuracy-baseline)
  - [Offline Assessment Tool (`mission2_assess.py`)](mission2-score-first.html#offline-assessment-tool)

- **[Adam and EasyStar 3 Precision Landing Flight Test](precision-landing-flight-test.html)**
  Staged verification and aircraft-specific tuning campaign for autonomous
  first contact inside the IMAV2026 fixed-wing 20 x 3 m precision box. Covers
  simulated rangefinder, crow characterization, crosswind prediction,
  touchdown attitude, abort/retry behavior, and competition readiness evidence.
  - [System and Controller](precision-landing-flight-test.html#system-under-test)
  - [Detailed Test Phases](precision-landing-flight-test.html#phase-0-software-and-geometry-verification)
  - [Parameter Tuning](precision-landing-flight-test.html#phase-9-parameter-tuning)
  - [Competition Readiness](precision-landing-flight-test.html#competition-readiness-gate)

---

## System Architecture

```text
 +-------------------------------------------------------+
 |               PAPARAZZI FLIGHT CONTROLLER             |
 |  - Navigation & Guidance (ETECS / Flight Plan)        |
 |  - Digital Cam UART Module (dc_periodic / RC Trigger) |
 +---------------------------+---------------------------+
                             | UART (115200 baud)
                             v
 +-------------------------------------------------------+
 |                  MORA COMPANION BOARD                 |
 |                                                       |
 |  +-------------------------------------------------+  |
 |  |                     CATIA                       |  |
 |  |  - Protocol Decoder & Trigger Dispatcher        |  |
 |  |  - EXIF Geotagging & Flight Pose Insertion      |  |
 |  |  - Acoustic Sample Ring Buffer & Solver         |  |
 |  +-------+----------------+---------------+--------+  |
 |          |                |               |           |
 |          v                v               v           |
 |    +-----------+    +-----------+   +-----------+     |
 |    |   AIcam   |    |  LWIRcam  |   |  EARcam   |     |
 |    | (IMX500)  |    | (Tiny1-C) |   |  (ALSA)   |     |
 |    +-----+-----+    +-----+-----+   +-----+-----+     |
 |          |                |               |           |
 |          +----------------+---------------+           |
 |                           v                           |
 |  +-------------------------------------------------+  |
 |  |             SODA (Image Analytics)              |  |
 |  |  - Photo Output:  ~/Pictures/                   |  |
 |  |  - Logs / Debug:  ~/usher_debug_data/           |  |
 |  +-------------------------------------------------+  |
 +-------------------------------------------------------+
```

---

## Quick Reference: Common Operations

| Operation | Command | Reference |
| --- | --- | --- |
| **Deploy to MORA** | `BUILD_JOBS=32 sw/airborne/modules/digital_cam/catia/deploy_mora.sh air@theatre` | [Deployment Guide](catia_camera_pipeline.html#deploy-to-mora) |
| **Check MORA Service** | `ssh air@theatre 'systemctl status catia.service'` | [Service Status](catia_camera_pipeline.html#5-verify-the-installed-release) |
| **Live Logs on MORA** | `ssh air@theatre 'journalctl -fu catia.service'` | [Live Logs](catia_camera_pipeline.html#5-verify-the-installed-release) |
| **Local Demo (No Hardware)** | `sw/airborne/modules/digital_cam/catia/run_local_demo.sh` | [Local Simulation](catia_camera_pipeline.html#start-here-local-simulation) |
| **Build Full Stack** | `make -C sw/airborne/modules/digital_cam/catia -j32 all` | [Build Instructions](catia_camera_pipeline.html#1-build-catia) |
| **Run Test Suite** | `bash sw/airborne/modules/digital_cam/catia/tests/build_layout_test.sh` | [Validation Tests](catia_camera_pipeline.html#3-build-and-validate-the-current-sources) |
| **Rebuild Documentation** | `make -C sw/airborne/modules/digital_cam/catia documentation` | [Docs Generator](catia_camera_pipeline.html#regenerate-the-diagram) |
