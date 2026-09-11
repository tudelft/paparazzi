
---

### Optimized Agentic AI Prompt

```markdown
# TASK INSTRUCTION: Create Paparazzi UAV Airframe Auto-Tuning Script (`log_2_tuned_airframe.py`)

## 1. System Context & Objective
You are an expert UAV flight control systems engineer and Python developer specializing in the Paparazzi UAV open-source framework. Your objective is to write a production-ready, highly optimized Python 3 script named `log_2_tuned_airframe.py`. 

This script parses Paparazzi telemetry flight logs to analyze aerodynamic behavior and automatically generates an updated, optimized airframe XML file.

---

## 2. Environment & File Paths
* **Script Location to Generate:** `~/paparazzi/sw/tools/autotune/log_2_tuned_airframe.py`
* **Output Airframe Directory:** `~/paparazzi/conf/airframes/OPENUAS/`
* **Log File Directory:** `~/paparazzi/var/logs/`
* **Target Log Files for Initial Calibration:**
  * Log config/telemetry: `26_09_05__18_42_28.log`
  * Telemetry dataset: `26_09_05__18_42_28.data`
* **Target Aircraft ID (AC_ID):** `129`

> **Critical Constraint:** All file system operations must explicitly handle symbolic links (resolve real paths using `pathlib.Path.resolve()`).

---

## 3. Iteration 1 Objective & Flight Dynamics Logic
The script must focus on solving **turn performance and trim offset issues** observed during stabilized manual flight:

1. **Neutral Pitch Offset ($\Theta$ / Theta):**
   * Parse pitch/attitude telemetry during straight-and-level stabilized flight.
   * Calculate the average pitch trim error and determine the correct neutral pitch parameter adjustment for the airframe configuration.

2. **Rudder-vons / Yaw Mixing & Turn Radius Optimization:**
   * Analysis indicates the aircraft currently exhibits an excessively wide turn radius due to insufficient yaw authority/coordination. Manual curves required high YAW actuator inputs.
   * Parse actuator commands and roll/yaw dynamics during manual curves.
   * Adjust or inject elevon-to-rudder (ruddervons) mixing or roll-yaw coordination gains in the XML configuration to enable a stable autonomous turn radius down to a **minimum of 30 meters**.

---

## 4. Execution Workflow & Technical Requirements

### A. Input Data Parsing
* Read `.log` (contains embedded airframe configuration, flight plan, telemetry definitions).
* Read `.data` (contains time-stamped message payloads for `AC_ID 129`).
* Efficiently process message streams (use `pandas`, `numpy`, or `multiprocessing` to ensure rapid log parsing).

### B. Airframe File Versioning & Output
* Identify the base airframe name (extracted from `.log` or parameter input).
* Check `~/paparazzi/conf/airframes/OPENUAS/` for existing optimization files.
* Save the newly tuned file using the mandatory naming scheme:  
  `<airframe_name>_optim_$$$.xml`  
  *(Where `$$$` is an auto-incrementing 3-digit version number, e.g., `_optim_001.xml`, `_optim_002.xml`).*

### C. Automated Compilation & Verification
* After writing the optimized XML, automatically trigger Paparazzi's build tools (e.g., `make AIRCRAFT=<target> build`) to verify that the generated XML parses and compiles without schema or syntax errors.
* Raise an explicit exception and log detailed diagnostics if the generated XML fails verification.

### D. Human-Readable Audit Report
* Every run must produce a clear, human-readable summary terminal output (and log file) detailing:
  1. Key telemetry metrics observed (mean pitch offset, yaw deflection during turns).
  2. Specific XML tags/parameters modified and their before/after values.
  3. Rationale for each parameter shift in plain engineering terms.
  4. Path to generated XML and build verification status (PASS/FAIL).

---

## 5. Code Quality Requirements
* **Language:** Clean, modular Python 3.8+ code using standard libraries + vectorization libraries (`numpy`/`pandas`) where applicable.
* **CLI Arguments:** Include `argparse` support allowing manual overrides for `--log-dir`, `--airframe-out`, `--ac-id`, and target turn radius.
* **Error Handling:** Robust handling for missing log files, corrupted records, broken symlinks, and XML schema malformations.

---

## 6. Deliverable Checklist
1. Generate `~/paparazzi/sw/tools/autotune/log_2_tuned_airframe.py`.
2. Ensure the script is executable (`chmod +x`).
3. Run the script against `26_09_05__18_42_28` logs for `AC_ID 129`.
4. Verify the output XML is correctly placed in `~/paparazzi/conf/airframes/OPENUAS/` and compiles cleanly.

```



----


Extend

For the IMAV2026 competition (TODO add rules section) we must perform a very precise landing on a TD stands for Touch down spot, we can use WP "TD". 
This percision landing must performed be fully autonomous for rhe maximum amount of points. 

Since we havbe lots of sensors and control options to make this a reality. 
We do have: 
* ublox M10 3 constelations 13Hz GNSS sensor
* Rangefinder sensor to deduct the groud distance in last few meters. 
* 310 barometric sensor and IMU fusion option
* we alreaydy have GLS routines in current sourcode, 
* an airspeed sensor good from 0 m/s to ~49m/s, 
* There is an option create and to use crow braking (Ailerons up and ruddervon elevstion a pit down, with live glideslope steering to land at the exact TD spot. 
* We have an enormous flexible fligtplan option where many variables of the autopilot can ve set
* We can call functions from withing the flightplan whenever and whatever
* We will have a well tuned airframe due to the "autotune" script create before.
* We have a rugit but small airfraam so a not all to steep angle at tough would likly not damage the aircraft.

