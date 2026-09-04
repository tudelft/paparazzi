# CATIA Camera Pipeline

 **CATIA in full the Camera Application Triggering Image Analysis** application runing on > a companion board which has a camera connected. As soon as CATIA receives a 'take photo now' command  from the Flightcontroller , camera obtains a JPEG image of current view, adds flight metadata like attitude and more to the EXIF data in the image  > and passes the finished image to SODA. for more image data extraction
.

**Doc Maintainers:** edit this `README.md`, `catia-flow.dot`, or
`desk-test-setup.svg`, then run `./update-documentation.sh`. The rendered PNGs,
flow SVG, and HTML page are generated files.

![CATIA capture and processing flow](catia-flow.png)

[Open the standalone HTML guide](index.html)

## Start Here: Local Simulation

This is the simplest way to verify the complete chain without camera hardware.
Run these commands from the Paparazzi repository root.

### 1. Build CATIA

```sh
make -C sw/airborne/modules/digital_cam/catia
```

### 2. Start the local camera service

```sh
sw/airborne/modules/digital_cam/catia/run_local_demo.sh
```

The script builds CATIA, creates the local serial bridge, and starts CATIA with
the bundled mock image. It also enables `--debug`. Leave this terminal running
while NPS is running.

The important startup line is:

```text
Started OK
```

CATIA and a Simulated Flight of Paparazzi aircraft may start in either order. The simulation UART retries while
`/tmp/catia-sim` is absent and reconnects automatically after CATIA is stopped
and started again. This allows CATIA sourcecode to be changed, then rebuilt, redeployed and restarted without even the need interrupting a running simulation.

### 3. Trigger a photo from Paparazzi

Start the aircraft simulation and trigger `DC_SHOOT`, for example through the
camera flight-plan block. Simulation sends the shot index, position, altitude
attitude, speed, and course to CATIA.

For every completed capture, CATIA prints debug info is enabled:

```text
Photo take 6
```

The corresponding local images somewhere like:

```text
sw/airborne/modules/digital_cam/catia/photos/m000006.jpg
```

### 4. Confirm the complete chain

A healthy local capture ends with messages similar to:

```text
Shooting: got image .../photos/m000006.jpg
Shooting: EXIF metadata added
Photo take 6
SODA: I looked at the image. Cool, right?
Shooting: soda return 0 ...
```

Stop CATIA with `Ctrl+C`. CATIA also stops the local `socat` bridge that it
created.

## Start Here: Physical Desk Test With MORA

Use this setup to test the deployed CATIA binary on for example a Raspberry Pi Zero 2 W
while Simualtion runs on a local PC. The USB-to-UART link carries the exact same MORA camera
messages used by the aircraft. When MORA (Magic Onboard Recognition Apparatus) is mentioned, it's the Computer Board with a camera.

![Local PC, USB-to-UART adapter, MORA, and Raspberry Pi camera desk-test setup](desk-test-setup.png)

### 1. Connect the desk hardware

1. Connect the Raspberry Pi camera to MORA with its CSI ribbon cable and validate it works
2. Connect a **3.3 V logic** USB-to-UART adapter to the laptop.
3. Cross the UART data lines: adapter TX to MORA RX, and adapter RX to MORA TX.
4. Connect adapter GND to MORA GND.
5. Connect the FTDI cable 5 V power output to MORA's 5 V input. The FTDI supply must
   provide enough current for the Raspberry Pi Zero 2 W and its camera. Do not
   connect the adapter's 3.3 V power output; 3.3 V refers only to UART signal
   levels in this setup.

On the local PC the adapter device is likely `/dev/ttyUSB0`. On MORA, CATIA uses the
`/dev/serial0` device. Both ends run at 115200 baud. The example `Easystar_3` simulated airframe
already configures these local PC-side settings. See the
`<module name="digital_cam_uart">` block of the airframe.

### 2. First run: real UART with a mock image

Start CATIA from the laptop and keep the SSH terminal open:

```sh
ssh -t air@theatre \
   'exec /home/air/digital_cam/catia --debug --mocktransform --test'
```

This first run exercises the physical UART, MORA framing, attitude transform,
EXIF metadata, and SODA while keeping the physical camera out of the test. Wait
for:

```text
CATIA:  serial device: /dev/serial0
Started OK
CATIA DEBUG:  waiting for MORA camera messages on /dev/serial0
```

Start `Easystar_3` NPS on the laptop and trigger `DC_SHOOT`. A successful shot
progresses through these diagnostics:

```text
received MORA frame start
accepted MORA message id 1
photo trigger received
Shooting: got image ...
Shooting: EXIF metadata added
Shooting: soda return 0 ...
```

Inspect the generated images from another laptop terminal:

```sh
ssh air@theatre 'ls -lh /home/air/digital_cam/photos/m*.jpg'
```

### 3. Second run: Raspberry Pi camera

Stop the mock-image run with `Ctrl+C`, then start CATIA with the physical camera:

```sh
ssh -t air@theatre \
   'exec /home/air/digital_cam/catia --aicam --debug'
```

Trigger another `DC_SHOOT`. Physical-camera images are written as
`/home/air/digital_cam/photos/aNNNNNN.jpg` before EXIF and SODA processing
completes.

### 4. Third run: Tiny 1-C LWIR camera

Connect the Tiny 1-C USB camera and start CATIA with the LWIR backend:

```sh
ssh -t air@theatre \
   'exec /home/air/digital_cam/catia --lwircam --debug'
```

Trigger `DC_SHOOT` again. CATIA starts the deployed `sample` executable in
one-shot mode, waits for a usable thermal frame, and writes a numbered JPEG as
`/home/air/digital_cam/photos/lNNNNNN.jpg`. CATIA then inserts the same flight
EXIF metadata used by the other camera backends and invokes SODA.

A successful capture includes these diagnostics:

```text
CATIA-N: requesting image from lwircam backend
selected camera index=0 vid=0bda pid=5840 name=USB Camera
LWIR CAPTURE: saved /home/air/digital_cam/photos/l00000N.jpg (256x192)
CATIA-N: Shooting: EXIF metadata added
CATIA-N: Shooting: soda return 0 ...
```

The deployment script installs a udev rule for USB device `0bda:5840`. The
vendor SDK needs read/write access to the raw `/dev/bus/usb/...` node in
addition to access to the V4L device. The rule assigns the raw USB node to the
`plugdev` group with mode `0660`.

Only CATIA should read the UART during these tests. Do not run `hexdump`,
`screen`, or another serial monitor at the same time because it will consume
bytes intended for CATIA. No manual `stty` command is needed; CATIA configures
`/dev/serial0` itself. Stop CATIA with `Ctrl+C` when the test is complete.

## Choose a Camera

CATIA separates the serial transport from the camera backend. `--local` creates
the simulator serial bridge; `--chdk`, `--aicam`, and `--lwircam` select
physical cameras.

| Goal | Command | Image source |
| --- | --- | --- |
| Complete local simulation | `catia --local` | Bundled mock JPEG |
| CHDK on the default serial port | `catia --chdk` | CHDK capture and download |
| Raspberry Pi camera | `catia --aicam` | `rpicam-still` |
| Tiny 1-C thermal camera | `catia --lwircam` | `sample --capture --output ...` |
| NPS transport with AI camera | `catia --local --aicam` | `rpicam-still` |
| NPS transport with CHDK | `catia --local --chdk` | CHDK capture and download |
| NPS transport with LWIR camera | `catia --local --lwircam` | Tiny 1-C one-shot capture |

When running `catia` directly from the repository root, use its full path:

```sh
sw/airborne/modules/digital_cam/catia/catia --aicam
```

For readability, later examples shorten this path to `catia`. Run those
examples from the CATIA directory, add that directory to `PATH`, or substitute
the full repository-relative path shown above.

Use a specific real serial endpoint with either physical camera:

```sh
sw/airborne/modules/digital_cam/catia/catia \
  --serial /dev/ttyUSB0 \
  --aicam
```

`--serial` and `--local` cannot be combined because `--local` owns its PTY
bridge.

### AI camera command

The AI-camera backend starts `rpicam-still` directly, without a shell:

```sh
rpicam-still \
   -o photos/a%06d.jpg \
  --width 4056 \
  --height 3040 \
  --hflip \
  --vflip
```

CATIA waits for the process to finish and rejects a missing, empty, or failed
capture.

### LWIR camera command

The LWIR backend starts one persistent `sample` process directly, without a
shell, during CATIA initialization:

```sh
/home/air/digital_cam/sample \
   --capture-server \
   --bare
```

## Test Any Camera Selection

Add `--test` to bypass camera hardware while retaining the selected backend in
the logs. This is useful before connecting CHDK or Raspberry Pi hardware.

```sh
catia --local --test
catia --chdk --test
catia --aicam --test
catia --lwircam --test
catia --local --chdk --test
catia --local --aicam --test
catia --local --lwircam --test
```

In all seven cases, CATIA uses a test JPEG and still performs the normal EXIF and
SODA stages. It does **not** start CHDK, `rpicam-still`, or the LWIR sample.

### Use a test-photo set

Create `testphotos` beside the built `catia` executable:

```text
sw/airborne/modules/digital_cam/catia/
|-- catia
`-- testphotos/
   |-- coast.jpg
   |-- field.jpg
   `-- village.jpg
```

Then run, for example:

```sh
sw/airborne/modules/digital_cam/catia/catia --aicam --test
```

Selection rules are deliberately simple:

1. Only readable files ending in `.jpg` are candidates; matching is
   case-insensitive.
2. With one candidate, CATIA always uses that image.
3. With several candidates, CATIA randomly selects one for each shot.
4. If the directory is absent or empty, CATIA uses the compiled default image.

To force one particular file and skip test-set selection:

```sh
catia --aicam --test --mock-image /absolute/path/example.jpg
```

## Simulate Aircraft Attitude

`--mocktransform` makes an image look as though it was captured at the
aircraft attitude carried in the MORA shot:

```sh
catia --local --aicam --test --mocktransform
```

The transform is disabled by default and requires parameter `--test`.

Near-horizon detection uses three configurable limits. Reaching any limit
activates the selected near-horizon behavior:

| C define | Make variable | Default | Meaning |
| --- | --- | --- | --- |
| `near_horizon_roll_deg` | `NEAR_HORIZON_ROLL_DEG` | `75.0` | Absolute roll limit |
| `near_horizon_pitch_deg` | `NEAR_HORIZON_PITCH_DEG` | `75.0` | Absolute pitch limit |
| `near_horizon_combined_tilt_deg` | `NEAR_HORIZON_COMBINED_TILT_DEG` | `70.0` | Optical-axis tilt derived from roll and pitch |

The default is:

```c
#define near_horizon_case_black 0
```

Alter it to `1` to skip terrain transformation and immediately generate a black
JPEG with the same dimensions. This is faster and useful when near-horizon
imagery should be treated as unavailable:

```sh
make -C sw/airborne/modules/digital_cam/catia \
   NEAR_HORIZON_CASE_BLACK=1
```

Customize thresholds at build time, for example:

```sh
make -C sw/airborne/modules/digital_cam/catia \
   NEAR_HORIZON_ROLL_DEG=70.0 \
   NEAR_HORIZON_PITCH_DEG=65.0 \
   NEAR_HORIZON_COMBINED_TILT_DEG=60.0
```

All thresholds are validated in the range `0..180` degrees.

CATIA also calculates and logs:

```text
blur_factor = ground_speed_m_s / 100
```

The blur factor is reserved for future use. No blur is currently applied.

## How the JPEG Is Saved

The answer depends on the selected source. CATIA does not use one universal
JPEG-saving function.

### Local and test images

`local_pipe_shoot()` copies the chosen source JPEG to `photos/m%06d.jpg`. It uses
bounded POSIX `open()`, `read()`, and `write()` loops and handles partial writes.
The bytes are already JPEG encoded; this stage does not decode or re-encode
them.

### Raspberry Pi AI camera

`ai_cam_pipe_shoot()` uses `posix_spawnp()` to execute `rpicam-still` with a
fixed argument list. **`rpicam-still` is the component that captures and JPEG
encodes the image.** The default result is `photos/a%06d.jpg`.

### Tiny 1-C LWIR camera

`lwir_cam_pipe_shoot()` uses `posix_spawn()` to execute the configured LWIR
sample with `--capture --output <filename>`. The sample owns USB acquisition,
startup-frame rejection, YUYV-to-RGB conversion, and initial JPEG encoding.
The default result is `photos/l%06d.jpg`.

### CHDK camera

`chdk_pipe_shoot()` instructs the CHDK camera to capture and download a JPEG.
CATIA moves the downloaded file to `photos/c%06d.jpg`.

### Optional transform and EXIF

After any backend returns a JPEG, the shared processing pipeline runs:

1. With `--mocktransform`, `image_mock_transform()` decodes the image, applies
   the attitude homography and borderless cover crop, then re-encodes it using
   the vendored IJG libjpeg implementation.
2. `image_exif_write()` builds metadata with bundled libexif and inserts the
   EXIF APP1 segment.
3. Both rewriting stages use a temporary file, `fsync()`, and atomic `rename()`;
   consumers never see a partially rewritten destination.
4. SODA starts only after the final JPEG and EXIF metadata are ready.

The EXIF record includes GPS coordinates, MSL and ground altitude, roll, pitch,
yaw, ground speed, course, shot index, and the original raw MORA fields.

## Performance and Target CPUs

The default build is tuned for both modern AMD64 Linux systems and ARM64 Linux
on the Raspberry Pi Zero 2 W without embedding CPU-specific instructions:

### Cross-compile for ARM64

Cross-compilation builds Raspberry Pi executables on an AMD64 development
computer. The resulting binaries cannot run on the development computer; copy
them to an ARM64 Linux target before running them.

On you local PC with e.g. Ubuntu Linux, install the AArch64 C and C++ cross-compilers once:

```sh
sudo apt update
sudo apt install gcc-aarch64-linux-gnu g++-aarch64-linux-gnu
```

From the Paparazzi repository root, clean any objects built for another CPU and
build CATIA with the cross-compilers:

```sh
make -C sw/airborne/modules/digital_cam/catia clean
make -C sw/airborne/modules/digital_cam/catia -j"$(nproc)" \
   CC=aarch64-linux-gnu-gcc \
   CXX=aarch64-linux-gnu-g++ \
   LWIR_ARCH=aarch64-gnu \
   LWIR_STATIC=1 \
   LWIR_VIDEO_DISPLAY=0
```

This builds CATIA, SODA, and the LWIR `sample`. The LWIR build selects
the bundled AArch64 SDK archives, links them statically for headless operation on MORA.

Confirm that all three executables target ARM64:

```sh
file sw/airborne/modules/digital_cam/catia/{catia,soda_local}
file sw/airborne/modules/digital_cam/catia/lwircam/sample
```

All lines should contain `ARM aarch64` and `statically linked`. The static
executables include their required C/C++ runtime code and bundled libraries,
but CATIA still needs its selected camera program, writable output directories,
and `soda_local` on the target.

CATIA embeds default file and directory paths at build time. If the Paparazzi
checkout has the same absolute path on both computers, the command above is
enough. Otherwise, set paths for the target computer while cross-compiling. For
example, to install the runtime files in `/opt/catia`:

```sh
make -C sw/airborne/modules/digital_cam/catia clean
make -C sw/airborne/modules/digital_cam/catia -j"$(nproc)" \
   CC=aarch64-linux-gnu-gcc \
   CXX=aarch64-linux-gnu-g++ \
   LWIR_ARCH=aarch64-gnu \
   LWIR_STATIC=1 \
   LWIR_VIDEO_DISPLAY=0 \
   CATIA_MOCK_IMAGE=/opt/catia/mock_image_01.jpg \
   CATIA_LOCAL_SODA=/opt/catia/soda_local \
   CATIA_LOCAL_PHOTO_DIR=/opt/catia/photos \
   CATIA_AI_CAM_PHOTO_DIR=/opt/catia/photos \
   CATIA_LWIR_CAM_PHOTO_DIR=/opt/catia/photos \
   CATIA_CHDK_PHOTO_DIR=/opt/catia/photos \
   CATIA_LWIR_CAM_COMMAND=/opt/catia/sample
```

#### Build and deploy with one command

`deploy_mora.sh` automates a clean ARM64 release build, creates the runtime directories
over SSH, and transfers CATIA, SODA, the LWIR sample, and both mock test images
with `rsync`. It also installs and reloads the Tiny 1-C udev rule, installs and
validates `catia.service`, enables it for every boot, starts it, and verifies
that it remains active. Its defaults match a MORA available as `itsme@itsmypie`
and install into `/home/itsme/digital_cam`:

```sh
sw/airborne/modules/digital_cam/catia/deploy_mora.sh
```

Before using the script, verify that public-key login works without a password
prompt:

```sh
ssh itsme@itsmypie true
```

The development computer needs `make`, `file`, `ssh`, `rsync`, and the AArch64
C/C++ compiler, `readelf`, and `strip` tools. The script stops before transfer
if a required command is missing, any executable is not AArch64, a binary is
not stripped, any debug section remains, or an unmanaged CATIA or LWIR server
is still running. This avoids replacing a live executable or competing for the
Tiny 1-C.

Supply a different SSH destination as the first argument. The systemd unit uses
the fixed canonical installation directory `/home/air/digital_cam`, so the
optional second argument must have that value:

```sh
sw/airborne/modules/digital_cam/catia/deploy_mora.sh \
   itsme@itsmypie /home/itsm/digital_cam
```

Set `BUILD_JOBS` to limit parallel compilation when needed:

```sh
BUILD_JOBS=4 sw/airborne/modules/digital_cam/catia/deploy_mora.sh
```

The deployed CATIA binary uses `/dev/serial0` by default and embeds paths below
the selected installation directory. The script creates the shared `photos`
directory. The system-level `catia.service` runs as the unprivileged `air`
user with `dialout` and `plugdev` access. It uses absolute paths, starts at
`multi-user.target`, and retries every three seconds without a start limit when
the serial device, USB camera, or another startup dependency is temporarily
unavailable. systemd marks it active only after CATIA has initialized both the
persistent LWIR capture server and `/dev/serial0`.

Inspect the service and follow its journal with:

```sh
ssh air@theatre 'systemctl status catia.service'
ssh air@theatre 'journalctl -fu catia.service'
```

CATIA and standalone `sample` runs require exclusive Tiny 1-C ownership. Stop
the service for maintenance or a manual test, and restart it afterward:

```sh
ssh air@theatre 'sudo systemctl stop catia.service'
ssh air@theatre \
   '/home/air/digital_cam/catia --debug --mocktransform --test'
ssh air@theatre 'sudo systemctl restart catia.service'
```

Disable automatic boot startup only when intentionally taking CATIA out of
service:

```sh
ssh air@theatre 'sudo systemctl disable --now catia.service'
ssh air@theatre 'sudo systemctl enable --now catia.service'
```

Test the deployed LWIR processing path without a USB camera using the bundled
mock image:

```sh
ssh air@theatre \
  'cd /home/air/digital_cam && ./sample \
   --mock-image mock_lwir_01.jpg \
   --output mock_lwir_processed.jpg'
```

This decodes the JPEG, converts its luminance to synthetic Y14 samples, runs the
vendor enhancement and RGB conversion functions, and writes a processed JPEG.
Because a JPEG has no radiometric temperature plane, the mock test reports a
constant synthetic `25.0 C` value. That value tests data plumbing only; it is
not a measured temperature. Running `./sample` without mock options retains the
vendor USB-camera workflow for the later hardware test.

Run a standalone real-camera one-shot test with:

```sh
ssh air@theatre \
   'cd /home/air/digital_cam && ./sample \
    --capture --output lwir_standalone.jpg'
```

To explicitly request the unfiltered image plane, add `--bare`:

```sh
ssh air@theatre \
   'cd /home/air/digital_cam && ./sample \
    --capture --bare --output lwir_standalone_bare.jpg'
```

For the complete CATIA, EXIF, and SODA path, use:

```sh
ssh -t air@theatre \
   'exec /home/air/digital_cam/catia --lwircam --debug'
```

After cross-compiling, restore runnable AMD64 binaries on the development
computer with a clean native build:

```sh
make -C sw/airborne/modules/digital_cam/catia clean
make -C sw/airborne/modules/digital_cam/catia -j"$(nproc)"
```

### Runtime performance

- CATIA, SODA, bundled IJG libjpeg, and bundled libexif compile with `-O2`;
- CATIA keeps the LWIR stream warm after a one-time startup qualification;
- steady-state LWIR capture takes about `0.35-0.40 s`, supporting one photo per second;
- accepted camera triggers queue instead of being discarded while a save finishes;
- the attitude warp advances its projective coordinates across each scanline;
- bilinear coordinates and weights are calculated once for all RGB channels;
- CATIA drains serial input in chunks instead of one byte per millisecond;
- `poll()` sleeps until serial or UDP work arrives, reducing idle CPU use;
- SODA starts directly with `posix_spawnp()` instead of through a shell;
- processing workers are detached, bounded, and drained before shutdown;
- CHDK reads have monotonic whole-operation deadlines and checked I/O;
- SODA is terminated and reaped during shutdown, with a bounded grace period;
- project sources compile with warnings treated as errors, while bundled
   third-party objects are isolated in the build.

On the development AMD64 system, ten `1200x899` decode-transform-encode cycles
improved from approximately `1.10 s` to `0.56 s` for a normal attitude and from
`1.41 s` to `0.75 s` for synthesized terrain: about `49%` and `47%` less wall
time respectively. The event-driven idle loop used zero measured CPU ticks in a
three-second local sample and dispatched a test trigger in under one
millisecond. These are local reference measurements, not Raspberry Pi
guarantees.

Override the portable optimization level when profiling or debugging:

```sh
make -C sw/airborne/modules/digital_cam/catia clean all OPTFLAGS=-O0
```

The repository contains an OpenCV-embedded libjpeg-turbo tree. It is not linked
directly because that component requires CMake-generated headers plus different
NASM/NEON SIMD source selection for AMD64 and ARM64. The current bundled IJG
codec remains self-contained and cross-compiles reliably. A future switch to
libjpeg-turbo should build it as a proper separate library rather than compile
its source files ad hoc.

## Output Locations

| Source | Default output | Filename example |
| --- | --- | --- |
| Local or `--test` | `catia/photos/` | `m000006.jpg` |
| AI camera | `catia/photos/` | `a000006.jpg` |
| LWIR camera | `catia/photos/` | `l000006.jpg` |
| CHDK | `catia/photos/` | `c000006.jpg` |

Change paths or the camera command at build time:

```sh
make -C sw/airborne/modules/digital_cam/catia \
   CATIA_LOCAL_PHOTO_DIR=/data/photos \
   CATIA_AI_CAM_PHOTO_DIR=/data/photos \
   CATIA_AI_CAM_COMMAND=/usr/bin/rpicam-still \
    CATIA_LWIR_CAM_PHOTO_DIR=/data/photos \
    CATIA_CHDK_PHOTO_DIR=/data/photos \
   CATIA_LWIR_CAM_COMMAND=/opt/catia/sample
```

## Command Reference

| Option | Meaning |
| --- | --- |
| `--local` | Create `/tmp/catia-sim` and `/tmp/catia-app` for local simulation |
| `--serial DEVICE` | Use a specific real serial endpoint |
| `--chdk` | Select the CHDK camera backend |
| `--aicam` | Select the Raspberry Pi camera backend |
| `--lwircam` | Select the Tiny 1-C LWIR camera backend |
| `--test` | Replace physical capture with a test JPEG |
| `--mock-image FILE` | Use one explicit JPEG instead of `testphotos` |
| `--mocktransform` | Apply test-only roll, pitch, and yaw transformation |
| `--debug` | Show serial traffic, MORA frame, trigger, and capture diagnostics |
| `--help` | Print the built-in command help |

## Troubleshooting

### `Socket: bind: Address already in use`

Another CATIA process is already using the socket. Stop the older process
before starting another instance.

```sh
pgrep -af catia
```

### `local mode is already running`

Only one process may own the local PTY bridge. Stop the existing local CATIA
process with `Ctrl+C`, then start the demo again.

### No image appears

`run_local_demo.sh` enables `--debug` automatically. Check the log in this
order:

1. `Started OK` confirms CATIA initialized.
2. `CATIA DEBUG: waiting for MORA data: 0 bytes, 0 valid frames, 0 rejected
   frames` means CATIA is healthy but has not received camera traffic yet. A
   reconnect-capable NPS instance will attach automatically within a short
   interval; the next camera trigger should then appear.
3. `received MORA frame start` confirms serial bytes reached CATIA.
4. `rejected MORA frame` indicates framing or checksum failure.
5. `accepted MORA message id 1` and `photo trigger received` confirm a valid
   shot command.
6. `SHOT NR` shows the decoded flight and attitude data.
7. `Shooting: got image ...` confirms capture or test-copy completion.
8. `EXIF metadata added` confirms metadata insertion.
9. `soda return 0` confirms analysis completed successfully.

Enable the same diagnostics for a non-local command by adding `--debug`:

```sh
catia --aicam --debug
```

For LWIR capture diagnostics, use `catia --lwircam --debug`. If device
selection succeeds but `uvc_camera_open` reports error `-3`, inspect the raw USB
node permissions. It must be writable by `plugdev`. The bus/device numbers can
change after reconnecting; use `lsusb -d 0bda:5840` to find the current node.
Re-run `deploy_mora.sh` or reload `/etc/udev/rules.d/99-tiny1c.rules` if its
group or mode is wrong.

### Restart CATIA without restarting NPS

The simulation UART checks for `/tmp/catia-sim` every 200 ms. When CATIA stops,
NPS detects the closed PTY and keeps flying. Use this development loop:

```sh
# Stop CATIA with Ctrl+C, then rebuild and restart it:
make -C sw/airborne/modules/digital_cam/catia
sw/airborne/modules/digital_cam/catia/run_local_demo.sh
```

NPS remains running throughout. Camera commands issued while CATIA is stopped
cannot be recovered, but later commands are delivered after reconnection.

### Build shows libexif or libjpeg warnings

The bundled third-party sources may emit compiler warnings. A successful build
still creates the `catia` and `soda_local` executables. Errors reported against
CATIA-owned sources should be investigated.

## Regenerate the Diagram

The Markdown guide and Graphviz DOT file are the canonical documentation
sources. After changing CATIA behavior or commands, update both as needed and
run this from the `documentation` directory:

```sh
./update-documentation.sh
```

The script validates its dependencies, renders `catia-flow.png` and
`catia-flow.svg` with Graphviz, composites `thelaptopscreen.png` into the
Inkscape-rendered `desk-test-setup.png`, and rebuilds `index.html` from this
Markdown file. It then checks that every generated file is nonempty.

Required tools:

- Graphviz `dot`;
- Inkscape;
- ImageMagick `magick`;
- Python 3;
- Python package `Markdown` (`python3 -m pip install Markdown`).

Do not edit `index.html`, `catia-flow.png`, `catia-flow.svg`, or
`desk-test-setup.png` directly; the next documentation update replaces them.
