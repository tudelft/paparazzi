# Raspberry Pi AI Camera

Setup, camera commands, and MORA integration for the Sony IMX500-based
Raspberry Pi AI Camera.

[Documentation Hub](index.html) | [Camera Pipeline](catia_camera_pipeline.html) |
[AI Camera](raspberry_pi_ai_camera.html) | [LWIR Calibration](lwir-calibration.html) |
[EARcam Guide](earcam-loudest-spot-explained.html) | [EARcam Data Flow](earcam-dataflow.html) |
[Mission 2 Plan](mission2-score-first.html)

## Hardware and Connections

The AI Camera uses a 12.3-megapixel Sony IMX500 sensor with on-sensor inference,
a CSI connection, and a quoted 78.3-degree field of view. It is not the USB
Tiny 1-C thermal camera, and it is not an external AI accelerator board.

Power the Raspberry Pi off before attaching or removing the ribbon cable.
Use the correct camera cable for the board's connector size. Seat it squarely
and close the connector latch without forcing it. Do not assume similarly
shaped CSI and DSI sockets have the same purpose on every Pi model.

| Board | Connection guidance |
| --- | --- |
| Raspberry Pi 5 | Either supported camera/display connector can be used for a camera. Use its matching cable. |
| Raspberry Pi Zero family | Use the smaller camera connector and a Zero-compatible ribbon cable. |
| Earlier full-size Raspberry Pi boards | Use the dedicated CSI camera connector. |
| Compute Module | Follow the carrier-board camera connector and routing documentation. |

## Install the Camera Software

Run these commands **on the MORA**, not on the development PC.
Use a current supported Raspberry Pi OS image and check the installed version:

```sh
cat /etc/os-release
```

OK install The AI Camera tools and some optional extra like FFMPeg

```sh
sudo apt update
sudo apt install -y imx500-all 
sudo apt install -y ffmpeg gstreamer1.0-x gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly imagemagick libimage-exiftool-perl
sudo reboot
```

Do NOT perform a "upgrade" "full-upgrade" or "rpi-update" since this will make your camera NOT WORK anymore!
It is very well that this is fixed by some kind soul, but at moment of writing 20260912 it is broken.

Better still even lock you current working kernel, Find the kernelname to lock via the dpkg method and use uname -a to see which one is currently in use: in this case linux-image-6.18.34+rpt-rpi-v

```sh
dpkg -l 'linux-image-*'
uname -a
sudo apt-mark hold linux-image-6.18.34+rpt-rpi-v8
```

Review the package-manager changes before confirming. upgrade is a
board-maintenance operation, not part of CATIA's binary deployment.
Reconnect after reboot. If `imx500-all` is unavailable, check the OS release and its
Raspberry Pi package repositories rather than installing unrelated drivers.

Modern Raspberry Pi OS uses the `rpicam-*` application names. The source also
contains older `libcamera-*` examples for Bullseye; this guide uses the modern
names. Installing the camera packages does not enable IMX500 inference in CATIA.

## Verify Detection and Capture

Stop other programs using the CSI camera before testing. On an installed MORA,
follow the [CATIA maintenance procedure](catia_camera_pipeline.html#deploy-to-mora) to stop the
service first. Do not run two camera clients concurrently.

Thus login via

```sh
ssh air@theatre
```

Then in the shell test if your RPI camera hardware is recognized

```sh
rpicam-hello --version
rpicam-hello --list-cameras
```

Look for the IMX500 in the detected-camera list. Use the reported camera index
and supported modes, not the IMX219 or IMX477 example values in generic guides.

For an SSH-only bench test on ORA, capture a JPEG without opening a preview via:

```sh
rpicam-still --nopreview --timeout 2000 --output ai-camera-test.jpg
ls -lh ai-camera-test.jpg
```

Copy the file to your local PC with rsync via this line on your LOCAL PC, not the MORA

```sh
 rsync -av --ignore-existing --info=progress2  air@theatre:/home/air/i-camera-test.jpg .
 
 
Inspect the actual image as well as the process exit status. A nonempty file
alone does not prove correct focus, exposure, or mounting orientation.
For multiple cameras, select an index explicitly, for example:

```sh
rpicam-hello --camera 0 -t 0
```

## Run IMX500 Inference Demos

These are standalone Raspberry Pi camera demonstrations from the source, not
CATIA flight-controller commands. Confirm the referenced JSON files exist in
the installed camera assets. Preview examples expect a usable display session;
add `--nopreview` for headless video recording.

### Object Detection Preview

```sh
rpicam-hello -t 0s \
  --post-process-file /usr/share/rpi-camera-assets/imx500_mobilenet_ssd.json \
  --viewfinder-width 1920 --viewfinder-height 1080 --framerate 30
```

### Record Object Detection Video

```sh
rpicam-vid -t 10s -o output.264 \
  --post-process-file /usr/share/rpi-camera-assets/imx500_mobilenet_ssd.json \
  --width 1920 --height 1080 --framerate 30
```

### Pose Detection Preview

```sh
rpicam-hello -t 0s \
  --post-process-file /usr/share/rpi-camera-assets/imx500_posenet.json \
  --viewfinder-width 1920 --viewfinder-height 1080 --framerate 30
```

The JSON selects the model/post-processing pipeline. These example resolutions
and frame rates are requests, not measured performance guarantees for a Pi Zero
2 W. Check actual throughput and memory use on the intended board.

## Camera Application Reference

| Application | Purpose |
| --- | --- |
| `rpicam-hello` | Check camera operation and view a preview. |
| `rpicam-jpeg` | Capture a JPEG with a small command-line interface. |
| `rpicam-still` | Still capture with more encoding and raw-image options; used by CATIA AIcam. |
| `rpicam-vid` | Record or stream encoded video. |
| `rpicam-raw` | Record sensor Bayer frames without a normal image container. |
| `rpicam-detect` | Optional host-side TensorFlow Lite detection application; distinct from IMX500 inference demos. |

Use each installed program's `--help` to check available options. Features and
encoders depend on the installed version and board. Do not assume optional
`rpicam-detect` is included in the default OS installation.

### Still Images and Exposure

```sh
rpicam-jpeg -o test.jpg -t 2000 --width 640 --height 480
rpicam-jpeg -o manual-exposure.jpg --shutter 20000 --gain 1.5
rpicam-jpeg -o darker.jpg --ev -0.5
rpicam-still -e png -o test.png
rpicam-still --raw --output test.jpg
```

`--shutter` is in microseconds: 20000 means 20 ms. Exposure must fit the selected
sensor mode and frame duration. Gain can increase noise; higher brightness is
not evidence of improved image quality. `--ev` changes the automatic exposure
target rather than specifying a fixed shutter time.

The encoding option selects the still-image format, not merely the filename
extension. `--raw` requests a Bayer DNG alongside the processed image. Raw RGB
or YUV output and `rpicam-raw` streams are different: they need external format
and dimension information and are not automatically DNG files.

### Video and Network Streaming

```sh
rpicam-vid -t 10s -o test.h264
rpicam-vid -t 10000 --codec mjpeg -o test.mjpeg
rpicam-raw -t 2000 -o test.raw
```

An H.264 elementary stream is not an MP4 container. Container and libav support
vary with the installed build. Raw capture can consume substantial storage and
write bandwidth; check free space before running it.

The source also describes UDP, TCP, and RTSP streaming. A TCP example for a
trusted bench network is:

```sh
rpicam-vid --nopreview -t 0 --inline --listen -o tcp://0.0.0.0:8888
```

On the viewing PC, replace `MORA_IP` with the board's address:

```sh
ffplay tcp://MORA_IP:8888 -fflags nobuffer -flags low_delay -framedrop
```

This listener has no authentication or encryption. Do not expose it to an
untrusted network. The source's high-frame-rate and overclocking examples are
not IMX500/MORA performance recommendations; leave board clocks unchanged for
initial validation.

## Parameters and Why They Matter

| Option | Use and benefit |
| --- | --- |
| `--help`, `--version` | Establish which syntax and software version you are testing. |
| `--list-cameras`, `--camera INDEX` | Find sensors and select the intended one. |
| `--timeout`, `-t` | Control preview settling or recording duration; zero runs continuously where supported. |
| `--nopreview` | Avoid a display requirement for SSH/headless operation. |
| `--preview`, `--fullscreen`, `--qt-preview` | Choose preview layout/backend; these do not set captured-image resolution. |
| `--width`, `--height` | Set still/video output dimensions. |
| `--viewfinder-width`, `--viewfinder-height` | Set preview-stream dimensions separately. |
| `--mode`, `--viewfinder-mode` | Request sensor readout modes; inspect `--list-cameras` first. |
| `--lores-width`, `--lores-height` | Request a smaller stream for supported analysis pipelines. |
| `--post-process-file` | Select a processing pipeline such as the IMX500 demos. |
| `--shutter`, `--gain`, `--ev` | Trade exposure time, gain, noise, and motion blur. |
| `--metering`, `--exposure` | Adjust automatic exposure's measurement/profile. |
| `--awb`, `--awbgains` | Choose automatic white balance or fixed red/blue gains. |
| `--brightness`, `--contrast`, `--saturation`, `--sharpness` | Adjust appearance; excessive processing can obscure useful detail. |
| `--denoise` | Trade noise reduction against detail and processing throughput. |
| `--tuning-file` | Override sensor/module tuning; do not reuse another sensor's tuning blindly. |
| `--hflip`, `--vflip`, `--rotation` | Change image orientation; keep geometric calibration consistent. |
| `--roi` | Crop using normalized `x,y,width,height`, for example `0.25,0.25,0.5,0.5`. |
| `--hdr` | Request HDR only where the sensor/platform supports it. |
| `--info-text`, `--verbose` | Display capture metadata or detailed diagnostics. |
| `--config` | Load camera-application options from a file for repeatable standalone tests. |

The source's autofocus controls apply only to cameras exposing autofocus,
such as Camera Module 3. The AI Camera has manually adjustable focus; do not
expect `--autofocus-mode` or `--lens-position` to actuate its lens.

## Picamera2 Programming

Picamera2 is the Raspberry Pi Python camera interface. Install it through the
OS package manager to match the system camera stack:

```sh
sudo apt install python3-picamera2
```

For API examples, consult the
[Picamera2 manual](https://datasheets.raspberrypi.com/camera/picamera2-manual.pdf).
The source covers preview, still capture, video, and camera configuration.
Important distinctions when developing a separate application:

- Preview, still, and video configurations choose different buffer and stream defaults.
- `main`, `lores`, and `raw` streams have different purposes and supported formats.
- More buffers can absorb scheduling delays but use additional memory.
- Queued frames may predate a capture request; `queue=False` changes this behavior,
  but does not by itself synchronize exposure with flight-controller pose.
- Preview-only transformations and capture transformations are not interchangeable.
- GUI preview backends need a display; headless capture does not.
- USB camera support is limited and does not replace the Tiny 1-C SDK integration.

Picamera2 is not a required runtime dependency of CATIA's `rpicam-still` backend.

## Use With CATIA on MORA

This section is project-specific guidance, separate from the Waveshare wiki.
First pass the standalone JPEG test. Then follow the
[MORA deployment guide](catia_camera_pipeline.html#deploy-to-mora).

CATIA's AIcam backend invokes `rpicam-still` to acquire photos. The supplied
service currently selects LWIRcam and EARcam; installing an AI Camera does not
automatically change that selection. For a controlled foreground test, stop
the installed service and start AIcam:

```sh
sudo systemctl stop catia.service
/home/air/digital_cam/catia --aicam --debug
```

Run this on MORA with its FC UART connected. Trigger `DC_SHOOT` from the flight
controller or NPS and select AIcam in the camera mask (bit 1, mask value `2`).
Inspect `photos/aNNNNNN.jpg`, EXIF processing, and SODA results. After stopping
the foreground run with `Ctrl+C`, restore the configured service:

```sh
sudo systemctl start catia.service
```

This tests optical capture, not IMX500 neural-network result ingestion. The
standalone detection demos above do not establish that CATIA consumes their
bounding boxes, classifications, or poses. Do not run them alongside CATIA.

## Troubleshooting

| Symptom | Check |
| --- | --- |
| No camera listed | Power off, reseat the correct ribbon, verify the connector and OS/IMX500 packages. |
| Preview fails over SSH | Use `--nopreview` for capture; a normal SSH shell is not a display session. |
| Camera busy | Stop CATIA and other camera applications before a standalone test. |
| Inference asset missing | Check `imx500-all` and the paths in `/usr/share/rpi-camera-assets`. |
| Image is blurred | Adjust physical focus, inspect exposure time, and test with a stationary target. |
| Low frame rate | Check requested sensor mode, processing, memory, power, and storage throughput. |
| CLI photo works but CATIA does not | Check selected backend/mask, `rpicam-still` availability, UART messages, and CATIA logs. |

No physical AI Camera test was performed when this document was added.

## Sources and Regeneration

- [Raspberry Pi camera software](https://www.raspberrypi.com/documentation/computers/camera_software.html)
- [Raspberry Pi camera hardware](https://www.raspberrypi.com/documentation/accessories/camera.html)
- [Picamera2 manual](https://datasheets.raspberrypi.com/camera/picamera2-manual.pdf)

Edit `raspberry_pi_ai_camera.md`, then run from the Paparazzi repository root:

```sh
make -C sw/airborne/modules/digital_cam/catia documentation
```

This regenerates `raspberry_pi_ai_camera.html` from the local Markdown alongside
the other guides. Do not edit the generated HTML.
Regeneration does not overwrite this Markdown.

## Backup

Now you have a working OS, it is a good time to backup the SD to an compressed img file via a script found in CATIA "tools" directory

```sh
sudo ./fast_sd_restore.sh mora_os_backup.img.xz /dev/sdX
```

Where /dev/sdX needs to be your real device e.g. "/dev/sda" of the source SD card device

### Restore

And in case you need to restore the whole ready prepared SD to e.g another SD via a script found in CATIA "tools" directory

```sh
sudo ./fast_sd_restore_and_expand_CAREFULL.sh mora_os_backup.img.xz /dev/sdX
```

Where /dev/sdX needs to be your real device e.g. "/dev/sda" of the target SD card device

May the Force be with you...
