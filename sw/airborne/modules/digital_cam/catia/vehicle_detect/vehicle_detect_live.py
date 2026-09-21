#!/usr/bin/env python3
"""Standalone live IMX500 vehicle-detection desk test.

Opens the Raspberry Pi AI Camera once, keeps it open in a loop, and runs the
fine-tuned single-class ("vehicle") network on real camera frames -- NOT the
tensor-injection debug path used by imx500_object_detection_injection_demo.py.
On a qualifying detection, saves the full frame (no crop) to --save-dir.

This is deliberately independent of CATIA/UART/flight control. See
Notes/Mission 1/IMAV input tensor injection.md for the bbox_order and firmware
upload timing facts this script relies on.

Usage (on the Pi, from /home/air/vehicle_detect):
    python3 vehicle_detect_live.py --model model/network.rpk --labels model/labels.txt
"""
import argparse
import os
import sys
import time

from PIL import ImageDraw, ImageFont
from picamera2 import Picamera2
from picamera2.devices import IMX500
from picamera2.devices.imx500 import NetworkIntrinsics


def get_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", default="model/network.rpk", help="Path to the packaged .rpk network")
    parser.add_argument("--labels", default="model/labels.txt", help="Path to the labels file")
    parser.add_argument("--save-dir", default="captures", help="Directory to save detection frames into")
    parser.add_argument("--threshold", type=float, default=0.6, help="Detection confidence threshold")
    parser.add_argument("--iou", type=float, default=0.65, help="IoU threshold (kept for parity; NMS is baked on-chip)")
    parser.add_argument("--max-detections", type=int, default=5, help="Cap on detections per frame")
    parser.add_argument("--cooldown", type=float, default=5.0, help="Minimum seconds between saves while continuously tracking")
    parser.add_argument("--reset-gap", type=float, default=2.0, help="Seconds of continuous absence before a sighting is considered over")
    parser.add_argument("--fps", type=int, default=None, help="Manual FrameRate override (default: intrinsics.inference_rate, else 10)")
    parser.add_argument("--main-size", default="1536x1152", help="WxH of the saved main stream")
    parser.add_argument("--print-intrinsics", action="store_true", help="Print network intrinsics then exit (no camera loop)")
    return parser.parse_args()


class Detection:
    def __init__(self, coords, category, conf, metadata, imx500, picam2):
        """Create a Detection, mapping raw network coords to real image pixels.

        This is a genuine capture (unlike tensor injection), so the ISP/ScalerCrop-
        aware imx500.convert_inference_coords() is the correct mapping here -- the
        injection demo skips that call because it has no real capture behind its
        metadata; this script must NOT copy that shortcut.
        """
        self.category = category
        self.conf = conf
        self.box = imx500.convert_inference_coords(coords, metadata, picam2)


def parse_detections(metadata, imx500, picam2, intrinsics, args):
    """Parse the output tensor into a list of Detection objects."""
    # imx500.config["input_tensor_size"] can transiently read (0, 0) right after
    # certain events (see Notes/Mission 1/IMAV input tensor injection.md, item 7).
    # get_input_size() is the same static network property but doesn't suffer
    # that race. Cheap, harmless guard, carried forward from the injection harness.
    if imx500.get_input_size() == (0, 0):
        return []

    np_outputs = imx500.get_outputs(metadata, add_batch=True)
    if np_outputs is None:
        return []

    input_w, input_h = imx500.get_input_size()
    boxes, scores, classes = np_outputs[0][0], np_outputs[1][0], np_outputs[2][0]

    if intrinsics.bbox_normalization:
        boxes = boxes / input_h

    if intrinsics.bbox_order == "xy":
        boxes = boxes[:, [1, 0, 3, 2]]

    detections = []
    for box, score, category in zip(boxes, scores, classes):
        if score > args.threshold:
            detections.append(Detection(box, category, score, metadata, imx500, picam2))
        if len(detections) >= args.max_detections:
            break
    return detections


def make_filename(conf):
    ts = time.strftime("%Y%m%d_%H%M%S")
    ms = int((time.time() % 1) * 1000)
    return f"vehicle_{ts}_{ms:03d}_conf{conf:.2f}.jpg"


def load_label_font():
    try:
        return ImageFont.load_default(size=32)
    except TypeError:
        return ImageFont.load_default()  # older Pillow without the size= param


def draw_hits(image, hits, font):
    """Draw a box + confidence label for every qualifying detection, in place."""
    draw = ImageDraw.Draw(image)
    for d in hits:
        x, y, w, h = d.box
        draw.rectangle([x, y, x + w, y + h], outline=(0, 255, 0), width=4)
        label = f"vehicle {d.conf:.2f}"
        label_origin = (x, max(0, y - 34))
        label_box = draw.textbbox(label_origin, label, font=font)
        draw.rectangle(label_box, fill=(0, 255, 0))
        draw.text(label_origin, label, fill=(0, 0, 0), font=font)


def main():
    args = get_args()

    imx500 = IMX500(args.model)  # NOT tensor_injection=True -- live capture, not debug
    intrinsics = imx500.network_intrinsics
    if not intrinsics:
        intrinsics = NetworkIntrinsics()
        intrinsics.task = "object detection"
    elif intrinsics.task != "object detection":
        sys.exit(f"Packaged network task is {intrinsics.task!r}, not 'object detection' -- wrong .rpk?")

    with open(args.labels) as f:
        intrinsics.labels = [line.strip() for line in f if line.strip()]

    # Confirmed via the tensor-injection harness: this model's NMSWrapper export
    # emits (x_min, y_min, x_max, y_max), the opposite of picamera2's "yx" default.
    # A property of the export itself, applies identically in live capture.
    intrinsics.bbox_order = "xy"
    # This export's raw boxes are absolute pixel coords within the 320x320 input
    # tensor, NOT already-normalized [0,1] fractions (confirmed in the injection
    # harness notes). imx500.convert_inference_coords() requires normalized input
    # (it multiplies by the full sensor width/height itself) -- bbox_normalization
    # True is what makes parse_detections() divide by input_h first. Without this,
    # boxes get multiplied by the sensor size TWICE, landing millions of pixels
    # off-canvas -- which is exactly why no box was visible before this fix.
    intrinsics.bbox_normalization = True
    intrinsics.iou_threshold = args.iou
    intrinsics.max_detections = args.max_detections

    intrinsics.update_with_defaults()

    if intrinsics.postprocess:
        sys.exit(f"Unexpected postprocess mode {intrinsics.postprocess!r}; "
                  "this script only supports the raw NMSWrapper output path.")

    labels = intrinsics.labels
    if "vehicle" not in labels:
        sys.exit(f"'vehicle' not found in labels: {labels}")
    vehicle_idx = labels.index("vehicle")

    if args.print_intrinsics:
        print(intrinsics)
        return

    os.makedirs(args.save_dir, exist_ok=True)

    frame_rate = args.fps or (intrinsics.inference_rate or 10)

    picam2 = Picamera2(imx500.camera_num)
    width, height = (int(v) for v in args.main_size.split("x"))
    config = picam2.create_preview_configuration(
        main={"size": (width, height)},
        controls={"FrameRate": frame_rate},
        buffer_count=4,
    )

    print("Uploading network firmware to the IMX500 sensor -- observed 17s clean, "
          "up to 48-60s if the rp2040-gpio-bridge chatter is active. This happens "
          "once for the whole run, not per detection.")
    imx500.show_network_fw_progress_bar()
    picam2.start(config, show_preview=False)

    if intrinsics.preserve_aspect_ratio:
        imx500.set_auto_aspect_ratio()

    print(f"Camera + network ready ({labels}, vehicle_idx={vehicle_idx}, "
          f"threshold={args.threshold}, cooldown={args.cooldown}s, "
          f"reset_gap={args.reset_gap}s). Entering detection loop (Ctrl+C to stop)...")

    label_font = load_label_font()

    state = "IDLE"
    last_positive_time = 0.0
    last_save_time = 0.0

    try:
        while True:
            request = picam2.capture_request()
            try:
                metadata = request.get_metadata()
                detections = parse_detections(metadata, imx500, picam2, intrinsics, args)
                hits = [d for d in detections
                        if int(d.category) == vehicle_idx and d.conf >= args.threshold]

                now = time.monotonic()
                if hits:
                    best = max(hits, key=lambda d: d.conf)
                    fresh = (state == "IDLE")
                    state = "TRACKING"
                    last_positive_time = now

                    due = fresh or (now - last_save_time) >= args.cooldown
                    if due:
                        fname = os.path.join(args.save_dir, make_filename(best.conf))
                        image = request.make_image("main")
                        draw_hits(image, hits, label_font)
                        image.save(fname)
                        last_save_time = now
                        print(f"[DETECT] vehicle conf={best.conf:.2f} box={best.box} "
                              f"-> saved {fname} (box drawn for {len(hits)} detection(s))")
                    else:
                        wait = args.cooldown - (now - last_save_time)
                        print(f"[DETECT] vehicle conf={best.conf:.2f} (debounced, {wait:.1f}s to next save)")
                else:
                    if state == "TRACKING" and (now - last_positive_time) > args.reset_gap:
                        state = "IDLE"
                        print("[INFO] vehicle no longer in view")
            finally:
                request.release()

    except KeyboardInterrupt:
        print("\nCtrl+C received, shutting down...")
    finally:
        picam2.stop()
        picam2.close()
        print("Camera closed cleanly.")


if __name__ == "__main__":
    main()
