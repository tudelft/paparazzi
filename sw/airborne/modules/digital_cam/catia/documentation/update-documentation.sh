#!/usr/bin/env bash

set -euo pipefail

DOCUMENTATION_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
cd "$DOCUMENTATION_DIR"

HTML_DIR="$DOCUMENTATION_DIR/html"
mkdir -p "$HTML_DIR"

if ! command -v dot >/dev/null 2>&1; then
  echo "update-documentation.sh: Graphviz 'dot' is required" >&2
  exit 1
fi
if ! command -v inkscape >/dev/null 2>&1; then
  echo "update-documentation.sh: Inkscape is required" >&2
  exit 1
fi
if ! command -v magick >/dev/null 2>&1; then
  echo "update-documentation.sh: ImageMagick 'magick' is required" >&2
  exit 1
fi
if ! command -v python3 >/dev/null 2>&1; then
  echo "update-documentation.sh: python3 is required" >&2
  exit 1
fi
if ! python3 -c 'import markdown' >/dev/null 2>&1; then
  echo "update-documentation.sh: Python package 'Markdown' is required" >&2
  echo "Install it with: python3 -m pip install Markdown" >&2
  exit 1
fi

dot -Tsvg catia-flow.dot -o "$HTML_DIR/catia-flow.svg"
dot -Tpng -Gdpi=180 catia-flow.dot -o "$HTML_DIR/catia-flow.png"
dot -Tsvg earcam-dataflow.dot -o "$HTML_DIR/earcam-dataflow.svg"
dot -Tpng -Gdpi=180 earcam-dataflow.dot -o "$HTML_DIR/earcam-dataflow.png"
dot -Tsvg lwir-calibration-workflow.dot -o "$HTML_DIR/lwir-calibration-workflow.svg"
dot -Tpng -Gdpi=180 lwir-calibration-workflow.dot -o "$HTML_DIR/lwir-calibration-workflow.png"

python3 generate_calibration_illustrations.py
for diagram in target coverage mount; do
  inkscape "$HTML_DIR/lwir-calibration-$diagram.svg" --export-type=png \
    --export-width=1320 --export-filename="$HTML_DIR/lwir-calibration-$diagram.png"
done

desk_base=$(mktemp --suffix=.png)
desk_screen=$(mktemp --suffix=.png)
trap 'rm -f "$desk_base" "$desk_screen"' EXIT
inkscape desk-test-setup.svg \
  --export-type=png \
  --export-filename="$desk_base"
magick thelaptopscreen.png \
  -resize '326x176^' \
  -gravity center \
  -extent 326x176 \
  "$desk_screen"
magick "$desk_base" "$desk_screen" \
  -geometry +127+236 \
  -composite \
  "$HTML_DIR/desk-test-setup.png"
cp desk-test-setup.svg "$HTML_DIR/desk-test-setup.svg"

# Copy static assets needed by HTML docs
for static_img in earcam-sound-picture-example.jpg imav2026_m4_nps_overview.jpg; do
  if [[ -f "$static_img" ]]; then
    cp "$static_img" "$HTML_DIR/"
  fi
done

python3 generate_html.py

for generated_file in "$HTML_DIR/catia-flow.svg" "$HTML_DIR/catia-flow.png" \
                      "$HTML_DIR/earcam-dataflow.svg" "$HTML_DIR/earcam-dataflow.png" \
                      "$HTML_DIR/desk-test-setup.png" "$HTML_DIR/desk-test-setup.svg" \
                      "$HTML_DIR/index.html" "$HTML_DIR/catia_camera_pipeline.html" \
                      "$HTML_DIR/earcam-loudest-spot-explained.html" "$HTML_DIR/earcam-dataflow.html" \
                      "$HTML_DIR/lwir-calibration.html" "$HTML_DIR/mission2-score-first.html" \
                      "$HTML_DIR/raspberry_pi_ai_camera.html" "$HTML_DIR/setup_os_rpi_zero_2w.html" \
                      "$HTML_DIR/precision-landing-flight-test.html" \
                      "$HTML_DIR/lwir-calibration-workflow.png" "$HTML_DIR/lwir-calibration-workflow.svg" \
                      "$HTML_DIR/lwir-calibration-target.png" "$HTML_DIR/lwir-calibration-target.svg" \
                      "$HTML_DIR/lwir-calibration-coverage.png" "$HTML_DIR/lwir-calibration-coverage.svg" \
                      "$HTML_DIR/lwir-calibration-mount.png" "$HTML_DIR/lwir-calibration-mount.svg" \
                      "$HTML_DIR/earcam-sound-picture-example.jpg" \
                      "$HTML_DIR/imav2026_m4_nps_overview.jpg"; do
  if [[ ! -s "$generated_file" ]]; then
    echo "update-documentation.sh: failed to generate $generated_file" >&2
    exit 1
  fi
done

echo "CATIA documentation updated: Hub, Camera Pipeline, Pi Zero 2 W setup, EARcam, EARcam dataflow, LWIR calibration, Mission 2, Precision Landing and Raspberry Pi AI Camera guides in $HTML_DIR"
