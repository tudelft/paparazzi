#!/usr/bin/env bash

set -euo pipefail

DOCUMENTATION_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
cd "$DOCUMENTATION_DIR"

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

dot -Tsvg catia-flow.dot -o catia-flow.svg
dot -Tpng -Gdpi=180 catia-flow.dot -o catia-flow.png
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
  desk-test-setup.png
python3 generate_html.py

for generated_file in catia-flow.svg catia-flow.png desk-test-setup.png index.html; do
  if [[ ! -s "$generated_file" ]]; then
    echo "update-documentation.sh: failed to generate $generated_file" >&2
    exit 1
  fi
done

echo "CATIA documentation updated: README.md, index.html, catia-flow.svg, catia-flow.png"
