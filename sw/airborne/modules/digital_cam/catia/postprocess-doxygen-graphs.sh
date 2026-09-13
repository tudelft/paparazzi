#!/usr/bin/env bash
# Normalize Doxygen's Graphviz PNGs to the CATIA documentation palette.
# Doxygen emits include/relation edge colors directly into raster images, so CSS
# cannot style them after generation. Preserve nodes and labels while replacing
# only the default light-blue relation strokes with black on white.
set -euo pipefail

html_dir=${1:-doxygen/html}
[[ -d "$html_dir" ]] || exit 0

while IFS= read -r -d '' graph; do
  magick "$graph" -background white -alpha remove -alpha off \
    -fuzz 18% -fill black -opaque '#63B8FF' "$graph"
done < <(find "$html_dir" -type f -name '*.png' -print0)
