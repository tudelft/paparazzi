#!/usr/bin/env python3
"""Overlay an EAR loudest-spot heatmap on satellite imagery.

Ground-side companion of CATIA's earcam backend. The in-flight heatmap
(photos/eNNNNNN.jpg) is rendered on MORA without any map data; this tool takes
that JPEG plus its .geo sidecar, fetches the satellite tiles covering the same
ground (same Google source and var/maps/Google cache as the Paparazzi GCS, so
tiles are shared both ways), blends the acoustic field translucently over them
and redraws the track dots, the loudest-spot marker and the scale bar crisply.

    ear_heatmap_overlay.py photos/e000686.jpg            -> photos/e000686_sat.jpg
    ear_heatmap_overlay.py photos/e000686.jpg --log earlogs/ear_20260908_103533.csv \
        --track var/nps_earcam/EasystarM4_earcam_mission.track.csv --truth 48.8105,7.8516

--log redraws the sample positions as antialiased dots from the session CSV,
--track draws the flown aircraft track (t_s,lat_deg,lon_deg,... CSV, e.g. from
nps_earcam_mission.py) on top, --search LAT,LON[,R] draws the area to search
(the point given by the organisers and its 25 m circle), --truth marks a known
source position (simulation) with a red diamond. The fused loudest spot is the
magenta crosshair. Requires Pillow and numpy.
"""

import argparse
import math
import sys
import time
import urllib.request
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

TILE_PX = 256
EARTH_RADIUS_M = 6378137.0
NO_DATA_RGB = np.array([18, 18, 24], dtype=np.float64)   # ear_heatmap.c "no data" colour
SOURCES = {
    "google": "http://mt1.google.com/vt/lyrs=s&x={x}&y={y}&z={z}",
    "esri": "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
}


def read_geo(path):
    geo = {}
    for line in Path(path).read_text().splitlines():
        if line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        geo[key.strip()] = float(value)
    for key in ("width_px", "height_px", "m_per_px", "lat_center", "lon_center", "lat_north", "lon_west"):
        if key not in geo:
            raise SystemExit(f"{path}: missing {key}")
    return geo


def read_log(path):
    """(t_s, lat, lon) per sample from an earlogs/ear_*.csv, skipping comments and header."""
    return read_points(path, 0, 2, 3, 1000.0)


def read_track(path):
    """(t_s, lat, lon) per fix from a track CSV: t_s,lat_deg,lon_deg,..."""
    return read_points(path, 0, 1, 2, 1.0)


def read_points(path, t_col, lat_col, lon_col, t_divisor):
    points = []
    for line in Path(path).read_text().splitlines():
        if not line or line[0] in "#t":
            continue
        fields = line.split(",")
        if len(fields) <= max(t_col, lat_col, lon_col):
            continue
        lat, lon = float(fields[lat_col]), float(fields[lon_col])
        if abs(lat) > 1e-6 and math.isfinite(lat) and math.isfinite(lon):
            points.append((float(fields[t_col]) / t_divisor, lat, lon))
    return points


# --- Web Mercator tiles ------------------------------------------------------

def parse_search(text):
    parts = [float(v) for v in text.split(",")]
    if len(parts) < 2:
        raise SystemExit("--search expects LAT,LON[,RADIUS_M]")
    return parts[0], parts[1], parts[2] if len(parts) > 2 else 25.0


def haversine_m(lat_a, lon_a, lat_b, lon_b):
    d_lat = math.radians(lat_b - lat_a)
    d_lon = math.radians(lon_b - lon_a)
    h = math.sin(d_lat / 2) ** 2 + math.cos(math.radians(lat_a)) * math.cos(math.radians(lat_b)) * math.sin(d_lon / 2) ** 2
    return 2.0 * EARTH_RADIUS_M * math.asin(math.sqrt(h))

def mercator_xy(lat, lon, zoom):
    """Fractional tile coordinates (x right, y down) at zoom."""
    n = 2.0 ** zoom
    x = (lon + 180.0) / 360.0 * n
    lat_r = math.radians(lat)
    y = (1.0 - math.log(math.tan(lat_r) + 1.0 / math.cos(lat_r)) / math.pi) / 2.0 * n
    return x, y


def mercator_latlon(x, y, zoom):
    n = 2.0 ** zoom
    lon = x / n * 360.0 - 180.0
    lat = math.degrees(math.atan(math.sinh(math.pi * (1.0 - 2.0 * y / n))))
    return lat, lon


def gcs_key(x, y, zoom):
    """Keyhole string used by the Paparazzi GCS tile cache (sw/lib/ocaml/gm.ml)."""
    quadrant = {(0, 0): "q", (1, 0): "r", (1, 1): "s", (0, 1): "t"}   # (x, y), y down
    key = "t"
    for bit in range(zoom - 1, -1, -1):
        key += quadrant[((x >> bit) & 1, (y >> bit) & 1)]
    return key


def fetch_tile(x, y, zoom, source, cache_dir, quiet):
    url = SOURCES[source].format(x=x, y=y, z=zoom)
    if source == "google":
        cache = cache_dir / "Google" / f"{gcs_key(x, y, zoom)}.jpg"
    else:
        cache = cache_dir / source / str(zoom) / f"{x}_{y}.jpg"
    if cache.exists():
        return Image.open(cache).convert("RGB")
    cache.parent.mkdir(parents=True, exist_ok=True)
    request = urllib.request.Request(url, headers={"User-Agent": "paparazzi-earcam-overlay"})
    for attempt in range(3):
        try:
            data = urllib.request.urlopen(request, timeout=15).read()
            cache.write_bytes(data)
            if not quiet:
                print(f"fetched {source} tile z{zoom} {x}/{y}")
            return Image.open(cache).convert("RGB")
        except Exception as error:  # network hiccup: retry, then give up on this tile
            if attempt == 2:
                print(f"warning: tile z{zoom} {x}/{y}: {error}", file=sys.stderr)
            time.sleep(1.0)
    return Image.new("RGB", (TILE_PX, TILE_PX), (40, 40, 40))


def choose_zoom(geo, max_zoom):
    """Zoom whose ground resolution is at least as fine as the heatmap pixel."""
    lat = geo["lat_center"]
    for zoom in range(12, max_zoom + 1):
        m_per_px = 2.0 * math.pi * EARTH_RADIUS_M * math.cos(math.radians(lat)) / (TILE_PX * 2.0 ** zoom)
        if m_per_px <= geo["m_per_px"]:
            return zoom
    return max_zoom


# --- Compositing -------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("heatmap", type=Path, help="photos/eNNNNNN.jpg written by CATIA (needs the .geo sidecar)")
    parser.add_argument("--geo", type=Path, help="georeference sidecar (default: heatmap with .geo)")
    parser.add_argument("--log", type=Path, help="earlogs CSV of the session, to redraw the sample dots crisply")
    parser.add_argument("--track", type=Path, help="aircraft track CSV (t_s,lat_deg,lon_deg,...) drawn on top")
    parser.add_argument("--truth", help="known source position LAT,LON (simulation), drawn as a red marker")
    parser.add_argument("--search", help="search area given by the organisers LAT,LON[,RADIUS_M] (default radius 25)")
    parser.add_argument("-o", "--output", type=Path, help="output JPEG (default: <heatmap>_sat.jpg)")
    parser.add_argument("--source", choices=sorted(SOURCES), default="google")
    parser.add_argument("--zoom", type=int, help="tile zoom (default: matches the heatmap resolution)")
    parser.add_argument("--max-zoom", type=int, default=20)
    parser.add_argument("--alpha", type=float, default=0.55, help="heatmap opacity over the imagery (0..1)")
    parser.add_argument("--cache", type=Path, default=Path(__file__).resolve().parents[5] / "var" / "maps",
                        help="tile cache root, shared with the GCS (default: <paparazzi>/var/maps)")
    parser.add_argument("--margin-m", type=float, default=15.0, help="imagery margin around the heatmap")
    parser.add_argument("--track-gap-s", type=float, default=1.5, help="fix interval that breaks the track line")
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args()

    geo = read_geo(args.geo or args.heatmap.with_suffix(".geo"))
    # Prefer the undecorated field (ear_heatmap_replay writes <stem>_field.jpg) so the
    # in-flight dots, marker and scale bar are not blended into the imagery.
    field_path = args.heatmap.with_name(args.heatmap.stem + "_field.jpg")
    heat = np.asarray(Image.open(field_path if field_path.exists() else args.heatmap).convert("RGB"),
                      dtype=np.float64)
    zoom = args.zoom or choose_zoom(geo, args.max_zoom)

    # Heatmap footprint in equirectangular metres about its centre (as ear_heatmap.c).
    m_per_deg_lat = math.radians(1.0) * EARTH_RADIUS_M
    m_per_deg_lon = m_per_deg_lat * math.cos(math.radians(geo["lat_center"]))
    east0 = (geo["lon_west"] - geo["lon_center"]) * m_per_deg_lon      # heatmap x=0
    north0 = (geo["lat_north"] - geo["lat_center"]) * m_per_deg_lat    # heatmap y=0
    width_m = geo["width_px"] * geo["m_per_px"]
    height_m = geo["height_px"] * geo["m_per_px"]

    # Output raster: Mercator tile grid covering the footprint plus margin (and the search circle).
    margin = args.margin_m
    if args.search:
        s_lat, s_lon, s_radius = parse_search(args.search)
        s_east = (s_lon - geo["lon_center"]) * m_per_deg_lon
        s_north = (s_lat - geo["lat_center"]) * m_per_deg_lat
        margin = max(margin, s_east + s_radius - (east0 + width_m) + 5.0, east0 - (s_east - s_radius) + 5.0,
                     s_north + s_radius - north0 + 5.0, (north0 - height_m) - (s_north - s_radius) + 5.0)
    lat_n = geo["lat_center"] + (north0 + margin) / m_per_deg_lat
    lat_s = geo["lat_center"] + (north0 - height_m - margin) / m_per_deg_lat
    lon_w = geo["lon_center"] + (east0 - margin) / m_per_deg_lon
    lon_e = geo["lon_center"] + (east0 + width_m + margin) / m_per_deg_lon
    x_w, y_n = mercator_xy(lat_n, lon_w, zoom)
    x_e, y_s = mercator_xy(lat_s, lon_e, zoom)
    tx0, ty0 = int(math.floor(x_w)), int(math.floor(y_n))
    tx1, ty1 = int(math.floor(x_e)), int(math.floor(y_s))
    mosaic = Image.new("RGB", ((tx1 - tx0 + 1) * TILE_PX, (ty1 - ty0 + 1) * TILE_PX))
    for ty in range(ty0, ty1 + 1):
        for tx in range(tx0, tx1 + 1):
            mosaic.paste(fetch_tile(tx, ty, zoom, args.source, args.cache, args.quiet),
                         ((tx - tx0) * TILE_PX, (ty - ty0) * TILE_PX))
    px0, py0 = int((x_w - tx0) * TILE_PX), int((y_n - ty0) * TILE_PX)
    px1, py1 = int(math.ceil((x_e - tx0) * TILE_PX)), int(math.ceil((y_s - ty0) * TILE_PX))
    base = np.asarray(mosaic.crop((px0, py0, px1, py1)), dtype=np.float64)
    out_h, out_w = base.shape[:2]
    # Snap the footprint to the crop's whole pixels so the mapping below is exact.
    x_w, x_e = tx0 + px0 / TILE_PX, tx0 + px1 / TILE_PX
    y_n, y_s = ty0 + py0 / TILE_PX, ty0 + py1 / TILE_PX

    # Every output pixel -> lat/lon (Mercator) -> heatmap pixel (equirectangular).
    ys, xs = np.mgrid[0:out_h, 0:out_w]
    merc_x = x_w + (xs + 0.5) / (px1 - px0) * (x_e - x_w)
    merc_y = y_n + (ys + 0.5) / (py1 - py0) * (y_s - y_n)
    n = 2.0 ** zoom
    lon = merc_x / n * 360.0 - 180.0
    lat = np.degrees(np.arctan(np.sinh(np.pi * (1.0 - 2.0 * merc_y / n))))
    hx = ((lon - geo["lon_center"]) * m_per_deg_lon - east0) / geo["m_per_px"]
    hy = (north0 - (lat - geo["lat_center"]) * m_per_deg_lat) / geo["m_per_px"]
    inside = (hx >= 0) & (hx < heat.shape[1]) & (hy >= 0) & (hy < heat.shape[0])
    ix = np.clip(hx.astype(int), 0, heat.shape[1] - 1)
    iy = np.clip(hy.astype(int), 0, heat.shape[0] - 1)
    sampled = heat[iy, ix]
    no_data = np.linalg.norm(sampled - NO_DATA_RGB, axis=2) < 24.0   # JPEG noise tolerance
    alpha = np.where(inside & ~no_data, args.alpha, 0.0)[..., None]
    out = base * (1.0 - alpha) + sampled * alpha

    # Redraw crisp overlays at 4x supersampling: dots, spot marker, scale bar.
    ss = 4
    layer = Image.new("RGBA", (out_w * ss, out_h * ss), (0, 0, 0, 0))
    draw = ImageDraw.Draw(layer)

    def to_px(lat_deg, lon_deg):
        mx, my = mercator_xy(lat_deg, lon_deg, zoom)
        return ((mx - x_w) / (x_e - x_w) * (px1 - px0) * ss,
                (my - y_n) / (y_s - y_n) * (py1 - py0) * ss)

    if args.log:
        # Sample positions: 1 px white dot in a 1 px black ring (antialiased by the 4x downscale).
        for _, lat_deg, lon_deg in read_log(args.log):
            cx, cy = to_px(lat_deg, lon_deg)
            draw.ellipse((cx - 2 * ss, cy - 2 * ss, cx + 2 * ss, cy + 2 * ss), fill=(0, 0, 0, 255))
            draw.ellipse((cx - ss, cy - ss, cx + ss, cy + ss), fill=(255, 255, 255, 255))
    # Ground resolution of the output at the centre latitude (Mercator tile pixel).
    m_per_out_px = 2.0 * math.pi * EARTH_RADIUS_M * math.cos(math.radians(geo["lat_center"])) / (TILE_PX * n)

    def label(x, y, text, rgb):
        draw.text((x, y), text, fill=rgb + (255,), font_size=15 * ss, stroke_width=2 * ss, stroke_fill=(0, 0, 0, 255))

    if args.search:
        # The area to search (rulebook: mannequins within 25 m of the given point).
        cx, cy = to_px(s_lat, s_lon)
        r = s_radius / m_per_out_px * ss
        draw.ellipse((cx - r, cy - r, cx + r, cy + r), outline=(0, 0, 0, 200), width=5 * ss)
        draw.ellipse((cx - r, cy - r, cx + r, cy + r), outline=(255, 255, 255, 255), width=2 * ss)
        for dx, dy in ((6 * ss, 0), (0, 6 * ss)):
            draw.line((cx - dx, cy - dy, cx + dx, cy + dy), fill=(0, 0, 0, 255), width=4 * ss)
            draw.line((cx - dx, cy - dy, cx + dx, cy + dy), fill=(255, 255, 255, 255), width=2 * ss)
        label(cx + 8 * ss, cy - 20 * ss, "given point", (255, 255, 255))
        label(cx - 60 * ss, cy - r - 22 * ss, f"search area r = {s_radius:.0f} m", (255, 255, 255))
    bar = min(50.0 / m_per_out_px, 0.6 * out_w) * ss
    bx, by = 0.04 * out_w * ss, (out_h - 0.06 * out_h) * ss
    draw.line((bx, by, bx + bar, by), fill=(0, 0, 0, 255), width=5 * ss)
    draw.line((bx, by, bx + bar, by), fill=(255, 255, 255, 255), width=2 * ss)
    for x in (bx, bx + bar):
        draw.line((x, by - 6 * ss, x, by + 6 * ss), fill=(0, 0, 0, 255), width=4 * ss)
        draw.line((x, by - 6 * ss, x, by + 6 * ss), fill=(255, 255, 255, 255), width=2 * ss)
    draw.text((bx, by - 22 * ss), "50 m", fill=(255, 255, 255, 255), font_size=14 * ss,
              stroke_width=2 * ss, stroke_fill=(0, 0, 0, 255))
    if args.track:
        # Flown track on top of everything, broken where fixes are missing.
        fixes = read_track(args.track)
        segments, segment = [], []
        for index, (stamp, lat_deg, lon_deg) in enumerate(fixes):
            if segment and stamp - fixes[index - 1][0] > args.track_gap_s:
                segments.append(segment)
                segment = []
            segment.append(to_px(lat_deg, lon_deg))
        segments.append(segment)
        for segment in segments:
            if len(segment) > 1:
                draw.line(segment, fill=(0, 0, 0, 230), width=3 * ss, joint="curve")
                draw.line(segment, fill=(255, 235, 120, 255), width=1 * ss, joint="curve")

    # Point markers last so nothing covers them.
    def crosshair(cx, cy, rgb, ring, arm, gap):
        for casing, colour, w in ((True, (0, 0, 0), 5), (False, rgb, 2)):
            for dx, dy in ((1, 0), (0, 1)):
                draw.line((cx + dx * gap, cy + dy * gap, cx + dx * arm, cy + dy * arm), fill=colour + (255,), width=w * ss)
                draw.line((cx - dx * gap, cy - dy * gap, cx - dx * arm, cy - dy * arm), fill=colour + (255,), width=w * ss)
            draw.ellipse((cx - ring, cy - ring, cx + ring, cy + ring), outline=colour + (255,), width=w * ss)
        draw.ellipse((cx - 1.5 * ss, cy - 1.5 * ss, cx + 1.5 * ss, cy + 1.5 * ss), fill=rgb + (255,))

    if "spot_lat" in geo:
        cx, cy = to_px(geo["spot_lat"], geo["spot_lon"])
        crosshair(cx, cy, (255, 40, 255), 9 * ss, 24 * ss, 4 * ss)
        label(cx + 12 * ss, cy + 10 * ss, "loudest spot (drop here)", (255, 40, 255))
    if args.truth:
        t_lat, t_lon = (float(v) for v in args.truth.split(",")[:2])
        cx, cy = to_px(t_lat, t_lon)
        # Red diamond so it stays distinct from the loudest-spot crosshair right next to it.
        d = 9 * ss
        diamond = [(cx, cy - d), (cx + d, cy), (cx, cy + d), (cx - d, cy)]
        draw.polygon(diamond, outline=(0, 0, 0, 255), width=5 * ss)
        draw.polygon(diamond, outline=(255, 40, 40, 255), width=2 * ss)
        text = "sound source"
        if "spot_lat" in geo:
            text += f" ({haversine_m(t_lat, t_lon, geo['spot_lat'], geo['spot_lon']):.1f} m off)"
        label(cx + 12 * ss, cy - 26 * ss, text, (255, 40, 40))
    layer = layer.resize((out_w, out_h), Image.LANCZOS)
    overlay = np.asarray(layer, dtype=np.float64)
    a = overlay[..., 3:4] / 255.0
    out = out * (1.0 - a) + overlay[..., :3] * a

    output = args.output or args.heatmap.with_name(args.heatmap.stem + "_sat.jpg")
    Image.fromarray(np.clip(out + 0.5, 0, 255).astype(np.uint8)).save(output, quality=92)
    print(f"wrote {output} ({out_w}x{out_h}, {args.source} z{zoom}, {m_per_out_px:.3f} m/px)")


if __name__ == "__main__":
    main()
