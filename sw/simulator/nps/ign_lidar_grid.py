#!/usr/bin/env python3
"""Fetch a local terrain/surface grid from the IGN LiDAR HD altimetry service (France).

For every grid point the service returns the terrain height (MNT), the surface height
including trees and buildings (MNS) and their difference (MNH), all 50 cm LiDAR HD
products. SRTM (30 m) cannot show a tree line 20 m wide; this can.

    ign_lidar_grid.py 48.81045 7.85170 300 10 data/terrain/imav2026_m4_ign_lidar_hd_10m.csv

writes a CSV with columns x_east_m, y_north_m, lat, lon, mnt_m, mns_m, mnh_m on a
(2*HALF_M/STEP_M + 1)^2 grid centred on LAT,LON. The CSV is read by
nps_earcam_mission.py (obstacle clearance check of the flown track) and by
catia/ear_heatmap_overlay.py --obstacles (red cells on the satellite picture).

Service: https://data.geopf.fr/altimetrie (free, no key; 100 points per request).
"""
import argparse
import csv
import json
import math
import sys
import time
import urllib.parse
import urllib.request

SERVICE = "https://data.geopf.fr/altimetrie/1.0/calcul/alti/rest/elevation.json"
RESOURCE = "ign_lidar_hd_mnx_mono_wld"
BATCH = 100


def fetch(points):
    query = urllib.parse.urlencode({
        "lon": "|".join(f"{p[3]:.7f}" for p in points),
        "lat": "|".join(f"{p[2]:.7f}" for p in points),
        "resource": RESOURCE,
        "measures": "true",
    })
    for attempt in range(5):
        try:
            with urllib.request.urlopen(SERVICE + "?" + query, timeout=90) as response:
                return json.load(response)["elevations"]
        except (OSError, ValueError, KeyError) as error:
            print(f"retry {attempt + 1}: {error}", file=sys.stderr)
            time.sleep(3.0)
    raise SystemExit("IGN altimetry service not reachable")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("lat", type=float)
    parser.add_argument("lon", type=float)
    parser.add_argument("half_m", type=float, help="half width of the square grid, m")
    parser.add_argument("step_m", type=float, help="grid spacing, m (the products are 50 cm; 5 to 10 m is plenty)")
    parser.add_argument("output")
    args = parser.parse_args()

    m_per_deg_lat = 111320.0
    m_per_deg_lon = 111320.0 * math.cos(math.radians(args.lat))
    n = int(round(2.0 * args.half_m / args.step_m)) + 1
    points = []
    for i in range(n):
        y = args.half_m - i * args.step_m
        for j in range(n):
            x = -args.half_m + j * args.step_m
            points.append((x, y, args.lat + y / m_per_deg_lat, args.lon + x / m_per_deg_lon))

    rows = []
    for start in range(0, len(points), BATCH):
        chunk = points[start:start + BATCH]
        for point, elevation in zip(chunk, fetch(chunk)):
            values = {"MNT": None, "MNS": None, "MNH": None}
            for measure in elevation.get("measures", []):
                for key in values:
                    if f"- {key} -" in measure.get("title", ""):
                        values[key] = measure["z"]
            rows.append((point[0], point[1], round(point[2], 7), round(point[3], 7),
                         values["MNT"], values["MNS"], values["MNH"]))
        print(f"{len(rows)}/{len(points)}", file=sys.stderr)

    with open(args.output, "w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["x_east_m", "y_north_m", "lat", "lon", "mnt_m", "mns_m", "mnh_m"])
        writer.writerows(rows)
    print(f"wrote {args.output}: {len(rows)} points, {args.step_m:g} m spacing")


if __name__ == "__main__":
    main()
