/*
 * Standalone Strasbourg-flyzone geofence check for IMAV Mision4
 * (mission4_strasbourg.xml).
 *
 * Split out of nav_couzin_fw.h on 2026-08-26: that module is swarm-
 * flocking logic for a different branch, but its geofence helpers
 * (point-in-polygon + outside buffer) are unrelated to the flocking
 * behavior and mission4 (Cherry flying solo) still needs them. This file
 * has no swarm/couzin dependency at all — same polygon and buffer
 * convention, copied verbatim rather than reimplemented, so mission4's
 * geofence behavior is unchanged by the split.
 */

#ifndef NAV_MISSION4_GEOFENCE_H
#define NAV_MISSION4_GEOFENCE_H

#include "std.h"

/**
 * Shared point-in-polygon and buffered-boundary tests. Both mission4 sites
 * use the same logic against different polygons, so the geometry lives here
 * once and each site below is just its own vertex list.
 */
static inline bool mission4_in_poly(const float *vx, const float *vy, int n, float x, float y)
{
  bool inside = false;
  int j = n - 1;
  for (int i = 0; i < n; i++) {
    if (((vy[i] > y) != (vy[j] > y)) &&
        (x < (vx[j] - vx[i]) * (y - vy[i]) / (vy[j] - vy[i]) + vx[i]))
      inside = !inside;
    j = i;
  }
  return inside;
}

/** Inside the polygon, or within buf metres OUTSIDE any boundary edge. */
static inline bool mission4_near_poly(const float *vx, const float *vy, int n,
                                      float x, float y, float buf)
{
  if (mission4_in_poly(vx, vy, n, x, y)) return true;
  float buf2 = buf * buf;
  for (int i = 0; i < n; i++) {
    int j = (i + 1) % n;
    float dx = vx[j] - vx[i], dy = vy[j] - vy[i];
    float len2 = dx*dx + dy*dy;
    float t = (len2 > 1e-9f) ? ((x - vx[i])*dx + (y - vy[i])*dy) / len2 : 0.f;
    if (t < 0.f) t = 0.f; else if (t > 1.f) t = 1.f;
    float ex = x - (vx[i] + t*dx), ey = y - (vy[i] + t*dy);
    if (ex*ex + ey*ey < buf2) return true;
  }
  return false;
}

/* 60 m OUTSIDE buffer for both sites: the safe zone is the flyzone polygon
   plus this margin beyond it. It exists so the launch and landing points,
   which sit just off the flyzone at both sites, do not trip the flight
   plan's geofence exception - Strasbourg's HOME is 16 m inside its polygon
   and Valkenburg's is 31 m outside its own. Widening it further would
   licence flight further outside the competition flyzone; the drop approach
   instead picks a bearing whose run-in stays inside (mission4_place_approach
   in the flight plan). */
#define MISSION4_GEOFENCE_BUFFER 60.0f

/**
 * mission4_in_strasbourg / _safe - FZ1-FZ6 Strasbourg flyzone hexagon,
 * refined in the pprzgcs Flight Plan Editor over satellite imagery (see the
 * FZ1-FZ6 waypoints and the Flyzone sector in mission4_strasbourg.xml).
 * West edge widened to the official IMAV Mission 1 Mapping Area 1 border:
 * FZ2 is Area 1's NW corner (48.810883N 7.851214E) and FZ1 is where the line
 * through Area 1's NW and SW (48.806895N 7.851654E) corners meets the old
 * FZ6-FZ1 edge. Those two corners were 18.5 m and 2.8 m outside the old
 * hexagon; the new one contains the old one entirely.
 */
static const float mission4_strasbourg_vx[6] = {-7.216f, -35.387f, 592.228f, 564.110f, 545.046f, 564.301f};
static const float mission4_strasbourg_vy[6] = {-241.108f, 246.917f, 419.242f,  56.754f,  26.094f, -67.131f};

static inline bool mission4_in_strasbourg(float x, float y)
{
  return mission4_in_poly(mission4_strasbourg_vx, mission4_strasbourg_vy, 6, x, y);
}

static inline bool mission4_in_strasbourg_safe(float x, float y)
{
  return mission4_near_poly(mission4_strasbourg_vx, mission4_strasbourg_vy, 6,
                            x, y, MISSION4_GEOFENCE_BUFFER);
}

/**
 * mission4_in_valkenburg / _safe - FZ1-FZ7 Valkenburg flyzone, converted
 * from conf/flight_plans/MS/valkenburg_flyzone.kml into the local frame of
 * mission4.xml (lat0 52.1681239, lon0 4.4124784 - the same anchor
 * mission1.xml uses, so waypoints are directly comparable between the two).
 * 890 x 804 m, 38.1 ha, against Strasbourg's narrower hexagon: every bearing
 * at a 130 m drop run-in clears 107-206 m here, so the approach is not
 * direction-constrained the way Strasbourg's is.
 */
static const float mission4_valkenburg_vx[7] = {-13.546f,  96.380f, 275.267f, 603.682f, 876.109f, 775.741f, 310.089f};
static const float mission4_valkenburg_vy[7] = {138.715f, -189.677f, -258.695f, -1.547f, 207.733f, 545.031f, 344.656f};

static inline bool mission4_in_valkenburg(float x, float y)
{
  return mission4_in_poly(mission4_valkenburg_vx, mission4_valkenburg_vy, 7, x, y);
}

static inline bool mission4_in_valkenburg_safe(float x, float y)
{
  return mission4_near_poly(mission4_valkenburg_vx, mission4_valkenburg_vy, 7,
                            x, y, MISSION4_GEOFENCE_BUFFER);
}

#endif /* NAV_MISSION4_GEOFENCE_H */
