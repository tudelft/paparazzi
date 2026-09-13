#pragma once
struct TestWaypoint { float east, north, altitude; };
extern struct TestWaypoint test_waypoints[2];
extern float test_altitude, test_target_altitude, test_preclimb;
#define WaypointX(index) test_waypoints[index].east
#define WaypointY(index) test_waypoints[index].north
#define WaypointAlt(index) test_waypoints[index].altitude
#define GetPosAlt() test_altitude
#define NavVerticalAltitudeMode(altitude, preclimb) do { test_target_altitude = (altitude); test_preclimb = (preclimb); } while (0)