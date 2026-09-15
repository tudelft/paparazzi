#pragma once
#define NB_WAYPOINT 2
extern float flight_altitude;
extern float test_nav_pitch, test_nav_throttle;
extern bool test_nav_route;
#define NavVerticalAutoThrottleMode(pitch) (test_nav_pitch = (pitch))
#define NavVerticalThrottleMode(throttle) (test_nav_throttle = (throttle))
#define NavAttitude(roll) ((void)(roll), test_nav_route = false)
#define NavApproachingFrom(to, from, time) ((void)(to), (void)(from), (void)(time), false)
#define NavSegment(from, to) ((void)(from), (void)(to), test_nav_route = true)
struct TestWaypoint { float east, north, altitude; };
extern struct TestWaypoint test_waypoints[2];
extern float test_altitude, test_target_altitude, test_preclimb;
#define WaypointX(index) test_waypoints[index].east
#define WaypointY(index) test_waypoints[index].north
#define WaypointAlt(index) test_waypoints[index].altitude
#define GetPosAlt() test_altitude
#define NavVerticalAltitudeMode(altitude, preclimb) do { test_target_altitude = (altitude); test_preclimb = (preclimb); } while (0)