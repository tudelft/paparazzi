/**
 * @file "modules/nld/soaring_pos_control.h"
 * Soaring pos controller for in air charging of the rotating wing drone
 */

#include "std.h"

// settings
extern float charger_e;
extern float charger_n;
extern float charger_u;
extern float charger_heading;
extern float charger_angle_to_vertical;
extern float max_altitude;
extern float min_altitude;
extern float waypoint_radius;
extern float altitude_step;
extern bool descending;

// functions
extern void soaring_pos_control_init(void);
extern void soaring_pos_control_periodic(void);
extern void startLineFollowing(void);