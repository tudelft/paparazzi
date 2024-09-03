/*
 * Copyright (C) Simon Cajagi
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/nld/soaring_pos_control.c"
 * @author Simon Cajagi
 */

#include "modules/nld/soaring_pos_control.h"
#include "firmwares/rotorcraft/navigation.h"
#include "generated/airframe.h"
#include "state.h"
#include "modules/core/abi.h"
#include <time.h>
#include <stdio.h>
#include "generated/flight_plan.h"

void moveWaypointOnLine(uint8_t waypoint, float altitude);
void moveWaypoint(uint8_t waypoint, struct EnuCoor_f *new_coor);
void initHeadingWaypoint(void);
float distance(struct EnuCoor_f *pt1, struct EnuCoor_f *pt2);

// Settings
float charger_e = 0;
float charger_n = 0;
float charger_u = 0;
float charger_heading = 0;
float charger_angle_to_vertical = 0;
float max_altitude = 10;
float min_altitude = 1;
float waypoint_radius = 0.3;
float altitude_step = 0.2;
bool descending = false;

// Global variables
struct EnuCoor_f current_position;
struct EnuCoor_f current_waypoint;
struct EnuCoor_f charger_position;

void soaring_pos_control_init(void)
{
  charger_position.x = charger_e;
  charger_position.y = charger_n;
  charger_position.z = charger_u;
  return;
}

void soaring_pos_control_periodic(void)
{
  // Get current position
  current_position.x = stateGetPositionEnu_f()->x;
  current_position.y = stateGetPositionEnu_f()->y;
  current_position.z = stateGetPositionEnu_f()->z;

  // Get position of waypoint
  current_waypoint.x = waypoint_get_enu_f(WP_GOAL)->x;
  current_waypoint.y = waypoint_get_enu_f(WP_GOAL)->y;
  current_waypoint.z = waypoint_get_enu_f(WP_GOAL)->z;

  // Update descending flag
  if (current_position.z - charger_position.z <= min_altitude) {
    descending = false;
  }
  //TODO: Add pusher motor actuator value > low value to the condition above

  // Check if within radius of waypoint, move waypoint
  if (distance(&current_position, &current_waypoint) <= waypoint_radius && descending) {
      moveWaypointOnLine(WP_GOAL, current_position.z - altitude_step);
  }

  return;
}

void startLineFollowing(void)
{
  descending = true;
  // Set the first waypoint at the top of the line
  moveWaypointOnLine(WP_GOAL, max_altitude);
  // Move heading waypoint in correct spot
  initHeadingWaypoint();
  nav_set_heading_towards_waypoint(WP_HEADING);
}

/*
 * Moves waypoint down by dAltitude along the charging line
 */
void moveWaypointOnLine(uint8_t waypoint, float altitude)
{
  struct EnuCoor_f new_coor;
  new_coor.x = (altitude - charger_position.z) * tanf(charger_angle_to_vertical) * sinf(charger_heading) + charger_position.x;  // East
  new_coor.y = (altitude - charger_position.z) * tanf(charger_angle_to_vertical) * cosf(charger_heading) + charger_position.y;  // North
  new_coor.z = altitude;                                                                                                        // Up
  moveWaypoint(waypoint, &new_coor);
}

void initHeadingWaypoint(void)
{
  // Created waypoint for heading control, 100m in the correct direction
  struct EnuCoor_f new_coor;
  new_coor.x = 100.0 * sinf(charger_heading) + charger_e;
  new_coor.y = 100.0 * cosf(charger_heading) + charger_n;
  moveWaypoint(WP_HEADING, &new_coor);
}

/*
 * Sets waypoint to the coordinates of 'new_coor', print a msg
 */
void moveWaypoint(uint8_t waypoint, struct EnuCoor_f *new_coor)
{
  // PRINT("Moving waypoint %d to x:%f y:%f\n", waypoint, POS_FLOAT_OF_BFP(new_coor->x),
  //               POS_FLOAT_OF_BFP(new_coor->y));
  waypoint_set_enu(waypoint, new_coor);
}

float distance(struct EnuCoor_f *pt1, struct EnuCoor_f *pt2)
{
  return sqrtf(powf(pt1->x - pt2->x, 2) + powf(pt1->y - pt2->y, 2) + powf(pt1->z - pt2->z, 2));
}
