/*
 * Copyright (C) 2024 Simon Cajagi <Cajagi@student.tudelft.nl>
 *
* This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/** @file "modules/deviation_from_wp/deviation_from_wp.c"
 * This module logs the distance from a waypoint to help analyse how turbulence affects NAV mode accuracy in different wind fields.
 */

#include "std.h"
#include "deviation_from_wp.h"
#include "state.h"
#include "generated/flight_plan.h"
#include "firmwares/rotorcraft/navigation.h"
#include "modules/datalink/telemetry.h"

// Settings
float wp_e;
float wp_n;
float wp_u;
bool send_data;
bool automated_test;

// Grid and exclusion zone parameters
float step_n = 0.25;  // Step size in north direction
float step_u = 0.25;  // Step size in up direction
float min_n = 0.5;   // Min limit in north direction
float max_n = 5.5;   // Max limit in north direction
float min_u = 1.0;   // Min limit in up direction
float max_u = 3.0;   // Max limit in up direction

// Exclusion zone lines (defined by two vertices each)
float ex1[2] = {5.5, 0.5};
float ex2[2] = {0.5, 3.0};

// Other variables
struct EnuCoor_f waypoint_pos;
struct EnuCoor_f current_pos;
bool current_automated_test = false;
bool within_margin = false;
uint32_t timer_start;
uint32_t recording_time = 15000; // 15 seconds

// Telemetry function
static void telemetry_send_debug_vect(struct transport_tx *trans, struct link_device *dev);

// Utility functions
bool point_below_line(float px, float py, float *v1, float *v2);
void start_timer();
bool check_timer(uint32_t duration);
void move_to_next_waypoint();

// Automated test function
void automated_test_function();
void automated_test_function_init();

void init_deviation_from_wp(void){
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_DEBUG_VECT, telemetry_send_debug_vect);
    // Update waypoint position
    waypoint_pos.x = wp_e;
    waypoint_pos.y = wp_n;
    waypoint_pos.z = wp_u;
    waypoint_set_enu(WP_DEVI, &waypoint_pos);
    if (automated_test) {
        automated_test_function();
    }
}

void periodic_deviation_from_wp(void){
    // Update current position
    current_pos.x = stateGetPositionEnu_f()->x;
    current_pos.y = stateGetPositionEnu_f()->y;
    current_pos.z = stateGetPositionEnu_f()->z;
    if (!current_automated_test && automated_test) {
        automated_test_function_init();
    }
    if (automated_test) {
        automated_test_function();
    }
    else if (!(wp_e == waypoint_pos.x && wp_n == waypoint_pos.y && wp_u == waypoint_pos.z)){
        // Check if waypoint position has changed
        // Update waypoint position
        waypoint_pos.x = wp_e;
        waypoint_pos.y = wp_n;
        waypoint_pos.z = wp_u;
        waypoint_set_enu(WP_DEVI, &waypoint_pos);
    }
}

static void telemetry_send_debug_vect(struct transport_tx *trans, struct link_device *dev) {
  float data[5];
  // Telemetry callback function
  if (send_data == false) {
    // To prevent recording data when module is not active
    data = {0, 0, 0, 0, 0};
  } else {
    data = {waypoint_pos.y, waypoint_pos.z, current_pos.x - waypoint_pos.x, current_pos.y - waypoint_pos.y, current_pos.z - waypoint_pos.z};
  }
  pprz_msg_send_DEBUG_VECT(trans, dev, AC_ID, strlen("wp_n, wp_u, err_e, err_n, err_u"), "wp_n, wp_u, err_e, err_n, err_u", 5, &data);
}

bool point_below_line(float px, float py, float *v1, float *v2) {
    // Calculate the slope (m) and y-intercept (b) of the line y = mx + b
    float m = (v2[1] - v1[1]) / (v2[0] - v1[0]);
    float b = v1[1] - m * v1[0];
    
    // Calculate the y-value of the line at px
    float line_y_at_px = m * px + b;
    
    // Check if the point (px, py) is below or on the line
    return py <= line_y_at_px;
}

void start_timer() {
    timer_start = get_sys_time_usec();
}

bool check_timer(uint32_t duration) {
    fprint("Timer: %d\n", get_sys_time_usec() - timer_start);
    return (get_sys_time_usec() - timer_start) >= duration;
}

void move_to_next_waypoint() {
    // Move to the next waypoint in the grid
    wp_n += step_n;
    if (wp_n > max_n) {
        wp_n = min_n;
        wp_u += step_u;
        if (wp_u > max_u) {
            // Automated test is finished
            automated_test = false;
            current_automated_test = false;
            send_data = false;
        }
    }
    // Check exclusion zone using lines
    while (point_below_line(wp_n, wp_u, ex1, ex2)) {
        wp_n += step_n;
        if (wp_n > max_n) {
            wp_n = min_n;
            wp_u += step_u;
            if (wp_u > max_u) {
                // Automated test is finished
                automated_test = false;
                current_automated_test = false;
                send_data = false;
            }
        }
    }
    waypoint_pos.x = wp_e;
    waypoint_pos.y = wp_n;
    waypoint_pos.z = wp_u;
    waypoint_set_enu(WP_DEVI, &waypoint_pos);
    within_margin = false;
}

void automated_test_function() {
    if (within_margin) {
        if (check_timer(5000)) { // Wait 5 seconds within margin before recording
            send_data = true;
            if (check_timer(5000 + recording_time)) { // Wait another 15 seconds while recording
                send_data = false;
                move_to_next_waypoint();
                start_timer();
            }
        }
    } else {
        if (fabs(current_pos.y - wp_n) <= 0.05 && fabs(current_pos.z - wp_u) <= 0.05) {
            within_margin = true;
            start_timer();
        }
    }
}

void automated_test_function_init() {
    // Reset variables
    current_automated_test = true;
    wp_e = 0;
    wp_n = min_n;
    wp_u = min_u;
    // Check exclusion zone using lines
    while (point_below_line(wp_n, wp_u, ex1, ex2)) {
        wp_n += step_n;
        if (wp_n > max_n) {
            wp_n = min_n;
            wp_u += step_u;
            if (wp_u > max_u) {
                // Automated test is finished
                automated_test = false;
                current_automated_test = false;
                send_data = false;
            }
        }
    }
    waypoint_pos.x = wp_e;
    waypoint_pos.y = wp_n;
    waypoint_pos.z = wp_u;
    waypoint_set_enu(WP_DEVI, &waypoint_pos);
    within_margin = false;
    send_data = false;
    start_timer();
}
