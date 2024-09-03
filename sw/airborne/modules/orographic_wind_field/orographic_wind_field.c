/*
 * Copyright (C) 2023 Dennis van Wijngaarden <D.C.vanWijngaarden@tudelft.nl>
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

/** @file "modules/orographic_wind_field/orographic_wind_field.c"
 * @author Simon Cajagi <Cajagi@student.tudelft.nl>
 * This module controls the winds in the NPS simulation based on the current position of the aircraft, to simulate updrafts above a dune.
 * The wind field is based on a CSV file storing the CFD simulated flow field over a slope in the wind tunnel.
 * This data is extruded to form an infinite slope along the east-west axis.
 * The wind blows from the north towards the south.
 * Outside of the are described by the CSV file, the wind speed is set to the base wind speed, still coming from the north.
 */

#include "std.h"
#include "/home/simiken1234/paparazzi/sw/simulator/nps/nps_atmosphere.h"
#include "orographic_wind_field.h"
#include "state.h"

#define MAX_LINE_LENGTH 128
#define MAX_ENTRIES 16384

// Settings
float base_wind_speed = 0.0;
uint8_t desired_slope_angle = 0;
uint8_t desired_turbulence_severity = 0;

// Other variables
typedef struct {
    double x, z, v_x, v_z, v_mag;
} WindData;

typedef struct {
    double x, z;
} Position;

static WindData wind_data[MAX_ENTRIES];
static int data_count = 0;
static double csv_base_wind_speed = 0;
static uint8_t loaded_slope_angle = 0;
static uint8_t current_turbulence_severity = 0;

static void get_filename(uint8_t desired_slope_angle, char *filename);
static int read_csv(const char *filename, WindData data[]);
static void interpolate_wind_speed(Position pos, double *v_x, double *v_z);
static void set_wind_speed(double v_x, double v_z);

void init_orographic_wind_field(void) {
    char filename[256];
    get_filename(desired_slope_angle, filename);
    data_count = read_csv(filename, wind_data);
    if (data_count < 0) {
        fprintf(stderr, "Failed to initialize wind field\n");
        exit(1);
    }
    loaded_slope_angle = desired_slope_angle;
    nps_atmosphere_set_turbulence_severity(desired_turbulence_severity);
    current_turbulence_severity = desired_turbulence_severity;
}

void periodic_orographic_wind_field(void) {
    if (desired_slope_angle != loaded_slope_angle) {
        init_orographic_wind_field();
    }
    if (desired_turbulence_severity != current_turbulence_severity) {
        nps_atmosphere_set_turbulence_severity(desired_turbulence_severity);
        current_turbulence_severity = desired_turbulence_severity;
    }
    struct NedCoor_f *position = stateGetPositionNed_f();
    //printf("Position: N = %f, E = %f, D = %f\n", position->x, position->y, position->z);
    Position pos = {position->x, -position->z};
    double v_x, v_z;
    interpolate_wind_speed(pos, &v_x, &v_z);
    //printf("Wind speed: %f, %f\n", v_x, v_z);
    set_wind_speed(v_x, v_z);
}

static void get_filename(uint8_t desired_slope_angle, char *filename) {
    const char *directory = "/home/simiken1234/paparazzi/sw/airborne/modules/orographic_wind_field/wind_field_csvs/";
    switch (desired_slope_angle) {
        case 0:
            strcpy(filename, directory);
            strcat(filename, "2d_ojf_213_8.5.csv");
            csv_base_wind_speed = 8.5;
            printf("Slope angle 21.3 deg. Original base wind speed 8.5 m/s\n");
            break;
        case 1:
            strcpy(filename, directory);
            strcat(filename, "2d_ojf_232_10.0.csv");
            csv_base_wind_speed = 10.0;
            printf("Slope angle 23.2 deg. Original base wind speed 10.0 m/s\n");
            break;
        case 2:
            strcpy(filename, directory);
            strcat(filename, "2d_ojf_252_8.0.csv");
            csv_base_wind_speed = 8.0;
            printf("Slope angle 25.2 deg. Original base wind speed 8.0 m/s\n");
            break;
        default:
            fprintf(stderr, "Invalid slope angle\n");
            exit(1);
    }
}

static int read_csv(const char *filename, WindData data[]) {
    FILE *file = fopen(filename, "r");
    if (!file) {
        perror("Failed to open file");
        return -1;
    }

    char line[MAX_LINE_LENGTH];
    int index = 0;
    if (!fgets(line, MAX_LINE_LENGTH, file)) { // Skip header
        fclose(file);
        return -1;
    }
    while (fgets(line, MAX_LINE_LENGTH, file)) {
        sscanf(line, "%lf,%lf,%lf,%lf,%lf", &data[index].x, &data[index].z, &data[index].v_x, &data[index].v_z, &data[index].v_mag);
        index++;
    }
    fclose(file);
    return index;
}

static void interpolate_wind_speed(Position pos, double *v_x, double *v_z) {
    if (pos.x < 0 || pos.x > 5.95 || pos.z < 0 || pos.z > 3.3) {
        *v_x = base_wind_speed;
        *v_z = 0;
        return;
    }

    double scale_factor = base_wind_speed / csv_base_wind_speed;

    int x0_index = (int)(pos.x / 0.05);
    int x1_index = x0_index + 1;
    int z0_index = (int)(pos.z / 0.05);
    int z1_index = z0_index + 1;

    double x0 = x0_index * 0.05;
    double x1 = x1_index * 0.05;
    double z0 = z0_index * 0.05;
    double z1 = z1_index * 0.05;

    WindData q11 = wind_data[z0_index * 120 + x0_index];
    WindData q21 = wind_data[z0_index * 120 + x1_index];
    WindData q12 = wind_data[z1_index * 120 + x0_index];
    WindData q22 = wind_data[z1_index * 120 + x1_index];

    double t = (pos.x - x0) / (x1 - x0);
    double u = (pos.z - z0) / (z1 - z0);

    double v_x_interp_x0 = q11.v_x + t * (q21.v_x - q11.v_x);
    double v_x_interp_x1 = q12.v_x + t * (q22.v_x - q12.v_x);
    double v_z_interp_x0 = q11.v_z + t * (q21.v_z - q11.v_z);
    double v_z_interp_x1 = q12.v_z + t * (q22.v_z - q12.v_z);

    *v_x = (v_x_interp_x0 + u * (v_x_interp_x1 - v_x_interp_x0)) * scale_factor;
    *v_z = (v_z_interp_x0 + u * (v_z_interp_x1 - v_z_interp_x0)) * scale_factor;
}

static void set_wind_speed(double v_x, double v_z) {
    nps_atmosphere_set_wind_ned(-v_x, 0, -v_z);
    // -v_x because in the simulation, the wind flows from positive x towards negative x
    // -v_z because v_z is positive up but the command is sent in NED coordinates
}