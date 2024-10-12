#include <stdio.h>

//Do not include the high power beacon, as suggested in the documentation. 
#define INCLUDE_HIGH_POW_BEC_IN_SIXDOF_MAP 0

//Define the number of beacons
#define N_BEACON 5

struct  __attribute__((__packed__)) register_sixdof_packet {
    //Motor command
    double timestamp_position;
	float x_abs_pos;
	float y_abs_pos;
    float z_abs_pos;
	float relative_phi_rad;
	float relative_theta_rad;
    float relative_psi_rad;
    float x_abs_pos_var;
    float y_abs_pos_var;
    float z_abs_pos_var;
    float phi_rad_var;
    float theta_rad_var;
    float psi_rad_var;
};

struct  __attribute__((__packed__)) register_beacon_packet {
    double timestamp_beacon[N_BEACON];
    int beacon_id[N_BEACON];
    float x_body_pos_beacon[N_BEACON];
    float y_body_pos_beacon[N_BEACON];
    float z_body_pos_beacon[N_BEACON];
};

struct __attribute__((__packed__)) fov_report_packet {
    double timestamp_fow[N_BEACON];
    int fov_beacon_id[N_BEACON];
    int sensor_seen_count[N_BEACON];
};

struct __attribute__((__packed__)) register_rel_angle_packet {
    int beacon_id[N_BEACON];
    double timestamp_rel_angle[N_BEACON];
    float x_angle_rad[N_BEACON];
    float z_angle_rad[N_BEACON];
    float intensity[N_BEACON];
    float width[N_BEACON];
};
