#include <stdio.h>


struct  __attribute__((__packed__)) register_sixdof_packet {
    //Motor command
    double timestamp_position;
	float x_body_pos;
	float y_body_pos;
    float z_body_pos;
	float relative_phi_rad;
	float relative_theta_rad;
    float relative_psi_rad;
    float x_body_pos_var;
    float y_body_pos_var;
    float z_body_pos_var;
    float phi_rad_var;
    float theta_rad_var;
    float psi_rad_var;
};