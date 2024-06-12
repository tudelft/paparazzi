#include "landing_path_gen.h"
#include <pthread.h>
#include "MATLAB_generated_files/path_optimizer_fcn.h"
#include "MATLAB_generated_files/path_optimizer_fcn_types.h"
#include "MATLAB_generated_files/rt_nonfinite.h"
#include <string.h>


void main() {

double landing_time_init = 0.1; 
double time_sample_while = 0.2; 
double max_time_of_landing_seconds = 10; 

// Define initial position and speed of the UAV in the NED frame
double P0[3] = {-10, 10, 5}; 
double V0[3] = {4, 3, -1};
double A0[3] = {0, 0, 0};

double coeffs_ship_prediction[18] = {0, 0, 0, 0, 4, 0,
                                     0, 0, 0, 0, 3, 10,
                                     0, 0, 0, 0, 0, 0};

double pos_gain = 1;
double speed_gain = 1e-1;
double acc_gain = 1e-1; 

double num_points = 10;

double v_max[3] = {15.0, 5.0, 3};
double v_min[3] = {-5.0, -5.0, -3};

// Define maximum and minimum speeds (in meters per second) and accelerations (in meters per second squared)
double a_max[3] = {10.0, 5.0, 5};
double a_min[3] = {-5.0, -5.0, -5};

double verbose = 1;
double plot_results = 1;

double num_points_plot = 10; 
double optimal_coeffs[18]; 
double landing_time; 

path_optimizer_fcn(coeffs_ship_prediction, landing_time_init, time_sample_while, 
    P0, V0, A0, v_max, v_min, a_max, a_min, num_points, num_points_plot, pos_gain, 
    speed_gain, acc_gain, max_time_of_landing_seconds, verbose, optimal_coeffs, 
    &landing_time);

}
