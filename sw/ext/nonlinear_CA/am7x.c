#include "am7x.h"
#include <pthread.h>
#include "MATLAB_generated_files/Nonlinear_controller_w_ail_basic_aero_outer_loop.h"
#include "MATLAB_generated_files/Nonlinear_controller_w_ail_basic_aero_inner_loop.h"
#include "MATLAB_generated_files/compute_acc_control_rf_basic.h"
#include "MATLAB_generated_files/compute_acc_nonlinear_control_rf_w_ailerons_v2_new_aero.h"
#include "MATLAB_generated_files/Nonlinear_controller_w_ail_new_aero_outer_loop.h"
#include "MATLAB_generated_files/Nonlinear_controller_w_ail_new_aero_inner_loop.h"
#include "MATLAB_generated_files/rt_nonfinite.h"
#include <string.h>
#include <glib.h>
#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>
#include "filters/low_pass_filter.h"

// #define TEST_CONTROLLER

#define USE_ARUCO
#define USE_SIXDOF

//Variable for current acceleration filtering:
Butterworth2LowPass modeled_accelerations_filter[6];
Butterworth2LowPass body_rates_filter[3], u_in_filter[NUM_ACT_IN_U_IN]; 
Butterworth2LowPass airspeed_filter, flight_path_angle_filter, psi_filter; 
//1-exp(-cutoff_frequency/sampling_frequency)
float filter_cutoff_first_order_pqr = filter_cutoff_first_order_pqr_init;
float pqr_first_order_filter_tau;
float p_filtered_ec = 0.0f, q_filtered_ec = 0.0f, r_filtered_ec = 0.0f; 

int filter_cutoff_frequency;
float tau_indi;

double u_init_outer[NUM_ACT_IN_U_IN];
double u_init_inner[NUM_ACT_IN_U_IN_INNER];

struct data_in_optimizer mydata_in_optimizer;
struct am7_data_out myam7_data_out;
struct am7_data_in myam7_data_in;
float extra_data_in[255];
float extra_data_out[255];
uint16_t buffer_in_counter;
char am7_msg_buf_in[sizeof(struct am7_data_in)*2]  __attribute__((aligned));
char am7_msg_buf_out[sizeof(struct am7_data_out)]  __attribute__((aligned));
uint32_t received_packets = 0, received_packets_tot = 0;
uint32_t sent_packets = 0, sent_packets_tot = 0;
uint32_t missed_packets = 0;
uint16_t sent_msg_id = 0, received_msg_id = 0;
int serial_port;
int serial_port_tf_mini;
struct timeval current_time, last_time_print_statistics, last_time_am7_msg_sent, aruco_time, sixdof_time, starting_time_program_execution;
struct timeval time_last_outer_loop, time_last_inner_loop, time_last_filt;

struct outer_loop_output myouter_loop_output;

pthread_mutex_t mutex_am7, mutex_optimizer_input, mutex_outer_loop_output;

int verbose_sixdof = 0;
int verbose_connection = 0;
int verbose_outer_loop = 0; 
int verbose_inner_loop = 0;
int verbose_runtime = 0; 
int verbose_data_in = 0; 
int verbose_submitted_data = 0; 
int verbose_ivy_bus = 0; 
int verbose_aruco = 1; 
int verbose_compute_accel = 0; 
int verbose_filters = 0; 

//LIDAR VARIABLES
int16_t lidar_dist_cm = -1; 
int16_t lidar_signal_strength = -1; 

//IYY BUS COM VARIABLES: 
double time_last_heartbeat_ivy_out = 0.0;

//COMPUTER VISION 
struct marker_detection_t aruco_detection;
int aruco_system_status = 0;
double last_ping_time_aruco = 0.0;
pthread_mutex_t mutex_aruco;

//SIXDOF VARIABLES: [default]
int desired_sixdof_mode = 3; //1 -->rel beacon pos; 2 -->rel beacon angle; 3-->sixdof mode. Default is 1. 
int beacon_tracking_id = 1640; 
float max_tolerance_variance_sixdof = 1.0f; //meters
int current_sixdof_mode; 
struct marker_detection_t sixdof_detection;
pthread_mutex_t mutex_sixdof;
double last_ping_time_sixdof = 0.0;

// Transpose an array from body reference frame to earth reference frame
void from_body_to_earth(float *out_array, const float *in_array, float Phi, float Theta, float Psi){

    float R_be_matrix[3][3];

    // Compute the elements of the matrix
    R_be_matrix[0][0] = cos(Theta) * cos(Psi);
    R_be_matrix[0][1] = -cos(Phi) * sin(Psi) + sin(Phi) * sin(Theta) * cos(Psi);
    R_be_matrix[0][2] = sin(Phi) * sin(Psi) + cos(Phi) * sin(Theta) * cos(Psi);
    R_be_matrix[1][0] = cos(Theta) * sin(Psi);
    R_be_matrix[1][1] = cos(Phi) * cos(Psi) + sin(Phi) * sin(Theta) * sin(Psi);
    R_be_matrix[1][2] = -sin(Phi) * cos(Psi) + cos(Phi) * sin(Theta) * sin(Psi);
    R_be_matrix[2][0] = -sin(Theta);
    R_be_matrix[2][1] = sin(Phi) * cos(Theta);
    R_be_matrix[2][2] = cos(Phi) * cos(Theta);

    // Do the multiplication between the input array and the rotation matrix:
    for (int i = 0; i < 3; i++) {
        out_array[i] = 0.; // Initialize output array element to zero
        for (int j = 0; j < 3; j++) {
            out_array[i] += R_be_matrix[i][j] * in_array[j];
        }
    }
}

void readLiDAR(){
  // Data Format for Benewake TFmini
  // ===============================
  // 9 bytes total per message:
  // 1) 0x59
  // 2) 0x59
  // 3) Dist_L (low 8bit)
  // 4) Dist_H (high 8bit)
  // 5) Strength_L (low 8bit)
  // 6) Strength_H (high 8bit)
  // 7) Reserved bytes
  // 8) Original signal quality degree
  // 9) Checksum parity bit (low 8bit), Checksum = Byte1 + Byte2 +...+Byte8. This is only a low 8bit though

  if(serialDataAvail(serial_port_tf_mini)>=9){ // When at least 9 bytes of data available (expected number of bytes for 1 signal), then read
    if((0x59 == serialGetchar (serial_port_tf_mini)) && (0x59 == serialGetchar (serial_port_tf_mini))) // byte 1 and byte 2
    {
      unsigned int t1 = serialGetchar (serial_port_tf_mini); // byte 3 = Dist_L
      unsigned int t2 = serialGetchar (serial_port_tf_mini); // byte 4 = Dist_H

      t2 <<= 8;
      t2 += t1;
      lidar_dist_cm = (int16_t) t2;

      t1 = serialGetchar (serial_port_tf_mini); // byte 5 = Strength_L
      t2 = serialGetchar (serial_port_tf_mini); // byte 6 = Strength_H

      t2 <<= 8;
      t2 += t1;

      lidar_signal_strength = (int16_t) t2;

      for(int i=0; i<3; i++) serialGetchar (serial_port_tf_mini); // byte 7, 8, 9 are ignored
    }
  }
}

void am7_init(){
  /////////////////////////////////////////////////////////////////////////////////////// Init serial lines
  //Init serial port for the communication
  if ((serial_port = serialOpen ("/dev/ttyS4", BAUDRATE_AM7)) < 0){
    fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno)) ;
  }
  if (wiringPiSetup () == -1){
    fprintf (stdout, "Unable to start wiringPi: %s\n", strerror (errno)) ;
  }

  //Init the lidar: 
  if ((serial_port_tf_mini = serialOpen ("/dev/ttyS3", BAUDRATE_TF_MINI)) < 0){
    fprintf (stderr, "Unable to open serial_port_tf_mini device: %s\n", strerror (errno)) ;
  }
  if (wiringPiSetup () == -1){
    fprintf (stdout, "Unable to start wiringPi: %s\n", strerror (errno)) ;
  }
  // Set to Standard Output mode the lidar
  serialPutchar(serial_port_tf_mini, 0x42);
  serialPutchar(serial_port_tf_mini, 0x57);
  serialPutchar(serial_port_tf_mini, 0x02);
  serialPutchar(serial_port_tf_mini, 0x00);
  serialPutchar(serial_port_tf_mini, 0x00);
  serialPutchar(serial_port_tf_mini, 0x00);
  serialPutchar(serial_port_tf_mini, 0x01);
  serialPutchar(serial_port_tf_mini, 0x06);

  /////////////////////////////////////////////////////////////////////////////////// Init the extra messages value 
  for(int i = 0; i < (sizeof(extra_data_in)/sizeof(float)); i++ ){
    extra_data_in[i] = 0.f;
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////// Init filters:
  filter_cutoff_frequency = (int)filter_cutoff_frequency_init;

  tau_indi = 1.0f / (filter_cutoff_frequency);

  for (int i = 0; i < 6; i++) {
    init_butterworth_2_low_pass(&modeled_accelerations_filter[i], tau_indi, refresh_time_filters, 0.0);
  }

  for (int i = 0; i < 3; i++) {
    init_butterworth_2_low_pass(&body_rates_filter[i], tau_indi, refresh_time_filters, 0.0);
  }

  for (int i = 0; i < NUM_ACT_IN_U_IN; i++) {
    init_butterworth_2_low_pass(&u_in_filter[i], tau_indi, refresh_time_filters, 0.0);
  }

  init_butterworth_2_low_pass(&airspeed_filter, tau_indi, refresh_time_filters, 0.0);
  init_butterworth_2_low_pass(&flight_path_angle_filter, tau_indi, refresh_time_filters, 0.0);
  init_butterworth_2_low_pass(&psi_filter, tau_indi, refresh_time_filters, 0.0);

  ///////////////////////////////////////////////////////////////////////////////////////////// Init watchdogs timers: 
  gettimeofday(&current_time, NULL);
  gettimeofday(&last_time_am7_msg_sent, NULL);
  gettimeofday(&aruco_time, NULL);
  gettimeofday(&sixdof_time, NULL);
  gettimeofday(&time_last_filt, NULL);
  gettimeofday(&starting_time_program_execution, NULL);

}

void writing_routine(){

    float extra_data_out_copy[255];
    struct am7_data_out myam7_data_out_copy;
    pthread_mutex_lock(&mutex_am7);
    memcpy(&myam7_data_out_copy, &myam7_data_out, sizeof(struct am7_data_out));
    memcpy(&extra_data_out_copy, &extra_data_out, sizeof(extra_data_out));
    pthread_mutex_unlock(&mutex_am7);
    myam7_data_out_copy.rolling_msg_out_id = sent_msg_id;
    myam7_data_out_copy.rolling_msg_out = extra_data_out_copy[sent_msg_id];
    
    //Add the lidar message to the data out value, no need to lock the mutex for this 
    //value as they run sequentially on the same thread: 
    myam7_data_out_copy.lidar_value_cm = lidar_dist_cm;
    myam7_data_out_copy.lidar_strength = lidar_signal_strength;

    uint8_T *buf_out = (uint8_T *)&myam7_data_out_copy;
    //Calculating the checksum
    uint8_T checksum_out_local = 0;

    for(uint16_T i = 0; i < sizeof(struct am7_data_out) - 1; i++){
      checksum_out_local += buf_out[i];
    }
    myam7_data_out_copy.checksum_out = checksum_out_local;
    //Send bytes
    serialPutchar(serial_port, START_BYTE);
    for(int i = 0; i < sizeof(struct am7_data_out); i++){
      serialPutchar(serial_port, buf_out[i]);
    }
    sent_packets++;
    sent_packets_tot++;
    gettimeofday(&last_time_am7_msg_sent, NULL);

    //Increase the counter to track the sending messages:
    sent_msg_id++;
    if(sent_msg_id == 255){
        sent_msg_id = 0;
    }
}

void reading_routine(){
    uint8_T am7_byte_in;
    if(serialDataAvail(serial_port) > 0){
      am7_byte_in = serialGetchar (serial_port);      
      if( (am7_byte_in == START_BYTE) || (buffer_in_counter > 0)){
        am7_msg_buf_in[buffer_in_counter] = am7_byte_in;
        buffer_in_counter ++;  
      }
      if (buffer_in_counter > sizeof(struct am7_data_in)){
        buffer_in_counter = 0;
        uint8_T checksum_in_local = 0;
        for(uint16_T i = 1; i < sizeof(struct am7_data_in) ; i++){
          checksum_in_local += am7_msg_buf_in[i];
        }
        if(checksum_in_local == am7_msg_buf_in[sizeof(struct am7_data_in)]){
          //Parse message
          pthread_mutex_lock(&mutex_am7);
          memcpy(&myam7_data_in, &am7_msg_buf_in[1], sizeof(struct am7_data_in));
          received_msg_id = myam7_data_in.rolling_msg_in_id;
          extra_data_in[received_msg_id] = myam7_data_in.rolling_msg_in;
          pthread_mutex_unlock(&mutex_am7); 

          received_packets++;
          received_packets_tot++;
        }
        else {
          missed_packets++;
        }
      }
    }
    else{
      usleep(5);
    }
} 

void send_states_on_ivy(){
    //Copy received message from UAV autopilot 
    struct am7_data_in myam7_data_in_copy;
    pthread_mutex_lock(&mutex_am7);   
    memcpy(&myam7_data_in_copy, &myam7_data_in, sizeof(struct am7_data_in));
    pthread_mutex_unlock(&mutex_am7);

    //Send messages over ivy bus for the python thread: 
    if(verbose_ivy_bus) printf("Sent received message from UAV on ivy bus\n");
    IvySendMsg("1 ROTORCRAFT_FP  %d %d %d  %d %d %d  %d %d %d  %d %d %d  %d %d %d",

            (int32_t) (myam7_data_in_copy.UAV_NED_pos_y/0.0039063),
            (int32_t) (myam7_data_in_copy.UAV_NED_pos_x/0.0039063),
            (int32_t) (-myam7_data_in_copy.UAV_NED_pos_z/0.0039063),

            (int32_t) (-1/0.0000019),
            (int32_t) (-1/0.0000019),
            (int32_t) (-1/0.0000019),

            (int32_t) ( (myam7_data_in_copy.phi_state_int*0.01) /0.0139882),
            (int32_t) ( (myam7_data_in_copy.theta_state_int*0.01) /0.0139882),
            (int32_t) ( (myam7_data_in_copy.psi_state_int*0.01) /0.0139882),

            (int32_t) (-1/0.0039063),
            (int32_t) (-1/0.0039063),
            (int32_t) (-1/0.0039063),

            (int32_t) (-1/0.0039063),
            (int32_t) (-1/0.0039063),
            (uint16_t) (-1/0.0039063));

    //Send heartbeat message over ivy bus every second:
    struct timespec ts;
    clock_gettime(CLOCK_BOOTTIME, &ts);
    //If time is greater than 1 second, send heartbeat message
    if(ts.tv_sec + ts.tv_nsec*1e-9 - time_last_heartbeat_ivy_out > 1.0){
      //Save the current timestamp
      time_last_heartbeat_ivy_out = ts.tv_sec + ts.tv_nsec*1e-9;
      IvySendMsg("HEARTBEAT_AM7 %f", time_last_heartbeat_ivy_out);
    }

}

void send_receive_am7(){

  //Reading routine: 
  reading_routine();
  //Writing routine with protection to not exceed desired packet frequency:
  gettimeofday(&current_time, NULL); 
  if((current_time.tv_sec*1e6 + current_time.tv_usec) - (last_time_am7_msg_sent.tv_sec*1e6 + last_time_am7_msg_sent.tv_usec) > (1e6/MAX_FREQUENCY_MSG_OUT)){
    writing_routine();
    send_states_on_ivy();
  }

  //Print some stats
  if(verbose_connection){
    gettimeofday(&current_time, NULL); 
    if((current_time.tv_sec*1e6 + current_time.tv_usec) - (last_time_print_statistics.tv_sec*1e6 + last_time_print_statistics.tv_usec) > 2*1e6){
      float ca7_message_frequency_RX = (received_packets*1e6)/((current_time.tv_sec*1e6 + current_time.tv_usec) - (last_time_print_statistics.tv_sec*1e6 + last_time_print_statistics.tv_usec));
      float ca7_message_frequency_TX = (sent_packets*1e6)/((current_time.tv_sec*1e6 + current_time.tv_usec) - (last_time_print_statistics.tv_sec*1e6 + last_time_print_statistics.tv_usec));
      printf("Total received packets = %d \n",received_packets_tot);
      printf("Total sent packets = %d \n",sent_packets_tot);
      printf("Tracking sent message id = %d \n",sent_msg_id);
      printf("Tracking received message id = %d \n",received_msg_id);
      printf("Corrupted packet received = %d \n",missed_packets);
      printf("Average message RX frequency = %f \n",ca7_message_frequency_RX);
      printf("Average message TX frequency = %f \n",ca7_message_frequency_TX);
      printf("Current lidar distance in cm = %d \n",lidar_dist_cm);
      printf("Current lidar strength = %d \n",lidar_signal_strength);
      fflush(stdout);
      received_packets = 0;
      sent_packets = 0;
      gettimeofday(&last_time_print_statistics, NULL);
    }
  }

}

void* first_thread() //Receive and send messages to autopilot and send states to ivy bus
{
  while(1){ 
    send_receive_am7();
    readLiDAR();
  }
}

void* second_thread() //Filter variables, compute modeled accelerations and fill up data_in_optimizer structure
{
  while(1){ 
    //Retrieve the data from the communication and physical system properties
    struct am7_data_in myam7_data_in_copy;
    float extra_data_in_copy[255];
    pthread_mutex_lock(&mutex_am7);   
    memcpy(&myam7_data_in_copy, &myam7_data_in, sizeof(struct am7_data_in));
    memcpy(&extra_data_in_copy, &extra_data_in, sizeof(extra_data_in));
    pthread_mutex_unlock(&mutex_am7);

    ////////////////////////////////////////////////////////////////////////////////////////////// Define Non real time variables:
    float K_p_T = extra_data_in_copy[0];
    float K_p_M = extra_data_in_copy[1];
    float m = extra_data_in_copy[2];
    float I_xx = extra_data_in_copy[3];
    float I_yy = extra_data_in_copy[4];
    float I_zz = extra_data_in_copy[5];
    float l_1 = extra_data_in_copy[6];
    float l_2 = extra_data_in_copy[7];
    float l_3 = extra_data_in_copy[8];
    float l_4 = extra_data_in_copy[9];
    float l_z = extra_data_in_copy[10];
    float max_omega = extra_data_in_copy[11];
    float min_omega = extra_data_in_copy[12];
    float max_b = extra_data_in_copy[13];
    float min_b = extra_data_in_copy[14];
    float max_g = extra_data_in_copy[15];
    float min_g = extra_data_in_copy[16];
    float max_theta = extra_data_in_copy[17];
    float min_theta = extra_data_in_copy[18];
    float max_alpha = extra_data_in_copy[19]* M_PI/180;
    float min_alpha = extra_data_in_copy[20]* M_PI/180;
    float max_phi = extra_data_in_copy[21];
    float Cm_zero = extra_data_in_copy[22];
    float Cm_alpha = extra_data_in_copy[23];
    float Cl_alpha = extra_data_in_copy[24];
    float Cd_zero = extra_data_in_copy[25];
    float K_Cd = extra_data_in_copy[26];
    float S = extra_data_in_copy[27];
    float wing_chord = extra_data_in_copy[28];
    float rho = extra_data_in_copy[29];
    float W_act_motor_const = extra_data_in_copy[30];
    float W_act_motor_speed = extra_data_in_copy[31]; 
    float W_act_tilt_el_const = extra_data_in_copy[32];
    float W_act_tilt_el_speed = extra_data_in_copy[33]; 
    float W_act_tilt_az_const = extra_data_in_copy[34];
    float W_act_tilt_az_speed = extra_data_in_copy[35]; 
    float W_act_theta_const = extra_data_in_copy[36];
    float W_act_theta_speed = extra_data_in_copy[37]; 
    float W_act_phi_const = extra_data_in_copy[38];
    float W_act_phi_speed = extra_data_in_copy[39]; 
    float W_dv_1 = extra_data_in_copy[40];
    float W_dv_2 = extra_data_in_copy[41];
    float W_dv_3 = extra_data_in_copy[42];
    float W_dv_4 = extra_data_in_copy[43];
    float W_dv_5 = extra_data_in_copy[44];
    float W_dv_6 = extra_data_in_copy[45];
    float gamma_quadratic_du = extra_data_in_copy[46];
  
    float Cy_beta = extra_data_in_copy[47];
    float Cl_beta = extra_data_in_copy[48];
    float wing_span = extra_data_in_copy[49];

    float aoa_protection_speed = extra_data_in_copy[50];

    float W_act_ailerons_const = extra_data_in_copy[51];
    float W_act_ailerons_speed = extra_data_in_copy[52]; 
    float min_delta_ailerons = extra_data_in_copy[53];
    float max_delta_ailerons = extra_data_in_copy[54];
    float CL_aileron = extra_data_in_copy[55];   

    float k_alt_tilt_constraint = extra_data_in_copy[56];
    float min_alt_tilt_constraint = extra_data_in_copy[57];

    float transition_speed = extra_data_in_copy[58];

    float desired_motor_value = extra_data_in_copy[59]; 
    float desired_el_value = extra_data_in_copy[60]; 
    float desired_az_value = extra_data_in_copy[61]; 
    float desired_ailerons_value = extra_data_in_copy[62];

    float theta_gain = extra_data_in_copy[63];
    float phi_gain = extra_data_in_copy[64];
    float p_body_gain = extra_data_in_copy[65];
    float q_body_gain = extra_data_in_copy[66];
    float r_body_gain = extra_data_in_copy[67];
    float k_d_airspeed = extra_data_in_copy[68];

    float min_theta_hard = extra_data_in_copy[69];
    float max_theta_hard = extra_data_in_copy[70];
    float min_phi_hard = extra_data_in_copy[71];
    float max_phi_hard = extra_data_in_copy[72];

    float disable_acc_decrement_inner_loop = extra_data_in_copy[73];

    float filter_cutoff_frequency_telem = extra_data_in_copy[74];
    float max_airspeed = extra_data_in_copy[75];
    
    float vert_acc_margin = extra_data_in_copy[76];

    float power_Cd_0 = extra_data_in_copy[77];
    float power_Cd_a = extra_data_in_copy[78];
    float prop_R = extra_data_in_copy[79];
    float prop_Cd_0 = extra_data_in_copy[80];
    float prop_Cl_0 = extra_data_in_copy[81];
    float prop_Cd_a = extra_data_in_copy[82];
    float prop_Cl_a = extra_data_in_copy[83];
    float prop_delta = extra_data_in_copy[84];
    float prop_sigma = extra_data_in_copy[85];
    float prop_theta = extra_data_in_copy[86];

    float beacon_tracking_id_local = extra_data_in_copy[87];
    float desired_sixdof_mode_local = extra_data_in_copy[88];

    float use_u_init_outer_loop = extra_data_in_copy[89];
    float use_u_init_inner_loop = extra_data_in_copy[90];

    float K_t_airspeed = extra_data_in_copy[91];

    float pqr_first_order_filter_omega_telem = extra_data_in_copy[92];

    float single_loop_controller = extra_data_in_copy[93];

    float use_new_aero_model = extra_data_in_copy[94];

    float use_received_ang_ref_in_inner_loop = extra_data_in_copy[95];

    //Exceptions: 
    if(beacon_tracking_id_local > 0.1f){
      pthread_mutex_lock(&mutex_sixdof);
      beacon_tracking_id = beacon_tracking_id_local;
      pthread_mutex_unlock(&mutex_sixdof);
    }
    if(desired_sixdof_mode_local > 0.1f){
      pthread_mutex_lock(&mutex_sixdof);
      desired_sixdof_mode = desired_sixdof_mode_local;
      pthread_mutex_unlock(&mutex_sixdof);
    }
    if(filter_cutoff_frequency_telem < 0.1f){
      filter_cutoff_frequency_telem = (int)filter_cutoff_frequency_init;
    }
    if(pqr_first_order_filter_omega_telem > 0.1f){
      filter_cutoff_first_order_pqr = pqr_first_order_filter_omega_telem;
    }
    pqr_first_order_filter_tau = 1.0f - exp(-filter_cutoff_first_order_pqr*refresh_time_filters);

    ///////////////////////////////////////////////////////////////////////////////////////////////// Define real time variables:
    float Phi = (myam7_data_in_copy.phi_state_int*1e-2 * M_PI/180);
    float Theta = (myam7_data_in_copy.theta_state_int*1e-2 * M_PI/180);
    float Psi = (myam7_data_in_copy.psi_state_int*1e-2 * M_PI/180);
    float delta_ailerons = (myam7_data_in_copy.ailerons_state_int*1e-2 * M_PI/180);
    float Omega_1 = (myam7_data_in_copy.motor_1_state_int*1e-1), Omega_2 = (myam7_data_in_copy.motor_2_state_int*1e-1);
    float Omega_3 = (myam7_data_in_copy.motor_3_state_int*1e-1), Omega_4 = (myam7_data_in_copy.motor_4_state_int*1e-1);
    float b_1 = (myam7_data_in_copy.el_1_state_int*1e-2 * M_PI/180), b_2 = (myam7_data_in_copy.el_2_state_int*1e-2 * M_PI/180);
    float b_3 = (myam7_data_in_copy.el_3_state_int*1e-2 * M_PI/180), b_4 = (myam7_data_in_copy.el_4_state_int*1e-2 * M_PI/180);
    float g_1 = (myam7_data_in_copy.az_1_state_int*1e-2 * M_PI/180), g_2 = (myam7_data_in_copy.az_2_state_int*1e-2 * M_PI/180);
    float g_3 = (myam7_data_in_copy.az_3_state_int*1e-2 * M_PI/180), g_4 = (myam7_data_in_copy.az_4_state_int*1e-2 * M_PI/180);
    float p = (myam7_data_in_copy.p_state_int*1e-1 * M_PI/180), q = (myam7_data_in_copy.q_state_int*1e-1 * M_PI/180); 
    float r = (myam7_data_in_copy.r_state_int*1e-1 * M_PI/180);
    float V = (myam7_data_in_copy.airspeed_state_int*1e-2);
    float flight_path_angle = (myam7_data_in_copy.gamma_state_int*1e-2 * M_PI/180);
    float Beta = (myam7_data_in_copy.beta_state_int*1e-2 * M_PI/180);

    float desired_theta_value = (myam7_data_in_copy.desired_theta_value_int*1e-2 * M_PI/180);
    float desired_phi_value = (myam7_data_in_copy.desired_phi_value_int*1e-2 * M_PI/180);

    float approach_mode = (myam7_data_in_copy.approach_boolean);
    float lidar_alt_corrected = (myam7_data_in_copy.lidar_alt_corrected_int*1e-2);
    float pseudo_control_ax = (myam7_data_in_copy.pseudo_control_ax_int*1e-2);
    float pseudo_control_ay = (myam7_data_in_copy.pseudo_control_ay_int*1e-2);
    float pseudo_control_az = (myam7_data_in_copy.pseudo_control_az_int*1e-2);
    float pseudo_control_p_dot = (myam7_data_in_copy.pseudo_control_p_dot_int*1e-1 * M_PI/180);
    float pseudo_control_q_dot = (myam7_data_in_copy.pseudo_control_q_dot_int*1e-1 * M_PI/180);
    float pseudo_control_r_dot = (myam7_data_in_copy.pseudo_control_r_dot_int*1e-1 * M_PI/180);

    float p_dot_filt_ec = (myam7_data_in_copy.p_dot_filt_int * 1e-1 * M_PI/180);
    float q_dot_filt_ec = (myam7_data_in_copy.q_dot_filt_int * 1e-1 * M_PI/180);
    float r_dot_filt_ec = (myam7_data_in_copy.r_dot_filt_int * 1e-1 * M_PI/180); 

    float psi_dot_cmd_ec = (myam7_data_in_copy.psi_dot_cmd_int * 1e-2 * M_PI/180);

    float failure_mode = (float) (myam7_data_in_copy.failure_mode);

    #ifdef TEST_CONTROLLER
    #warning "You are using the testing variable, watch out!"
    //Replace variables with dummy values, to test: 
    float pi = M_PI; 
    //Real time variables: 
    Omega_1 = 800;
    Omega_2 = 800;
    Omega_3 = 800;
    Omega_4 = 800;
    b_1 = 0 * pi/180;
    b_2 = 0 * pi/180;
    b_3 = 0 * pi/180;
    b_4 = 0 * pi/180;
    g_1 = 0 * pi/180;
    g_2 = 0 * pi/180;
    g_3 = 0 * pi/180;
    g_4 = 0 * pi/180;
    p = 0 * pi/180;
    q = 0 * pi/180;
    r = 0 * pi/180;
    V = 0;
    flight_path_angle = 0 * pi/180;
    Beta = 0 * pi/180;
    Phi = 0 * pi/180;
    Theta = 0 * pi/180;
    Psi = 0 * pi/180;
    delta_ailerons = 0 * pi/180;
    desired_theta_value = 0 * pi/180;
    desired_phi_value = 0 * pi/180;
    approach_mode = 0;
    lidar_alt_corrected = 0;
    pseudo_control_ax = 0;
    pseudo_control_ay = 0;
    pseudo_control_az = 0;
    pseudo_control_p_dot = 0 * pi/180;
    pseudo_control_q_dot = 0 * pi/180;
    pseudo_control_r_dot = 0 * pi/180;
    p_dot_filt_ec = 0 * pi/180;
    q_dot_filt_ec = 0 * pi/180;
    r_dot_filt_ec = 0 * pi/180;
    psi_dot_cmd_ec = 0 * pi/180;
    failure_mode = 3;

    //Parameters:
    K_p_T = 1.106465e-5;
    K_p_M = 1.835091e-7;
    m = 2.45;
    I_xx = 0.156548;
    I_yy = 0.161380;
    I_zz = 0.258662;
    l_1 = 0.228;
    l_2 = 0.228;
    l_3 = 0.37;
    l_4 = 0.37;
    l_z = 0;
    max_omega = 1000;
    min_omega = 100;
    max_b = 25;
    min_b = -130;
    max_g = 90;
    min_g = -90;
    max_theta = 50;
    min_theta = -15;
    max_phi = 80;
    Cm_zero = 0.05;
    Cm_alpha = -0.1;
    Cl_alpha = 3.5;
    Cd_zero = 0.2;
    K_Cd = 0.08;
    S = 0.43;
    wing_chord = 0.3;
    rho = 1.225;
    W_act_motor_const = 10;
    W_act_motor_speed = 0;
    W_act_tilt_el_const = 0;
    W_act_tilt_el_speed = 0;
    W_act_tilt_az_const = 0;
    W_act_tilt_az_speed = 10;
    W_act_theta_const = 100;
    W_act_theta_speed = -15;
    W_act_phi_const = 100;
    W_act_phi_speed = -15;
    W_act_ailerons_const = .5;
    W_act_ailerons_speed = 0;
    W_dv_1 = 0.01;
    W_dv_2 = 0.01;
    W_dv_3 = 0.05;
    W_dv_4 = 0.1;
    W_dv_5 = 0.1;
    W_dv_6 = 0.1;
    gamma_quadratic_du = .5e-7;
    Cy_beta = 0.1;
    Cl_beta = 0.1;
    wing_span = 1.4;
    aoa_protection_speed = 3;
    W_act_ailerons_const = 0.5;
    W_act_ailerons_speed = 0;
    min_delta_ailerons = -25;
    max_delta_ailerons = 25;
    CL_aileron = 0.1;
    k_alt_tilt_constraint = 55;
    min_alt_tilt_constraint = 0.2;
    transition_speed = 7;
    desired_motor_value = 0;
    desired_el_value = 0;
    desired_az_value = 0;
    desired_ailerons_value = 0;
    theta_gain = 1;
    phi_gain = 1;
    p_body_gain = 1;
    q_body_gain = 1;
    r_body_gain = 1;
    k_d_airspeed = 0.02;
    min_theta_hard = -30 * pi/180;
    max_theta_hard = 60 * pi/180;
    min_phi_hard = -80 * pi/180;
    max_phi_hard = 80 * pi/180;
    disable_acc_decrement_inner_loop = 0;
    filter_cutoff_frequency_telem = 12;
    max_airspeed = 15;
    vert_acc_margin = 0.5;
    power_Cd_0 = 0.05;
    power_Cd_a = 0.36;
    prop_R = 0.1270;
    prop_Cd_0 = 0.05;
    prop_Cl_0 = 0.0;
    prop_Cd_a = 0.36;
    prop_Cl_a = 3.46;
    prop_delta = 0.2;
    prop_sigma = 0.0652;
    prop_theta = 0.2188;
    beacon_tracking_id_local = 1636;
    desired_sixdof_mode_local = 1;
    use_u_init_outer_loop = 1;
    use_u_init_inner_loop = 1;
    K_t_airspeed = 0.02;
    pqr_first_order_filter_omega_telem = 12;
    single_loop_controller = 0;
    use_new_aero_model = 0;
    use_received_ang_ref_in_inner_loop = 0;
    
    #endif

    //Bound motor values to be within min and max value to avoid NaN
    Bound(Omega_1,min_omega,max_omega);
    Bound(Omega_2,min_omega,max_omega);
    Bound(Omega_3,min_omega,max_omega);
    Bound(Omega_4,min_omega,max_omega);

    //////////////////////////////////////////////////////////////////////////////////////////Compute modeled accellerations:
    float K_p_T_airspeed_corrected = K_p_T * (1 - V*K_t_airspeed);
    float K_p_M_airspeed_corrected = K_p_M * (1 - V*K_t_airspeed);
    double gain_motor = (max_omega - min_omega)/2;
    double Omega_1_scaled = Omega_1 / gain_motor;
    double Omega_2_scaled = Omega_2 / gain_motor;
    double V_scaled = V/max_airspeed;
    double modeled_accelerations[6];

    if(use_new_aero_model  > 0.5f)
      compute_acc_nonlinear_control_rf_w_ailerons_v2_new_aero(
              Beta, CL_aileron, Cd_zero, Cl_alpha,
              Cm_zero, Cm_alpha, I_xx, I_yy, I_zz,
              K_Cd, Omega_1, Omega_2, Omega_3, Omega_4,
              Omega_1_scaled, Omega_2_scaled, Phi, S,
              Theta, V, V_scaled, b_1, b_2, b_3,
              b_4, delta_ailerons, flight_path_angle, g_1,
              g_2, g_3, g_4, l_1, l_2, l_3,
              l_4, l_z, m, p, prop_R, prop_Cd_0,
              prop_Cl_0, prop_Cd_a, prop_Cl_a, prop_delta,
              prop_sigma, prop_theta, q, r, rho,
              wing_span, wing_chord, modeled_accelerations);
    else{
      compute_acc_control_rf_basic(
              Beta, CL_aileron, Cd_zero, Cl_alpha,
              Cm_zero, Cm_alpha, I_xx, I_yy, I_zz,
              K_Cd, K_p_M_airspeed_corrected, K_p_T_airspeed_corrected, Omega_1, Omega_2,
              Omega_3, Omega_4, Phi, S, Theta,
              V, b_1, b_2, b_3, b_4,
              delta_ailerons, flight_path_angle, g_1, g_2,
              g_3, g_4, l_1, l_2, l_3, l_4,
              l_z, m, p, q, r, rho,
              wing_chord, modeled_accelerations);
    }


    //FILTER VALUES:
    ///////////////////////////////////Reset filters in case we want a different omega than the one inputted initially
    if((int)filter_cutoff_frequency_telem != (int)filter_cutoff_frequency){
      filter_cutoff_frequency = (int)filter_cutoff_frequency_telem;
      //Re-init filters: 
      tau_indi = 1.0f / (filter_cutoff_frequency);
      
      for (int i = 0; i < 6; i++) {
        init_butterworth_2_low_pass(&modeled_accelerations_filter[i], tau_indi, refresh_time_filters, 0.0);
      }

      for (int i = 0; i < 3; i++) {
        init_butterworth_2_low_pass(&body_rates_filter[i], tau_indi, refresh_time_filters, 0.0);
      }

      for (int i = 0; i < NUM_ACT_IN_U_IN; i++) {
        init_butterworth_2_low_pass(&u_in_filter[i], tau_indi, refresh_time_filters, 0.0);
      }

      init_butterworth_2_low_pass(&airspeed_filter, tau_indi, refresh_time_filters, 0.0);
      init_butterworth_2_low_pass(&flight_path_angle_filter, tau_indi, refresh_time_filters, 0.0);
      init_butterworth_2_low_pass(&psi_filter, tau_indi, refresh_time_filters, 0.0);
    }

    //////////////////////////////////////////////////////////////////////Filter body rates for the error controller
    p_filtered_ec = p_filtered_ec + pqr_first_order_filter_tau * (p - p_filtered_ec);
    q_filtered_ec = q_filtered_ec + pqr_first_order_filter_tau * (q - q_filtered_ec);
    r_filtered_ec = r_filtered_ec + pqr_first_order_filter_tau * (r - r_filtered_ec);

    //////////////////////////////////////////////////////////////////////////////////////Filter current modeled acc
    float modeled_accelerations_filtered[6];
    for(int i = 0; i < 6; i++){
      update_butterworth_2_low_pass(&modeled_accelerations_filter[i], (float) modeled_accelerations[i]);
      //Check for NaN in filters and reset them: 
      if(modeled_accelerations_filter[i].o[0] != modeled_accelerations_filter[i].o[0]){
        init_butterworth_2_low_pass(&modeled_accelerations_filter[i], tau_indi, refresh_time_filters, 0.0);
        if(verbose_filters){
          printf("WARNING, modeled_accelerations_filter filter number %d REINITIALIZED!!!! \n",i);
        }
      }
      modeled_accelerations_filtered[i] = (double) modeled_accelerations_filter[i].o[0];
    }

    //////////////////////////////////////////////////////////////////////////////////////////////Filter body rates 
    float body_rates_array_filtered[3];
    float body_rates_array[3] = {p, q, r};
    for (int i = 0; i < 3; i++) {
      update_butterworth_2_low_pass(&body_rates_filter[i], (float) body_rates_array[i]);
      //Check for NaN in filters and reset them: 
      if(body_rates_filter[i].o[0] != body_rates_filter[i].o[0]){
        init_butterworth_2_low_pass(&body_rates_filter[i], tau_indi, refresh_time_filters, 0.0);
        if(verbose_filters){
          printf("WARNING, body_rates_filter filter number %d REINITIALIZED!!!! \n",i);
        }
      }
      body_rates_array_filtered[i] = body_rates_filter[i].o[0]; 
    }

    //////////////////////////////////////////////////////////////////////////////////////////////Filter u_in (full)
    float u_in_filtered[NUM_ACT_IN_U_IN];
    float u_in[NUM_ACT_IN_U_IN] = {Omega_1, Omega_2, Omega_3, Omega_4, b_1, b_2, b_3, b_4, g_1, g_2, g_3, g_4, Theta, Phi, delta_ailerons};
    for (int i = 0; i < NUM_ACT_IN_U_IN; i++) {
      update_butterworth_2_low_pass(&u_in_filter[i], (float) u_in[i]);
      //Check for NaN in filters and reset them: 
      if(u_in_filter[i].o[0] != u_in_filter[i].o[0]){
        init_butterworth_2_low_pass(&u_in_filter[i], tau_indi, refresh_time_filters, 0.0);
        if(verbose_filters){
          printf("WARNING, u_in_filter filter number %d REINITIALIZED!!!! \n",i);
        }
      }
      u_in_filtered[i] = u_in_filter[i].o[0];
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////Filter airspeed
    update_butterworth_2_low_pass(&airspeed_filter, (float) V);
    //Check for NaN in filter and reset it: 
    if(airspeed_filter.o[0] != airspeed_filter.o[0]){
      init_butterworth_2_low_pass(&airspeed_filter, tau_indi, refresh_time_filters, 0.0);
      if(verbose_filters){
        printf("WARNING, airspeed filter REINITIALIZED!!!! \n");
      }
    }
    float airspeed_filtered = airspeed_filter.o[0]; 

    ///////////////////////////////////////////////////////////////////////////////////////////Filter flight path angle
    update_butterworth_2_low_pass(&flight_path_angle_filter, (float) flight_path_angle);
    //Check for NaN in filter and reset it: 
    if(flight_path_angle_filter.o[0] != flight_path_angle_filter.o[0]){
      init_butterworth_2_low_pass(&flight_path_angle_filter, tau_indi, refresh_time_filters, 0.0);
      if(verbose_filters){
        printf("WARNING, flight path angle filter REINITIALIZED!!!! \n");
      }
    }
    float flight_path_angle_filtered = flight_path_angle_filter.o[0];

    ///////////////////////////////////////////////////////////////////////////////////////////Filter psi
    update_butterworth_2_low_pass(&psi_filter, (float) Psi);
    //Check for NaN in filter and reset it:
    if(psi_filter.o[0] != psi_filter.o[0]){
      init_butterworth_2_low_pass(&psi_filter, tau_indi, refresh_time_filters, 0.0);
      if(verbose_filters){
        printf("WARNING, psi filter REINITIALIZED!!!! \n");
      }
    }
    float psi_filtered = psi_filter.o[0];

    ////////////////////////////////////////////////////////////////////////////////////// ASSIGN TO THE DATA IN STRUCTURE

    struct data_in_optimizer mydata_in_optimizer_copy;
    //Real time filtered: 
    mydata_in_optimizer_copy.motor_1_state_filtered = u_in_filtered[0]; 
    mydata_in_optimizer_copy.motor_2_state_filtered = u_in_filtered[1];
    mydata_in_optimizer_copy.motor_3_state_filtered = u_in_filtered[2];
    mydata_in_optimizer_copy.motor_4_state_filtered = u_in_filtered[3];
    mydata_in_optimizer_copy.el_1_state_filtered = u_in_filtered[4];
    mydata_in_optimizer_copy.el_2_state_filtered = u_in_filtered[5];
    mydata_in_optimizer_copy.el_3_state_filtered = u_in_filtered[6];
    mydata_in_optimizer_copy.el_4_state_filtered = u_in_filtered[7];
    mydata_in_optimizer_copy.az_1_state_filtered = u_in_filtered[8];
    mydata_in_optimizer_copy.az_2_state_filtered = u_in_filtered[9];
    mydata_in_optimizer_copy.az_3_state_filtered = u_in_filtered[10];
    mydata_in_optimizer_copy.az_4_state_filtered = u_in_filtered[11];
    mydata_in_optimizer_copy.theta_state_filtered = u_in_filtered[12];
    mydata_in_optimizer_copy.phi_state_filtered = u_in_filtered[13];
    mydata_in_optimizer_copy.ailerons_state_filtered = u_in_filtered[14];
    mydata_in_optimizer_copy.psi_state_filtered = psi_filtered;
    mydata_in_optimizer_copy.gamma_state_filtered = flight_path_angle_filtered;
    mydata_in_optimizer_copy.p_state_filtered = body_rates_array_filtered[0];
    mydata_in_optimizer_copy.q_state_filtered = body_rates_array_filtered[1];
    mydata_in_optimizer_copy.r_state_filtered = body_rates_array_filtered[2];
    mydata_in_optimizer_copy.airspeed_state_filtered = airspeed_filtered;
    mydata_in_optimizer_copy.beta_state_filtered = Beta;
    mydata_in_optimizer_copy.approach_boolean = approach_mode;
    mydata_in_optimizer_copy.lidar_alt_corrected = lidar_alt_corrected;
    mydata_in_optimizer_copy.pseudo_control_ax = pseudo_control_ax;
    mydata_in_optimizer_copy.pseudo_control_ay = pseudo_control_ay;
    mydata_in_optimizer_copy.pseudo_control_az = pseudo_control_az;
    mydata_in_optimizer_copy.pseudo_control_p_dot = pseudo_control_p_dot;
    mydata_in_optimizer_copy.pseudo_control_q_dot = pseudo_control_q_dot;
    mydata_in_optimizer_copy.pseudo_control_r_dot = pseudo_control_r_dot;
    mydata_in_optimizer_copy.desired_theta_value = desired_theta_value;
    mydata_in_optimizer_copy.desired_phi_value = desired_phi_value;

    //Values for the error controller:
    mydata_in_optimizer_copy.phi_state_ec = Phi; //Unfiltered 
    mydata_in_optimizer_copy.theta_state_ec = Theta; //Unfiltered
    mydata_in_optimizer_copy.psi_state_ec = Psi; //Unfiltered
    mydata_in_optimizer_copy.p_state_ec = p_filtered_ec; //Filtered with first order filter (locally)
    mydata_in_optimizer_copy.q_state_ec = q_filtered_ec; //Filtered with first order filter (locally)
    mydata_in_optimizer_copy.r_state_ec = r_filtered_ec; //Filtered with first order filter (locally)
    mydata_in_optimizer_copy.p_dot_state_ec = p_dot_filt_ec; //Filtered with second order butterworth filter (on AP)
    mydata_in_optimizer_copy.q_dot_state_ec = q_dot_filt_ec; //Filtered with second order butterworth filter (on AP)
    mydata_in_optimizer_copy.r_dot_state_ec = r_dot_filt_ec; //Filtered with second order butterworth filter (on AP)
    mydata_in_optimizer_copy.theta_gain = theta_gain;
    mydata_in_optimizer_copy.phi_gain = phi_gain;
    mydata_in_optimizer_copy.p_body_gain = p_body_gain;
    mydata_in_optimizer_copy.q_body_gain = q_body_gain;
    mydata_in_optimizer_copy.r_body_gain = r_body_gain;
    mydata_in_optimizer_copy.k_d_airspeed = k_d_airspeed;
    mydata_in_optimizer_copy.min_theta_hard = min_theta_hard;
    mydata_in_optimizer_copy.max_theta_hard = max_theta_hard;
    mydata_in_optimizer_copy.min_phi_hard = min_phi_hard;
    mydata_in_optimizer_copy.max_phi_hard = max_phi_hard;
    mydata_in_optimizer_copy.psi_dot_cmd_ec = psi_dot_cmd_ec;

    //Values for the failure: 
    mydata_in_optimizer_copy.failure_mode = failure_mode;

    //Modeled accellerations filtered: 
    mydata_in_optimizer_copy.modeled_ax_filtered = modeled_accelerations_filtered[0]; 
    mydata_in_optimizer_copy.modeled_ay_filtered = modeled_accelerations_filtered[1]; 
    mydata_in_optimizer_copy.modeled_az_filtered = modeled_accelerations_filtered[2];
    mydata_in_optimizer_copy.modeled_p_dot_filtered = modeled_accelerations_filtered[3];
    mydata_in_optimizer_copy.modeled_q_dot_filtered = modeled_accelerations_filtered[4];
    mydata_in_optimizer_copy.modeled_r_dot_filtered = modeled_accelerations_filtered[5];

    //Non real time: 
    mydata_in_optimizer_copy.K_p_T = K_p_T_airspeed_corrected;
    mydata_in_optimizer_copy.K_p_M = K_p_M_airspeed_corrected;
    mydata_in_optimizer_copy.m = m;
    mydata_in_optimizer_copy.I_xx = I_xx;
    mydata_in_optimizer_copy.I_yy = I_yy;
    mydata_in_optimizer_copy.I_zz = I_zz;
    mydata_in_optimizer_copy.l_1 = l_1;
    mydata_in_optimizer_copy.l_2 = l_2;
    mydata_in_optimizer_copy.l_3 = l_3;
    mydata_in_optimizer_copy.l_4 = l_4;
    mydata_in_optimizer_copy.l_z = l_z;
    mydata_in_optimizer_copy.max_omega = max_omega;
    mydata_in_optimizer_copy.min_omega = min_omega;
    mydata_in_optimizer_copy.max_b = max_b;
    mydata_in_optimizer_copy.min_b = min_b;
    mydata_in_optimizer_copy.max_g = max_g;
    mydata_in_optimizer_copy.min_g = min_g;
    mydata_in_optimizer_copy.max_theta = max_theta;
    mydata_in_optimizer_copy.min_theta = min_theta;
    mydata_in_optimizer_copy.max_alpha = max_alpha;
    mydata_in_optimizer_copy.min_alpha = min_alpha;
    mydata_in_optimizer_copy.max_phi = max_phi;
    mydata_in_optimizer_copy.Cm_zero = Cm_zero;
    mydata_in_optimizer_copy.Cm_alpha = Cm_alpha;
    mydata_in_optimizer_copy.Cl_alpha = Cl_alpha;
    mydata_in_optimizer_copy.Cd_zero = Cd_zero;
    mydata_in_optimizer_copy.K_Cd = K_Cd;
    mydata_in_optimizer_copy.S = S;
    mydata_in_optimizer_copy.wing_chord = wing_chord;
    mydata_in_optimizer_copy.rho = rho;
    mydata_in_optimizer_copy.W_act_motor_const = W_act_motor_const;
    mydata_in_optimizer_copy.W_act_motor_speed = W_act_motor_speed;
    mydata_in_optimizer_copy.W_act_tilt_el_const = W_act_tilt_el_const;
    mydata_in_optimizer_copy.W_act_tilt_el_speed = W_act_tilt_el_speed;
    mydata_in_optimizer_copy.W_act_tilt_az_const = W_act_tilt_az_const;
    mydata_in_optimizer_copy.W_act_tilt_az_speed = W_act_tilt_az_speed;
    mydata_in_optimizer_copy.W_act_theta_const = W_act_theta_const;
    mydata_in_optimizer_copy.W_act_theta_speed = W_act_theta_speed;
    mydata_in_optimizer_copy.W_act_phi_const = W_act_phi_const;
    mydata_in_optimizer_copy.W_act_phi_speed = W_act_phi_speed;
    mydata_in_optimizer_copy.W_dv_1 = W_dv_1;
    mydata_in_optimizer_copy.W_dv_2 = W_dv_2;
    mydata_in_optimizer_copy.W_dv_3 = W_dv_3;
    mydata_in_optimizer_copy.W_dv_4 = W_dv_4;
    mydata_in_optimizer_copy.W_dv_5 = W_dv_5;
    mydata_in_optimizer_copy.W_dv_6 = W_dv_6;
    mydata_in_optimizer_copy.gamma_quadratic_du = gamma_quadratic_du;
    mydata_in_optimizer_copy.Cy_beta = Cy_beta;
    mydata_in_optimizer_copy.Cl_beta = Cl_beta;
    mydata_in_optimizer_copy.wing_span = wing_span;
    mydata_in_optimizer_copy.aoa_protection_speed = aoa_protection_speed;
    mydata_in_optimizer_copy.W_act_ailerons_const = W_act_ailerons_const;
    mydata_in_optimizer_copy.W_act_ailerons_speed = W_act_ailerons_speed;
    mydata_in_optimizer_copy.min_delta_ailerons = min_delta_ailerons;
    mydata_in_optimizer_copy.max_delta_ailerons = max_delta_ailerons;
    mydata_in_optimizer_copy.CL_aileron = CL_aileron;
    mydata_in_optimizer_copy.k_alt_tilt_constraint = k_alt_tilt_constraint;
    mydata_in_optimizer_copy.min_alt_tilt_constraint = min_alt_tilt_constraint;
    mydata_in_optimizer_copy.transition_speed = transition_speed;
    mydata_in_optimizer_copy.desired_motor_value = desired_motor_value;
    mydata_in_optimizer_copy.desired_el_value = desired_el_value;
    mydata_in_optimizer_copy.desired_az_value = desired_az_value;
    mydata_in_optimizer_copy.desired_ailerons_value = desired_ailerons_value;
    mydata_in_optimizer_copy.disable_acc_decrement_inner_loop = disable_acc_decrement_inner_loop;
    mydata_in_optimizer_copy.vert_acc_margin = vert_acc_margin;
    mydata_in_optimizer_copy.power_Cd_0 = power_Cd_0;
    mydata_in_optimizer_copy.power_Cd_a = power_Cd_a;
    mydata_in_optimizer_copy.prop_R = prop_R;
    mydata_in_optimizer_copy.prop_Cd_0 = prop_Cd_0;
    mydata_in_optimizer_copy.prop_Cl_0 = prop_Cl_0;
    mydata_in_optimizer_copy.prop_Cd_a = prop_Cd_a;
    mydata_in_optimizer_copy.prop_Cl_a = prop_Cl_a;
    mydata_in_optimizer_copy.prop_delta = prop_delta;
    mydata_in_optimizer_copy.prop_sigma = prop_sigma;
    mydata_in_optimizer_copy.prop_theta = prop_theta;
    mydata_in_optimizer_copy.max_airspeed = max_airspeed;
    mydata_in_optimizer_copy.use_u_init_outer_loop = use_u_init_outer_loop;
    mydata_in_optimizer_copy.use_u_init_inner_loop = use_u_init_inner_loop;

    mydata_in_optimizer_copy.single_loop_controller = single_loop_controller;
    mydata_in_optimizer_copy.use_new_aero_model = use_new_aero_model;
    mydata_in_optimizer_copy.use_received_ang_ref_in_inner_loop = use_received_ang_ref_in_inner_loop;
    

    pthread_mutex_lock(&mutex_optimizer_input);
    memcpy(&mydata_in_optimizer, &mydata_in_optimizer_copy, sizeof(struct data_in_optimizer));
    pthread_mutex_unlock(&mutex_optimizer_input);

    //Wait until time is not at least refresh_time_filters
    gettimeofday(&current_time, NULL);
    while(((current_time.tv_sec*1e6 + current_time.tv_usec) - (time_last_filt.tv_sec*1e6 + time_last_filt.tv_usec)) < refresh_time_filters*1e6){
      usleep(5);
      gettimeofday(&current_time, NULL);
    }
    if(verbose_runtime){
      printf("Effective refresh time filters: %f \n", ((current_time.tv_sec*1e6 + current_time.tv_usec) - (time_last_filt.tv_sec*1e6 + time_last_filt.tv_usec)));
    }
    //Update time_last_filt:
    gettimeofday(&time_last_filt, NULL);

  }
}

void* third_thread() //Run the outer loop of the optimization code
{
  while(1){ 
    //Retrieve the data from the filtering and assign variables thread
    struct data_in_optimizer mydata_in_optimizer_copy;
    pthread_mutex_lock(&mutex_optimizer_input);
    memcpy(&mydata_in_optimizer_copy, &mydata_in_optimizer, sizeof(struct data_in_optimizer));
    pthread_mutex_unlock(&mutex_optimizer_input);

    double verbose = verbose_outer_loop;

    //Check if u_init_outer was not initialized yet:
    if(u_init_outer[0] < 1){
      u_init_outer[0] = mydata_in_optimizer_copy.motor_1_state_filtered;
      u_init_outer[1] = mydata_in_optimizer_copy.motor_2_state_filtered;
      u_init_outer[2] = mydata_in_optimizer_copy.motor_3_state_filtered;
      u_init_outer[3] = mydata_in_optimizer_copy.motor_4_state_filtered;
      u_init_outer[4] = mydata_in_optimizer_copy.el_1_state_filtered;
      u_init_outer[5] = mydata_in_optimizer_copy.el_2_state_filtered;
      u_init_outer[6] = mydata_in_optimizer_copy.el_3_state_filtered;
      u_init_outer[7] = mydata_in_optimizer_copy.el_4_state_filtered;
      u_init_outer[8] = mydata_in_optimizer_copy.az_1_state_filtered;
      u_init_outer[9] = mydata_in_optimizer_copy.az_2_state_filtered;
      u_init_outer[10] = mydata_in_optimizer_copy.az_3_state_filtered;
      u_init_outer[11] = mydata_in_optimizer_copy.az_4_state_filtered;
      u_init_outer[12] = mydata_in_optimizer_copy.theta_state_filtered;
      u_init_outer[13] = mydata_in_optimizer_copy.phi_state_filtered;
      u_init_outer[14] = mydata_in_optimizer_copy.ailerons_state_filtered;
    }

    //Prepare input variables from the mydata_in_optimizer_copy structure: 
    double K_p_T =(double) mydata_in_optimizer_copy.K_p_T;
    double K_p_M =(double) mydata_in_optimizer_copy.K_p_M;
    double m =(double) mydata_in_optimizer_copy.m;
    double I_xx =(double) mydata_in_optimizer_copy.I_xx;
    double I_yy =(double) mydata_in_optimizer_copy.I_yy;
    double I_zz =(double) mydata_in_optimizer_copy.I_zz;
    double l_1 =(double) mydata_in_optimizer_copy.l_1;
    double l_2 =(double) mydata_in_optimizer_copy.l_2;
    double l_3 =(double) mydata_in_optimizer_copy.l_3;
    double l_4 =(double) mydata_in_optimizer_copy.l_4;
    double l_z =(double) mydata_in_optimizer_copy.l_z;
    double Phi =(double) mydata_in_optimizer_copy.phi_state_filtered;
    double Theta =(double) mydata_in_optimizer_copy.theta_state_filtered;
    double Omega_1 =(double) mydata_in_optimizer_copy.motor_1_state_filtered;
    double Omega_2 =(double) mydata_in_optimizer_copy.motor_2_state_filtered;
    double Omega_3 =(double) mydata_in_optimizer_copy.motor_3_state_filtered;
    double Omega_4 =(double) mydata_in_optimizer_copy.motor_4_state_filtered;
    double b_1 =(double) mydata_in_optimizer_copy.el_1_state_filtered;
    double b_2 =(double) mydata_in_optimizer_copy.el_2_state_filtered;
    double b_3 =(double) mydata_in_optimizer_copy.el_3_state_filtered;
    double b_4 =(double) mydata_in_optimizer_copy.el_4_state_filtered;
    double g_1 =(double) mydata_in_optimizer_copy.az_1_state_filtered;
    double g_2 =(double) mydata_in_optimizer_copy.az_2_state_filtered;
    double g_3 =(double) mydata_in_optimizer_copy.az_3_state_filtered;
    double g_4 =(double) mydata_in_optimizer_copy.az_4_state_filtered;
    double delta_ailerons =(double) mydata_in_optimizer_copy.ailerons_state_filtered;
    double W_act_motor_const =(double) mydata_in_optimizer_copy.W_act_motor_const;
    double W_act_motor_speed =(double) mydata_in_optimizer_copy.W_act_motor_speed;
    double W_act_tilt_el_const =(double) mydata_in_optimizer_copy.W_act_tilt_el_const;
    double W_act_tilt_el_speed =(double) mydata_in_optimizer_copy.W_act_tilt_el_speed;
    double W_act_tilt_az_const =(double) mydata_in_optimizer_copy.W_act_tilt_az_const;
    double W_act_tilt_az_speed =(double) mydata_in_optimizer_copy.W_act_tilt_az_speed;
    double W_act_theta_const =(double) mydata_in_optimizer_copy.W_act_theta_const;
    double W_act_theta_speed =(double) mydata_in_optimizer_copy.W_act_theta_speed;
    double W_act_phi_const =(double) mydata_in_optimizer_copy.W_act_phi_const;
    double W_act_phi_speed =(double) mydata_in_optimizer_copy.W_act_phi_speed;
    double W_act_ailerons_const =(double) mydata_in_optimizer_copy.W_act_ailerons_const;
    double W_act_ailerons_speed =(double) mydata_in_optimizer_copy.W_act_ailerons_speed;
    double W_dv_1 =(double) mydata_in_optimizer_copy.W_dv_1;
    double W_dv_2 =(double) mydata_in_optimizer_copy.W_dv_2;
    double W_dv_3 =(double) mydata_in_optimizer_copy.W_dv_3;
    double W_dv_4 =(double) mydata_in_optimizer_copy.W_dv_4;
    double W_dv_5 =(double) mydata_in_optimizer_copy.W_dv_5;
    double W_dv_6 =(double) mydata_in_optimizer_copy.W_dv_6;
    double max_omega =(double) mydata_in_optimizer_copy.max_omega;
    double min_omega =(double) mydata_in_optimizer_copy.min_omega;
    double max_b =(double) mydata_in_optimizer_copy.max_b;
    double min_b =(double) mydata_in_optimizer_copy.min_b;
    double max_g =(double) mydata_in_optimizer_copy.max_g;
    double min_g =(double) mydata_in_optimizer_copy.min_g;
    double max_theta =(double) mydata_in_optimizer_copy.max_theta;
    double min_theta =(double) mydata_in_optimizer_copy.min_theta;
    double max_phi =(double) mydata_in_optimizer_copy.max_phi;
    double max_delta_ailerons =(double) mydata_in_optimizer_copy.max_delta_ailerons;
    double min_delta_ailerons =(double) mydata_in_optimizer_copy.min_delta_ailerons;
    double dv[6] = {(double) mydata_in_optimizer_copy.pseudo_control_ax,
                    (double) mydata_in_optimizer_copy.pseudo_control_ay,
                    (double) mydata_in_optimizer_copy.pseudo_control_az,
                    (double) 0.0f,
                    (double) 0.0f,
                    (double) 0.0f};          
    double p =(double) mydata_in_optimizer_copy.p_state_filtered;
    double q =(double) mydata_in_optimizer_copy.q_state_filtered;
    double r =(double) mydata_in_optimizer_copy.r_state_filtered;
    double Cm_zero =(double) mydata_in_optimizer_copy.Cm_zero;
    double Cl_alpha =(double) mydata_in_optimizer_copy.Cl_alpha;
    double Cd_zero =(double) mydata_in_optimizer_copy.Cd_zero;
    double K_Cd =(double) mydata_in_optimizer_copy.K_Cd;
    double Cm_alpha =(double) mydata_in_optimizer_copy.Cm_alpha;
    double CL_aileron =(double) mydata_in_optimizer_copy.CL_aileron;
    double rho =(double) mydata_in_optimizer_copy.rho;
    double V =(double) mydata_in_optimizer_copy.airspeed_state_filtered;
    double S =(double) mydata_in_optimizer_copy.S;
    double wing_chord =(double) mydata_in_optimizer_copy.wing_chord;
    double flight_path_angle =(double) mydata_in_optimizer_copy.gamma_state_filtered;
    double max_alpha =(double) mydata_in_optimizer_copy.max_alpha;
    double min_alpha =(double) mydata_in_optimizer_copy.min_alpha;
    double Beta =(double) mydata_in_optimizer_copy.beta_state_filtered;
    double gamma_quadratic_du =(double) mydata_in_optimizer_copy.gamma_quadratic_du;
    double desired_motor_value =(double) mydata_in_optimizer_copy.desired_motor_value;
    double desired_el_value =(double) mydata_in_optimizer_copy.desired_el_value;
    double desired_az_value =(double) mydata_in_optimizer_copy.desired_az_value;
    double desired_theta_value =(double) mydata_in_optimizer_copy.desired_theta_value;
    double desired_phi_value =(double) mydata_in_optimizer_copy.desired_phi_value;
    double desired_ailerons_value =(double) mydata_in_optimizer_copy.desired_ailerons_value;
    double k_alt_tilt_constraint =(double) mydata_in_optimizer_copy.k_alt_tilt_constraint;
    double min_alt_tilt_constraint =(double) mydata_in_optimizer_copy.min_alt_tilt_constraint;
    double lidar_alt_corrected =(double) mydata_in_optimizer_copy.lidar_alt_corrected;
    double approach_mode =(double) mydata_in_optimizer_copy.approach_boolean;
    double aoa_protection_speed =(double) mydata_in_optimizer_copy.aoa_protection_speed;
    double vert_acc_margin =(double) mydata_in_optimizer_copy.vert_acc_margin;
    double max_airspeed = (double) mydata_in_optimizer_copy.max_airspeed;
    double power_Cd_0 =(double) mydata_in_optimizer_copy.power_Cd_0;
    double power_Cd_a =(double) mydata_in_optimizer_copy.power_Cd_a;
    double prop_R =(double) mydata_in_optimizer_copy.prop_R;
    double prop_Cd_0 =(double) mydata_in_optimizer_copy.prop_Cd_0;
    double prop_Cl_0 =(double) mydata_in_optimizer_copy.prop_Cl_0;
    double prop_Cd_a =(double) mydata_in_optimizer_copy.prop_Cd_a;
    double prop_Cl_a =(double) mydata_in_optimizer_copy.prop_Cl_a;
    double prop_delta =(double) mydata_in_optimizer_copy.prop_delta;
    double prop_sigma =(double) mydata_in_optimizer_copy.prop_sigma;
    double prop_theta =(double) mydata_in_optimizer_copy.prop_theta;
    double wing_span =(double) mydata_in_optimizer_copy.wing_span;

    double induced_failure =(double) mydata_in_optimizer_copy.failure_mode;

    //ERROR CONTROLLER
    double p_body_current =(double) mydata_in_optimizer_copy.p_state_ec;
    double q_body_current =(double) mydata_in_optimizer_copy.q_state_ec;
    double r_body_current =(double) mydata_in_optimizer_copy.r_state_ec;
    double p_dot_current =(double) mydata_in_optimizer_copy.p_dot_state_ec;
    double q_dot_current =(double) mydata_in_optimizer_copy.q_dot_state_ec;
    double r_dot_current =(double) mydata_in_optimizer_copy.r_dot_state_ec;
    double phi_current =(double) mydata_in_optimizer_copy.phi_state_ec;
    double theta_current =(double) mydata_in_optimizer_copy.theta_state_ec;
    double angular_proportional_gains[3] = {(double) mydata_in_optimizer_copy.phi_gain, 
                                            (double) mydata_in_optimizer_copy.theta_gain, 
                                            (double) 1.0f};
    double angular_derivative_gains[3] = {(double) mydata_in_optimizer_copy.p_body_gain, 
                                          (double) mydata_in_optimizer_copy.q_body_gain, 
                                          (double) mydata_in_optimizer_copy.r_body_gain};
    double theta_hard_max =(double) mydata_in_optimizer_copy.max_theta_hard;
    double theta_hard_min =(double) mydata_in_optimizer_copy.min_theta_hard;
    double phi_hard_max =(double) mydata_in_optimizer_copy.max_phi_hard;
    double phi_hard_min =(double) mydata_in_optimizer_copy.min_phi_hard;
    double k_d_airspeed =(double) mydata_in_optimizer_copy.k_d_airspeed;
    double des_psi_dot =(double) mydata_in_optimizer_copy.psi_dot_cmd_ec;
    double current_accelerations[6] = {(double) mydata_in_optimizer_copy.modeled_ax_filtered,
                                       (double) mydata_in_optimizer_copy.modeled_ay_filtered,
                                       (double) mydata_in_optimizer_copy.modeled_az_filtered,
                                       (double) 0.0f,
                                       (double) 0.0f,
                                       (double) 0.0f};

    //Assign u_init from u_init_outer
    double u_init[NUM_ACT_IN_U_IN]; 
    for (int i=0; i< NUM_ACT_IN_U_IN; i++){
      u_init[i] = u_init_outer[i];
    }
    double use_u_init =(double) mydata_in_optimizer_copy.use_u_init_outer_loop;

    //Assign the du values manually: 
    double W_act_motor_du = (double) 1.0f; 
    double W_act_tilt_el_du = (double) 1.0f;
    double W_act_tilt_az_du = (double) 1.0f;
    double W_act_theta_du = (double) 1.0f;
    double W_act_phi_du = (double) 1.0f;
    double W_act_ailerons_du = (double) 1.0f;
    double gamma_quadratic_du2 = (double) 0.0f;

    //Assign the failure gains manually: 
    double W_act_motor_failure = (double) 20.0f; 
    double W_act_tilt_el_failure = (double) 0.0f;
    double W_act_tilt_az_failure = (double) 0.0f;
    double W_act_theta_failure = (double) 1.0f;
    double W_act_phi_failure = (double) 1.0f;
    double W_act_ailerons_failure = (double) 0.5f;

    double W_dv_1_failure = (double) 0.1f;
    double W_dv_2_failure = (double) 0.1f;
    double W_dv_3_failure = (double) 0.1f;
    double W_dv_4_failure = (double) 0.01f;
    double W_dv_5_failure = (double) 0.01f;
    double W_dv_6_failure = (double) 0.01f;

    double gamma_quadratic_du_failure = (double) 5e-7f;

    //Prepare output variables: 
    double u_out[NUM_ACT_IN_U_IN];
    double dv_pqr_dot_target[3];
    double theta_cmd_rad;
    double phi_cmd_rad;
    double acc_decrement_aero[6];
    double residuals[6];
    double elapsed_time;
    double N_iterations;
    double N_evaluation;
    double exitflag;

    //Replace the values of the pseudo control variables if the single loop controller is active:
    if(mydata_in_optimizer_copy.single_loop_controller > 0.5f){
      //Assign the angular pseudo control variables and the modeled angular accelerations:
      dv[3] = (double) mydata_in_optimizer_copy.pseudo_control_p_dot;
      dv[4] = (double) mydata_in_optimizer_copy.pseudo_control_q_dot;
      dv[5] = (double) mydata_in_optimizer_copy.pseudo_control_r_dot;
      current_accelerations[3] = (double) mydata_in_optimizer_copy.modeled_p_dot_filtered;
      current_accelerations[4] = (double) mydata_in_optimizer_copy.modeled_q_dot_filtered;
      current_accelerations[5] = (double) mydata_in_optimizer_copy.modeled_r_dot_filtered;
    }   

    //if verbose_outer_loop is set to 1, print the input variables:
    if(verbose_outer_loop){
      printf("\n OUTER LOOP INPUTS TO FUNCTION--------------------------------------------------- \n"); 
      printf("K_p_T = %f \n",(float) K_p_T);
      printf("K_p_M = %f \n",(float) K_p_M);
      printf("m = %f \n",(float) m);
      printf("I_xx = %f \n",(float) I_xx);
      printf("I_yy = %f \n",(float) I_yy);
      printf("I_zz = %f \n",(float) I_zz);
      printf("l_1 = %f \n",(float) l_1);
      printf("l_2 = %f \n",(float) l_2);
      printf("l_3 = %f \n",(float) l_3);
      printf("l_4 = %f \n",(float) l_4);
      printf("l_z = %f \n",(float) l_z);
      printf("Phi = %f \n",(float) Phi);
      printf("Theta = %f \n",(float) Theta);
      printf("Omega_1 = %f \n",(float) Omega_1);
      printf("Omega_2 = %f \n",(float) Omega_2);
      printf("Omega_3 = %f \n",(float) Omega_3);
      printf("Omega_4 = %f \n",(float) Omega_4);
      printf("b_1 = %f \n",(float) b_1);
      printf("b_2 = %f \n",(float) b_2);
      printf("b_3 = %f \n",(float) b_3);
      printf("b_4 = %f \n",(float) b_4);
      printf("g_1 = %f \n",(float) g_1);
      printf("g_2 = %f \n",(float) g_2);
      printf("g_3 = %f \n",(float) g_3);
      printf("g_4 = %f \n",(float) g_4);
      printf("delta_ailerons = %f \n",(float) delta_ailerons);
      printf("W_act_motor_const = %f \n",(float) W_act_motor_const);
      printf("W_act_motor_speed = %f \n",(float) W_act_motor_speed);
      printf("W_act_tilt_el_const = %f \n",(float) W_act_tilt_el_const);
      printf("W_act_tilt_el_speed = %f \n",(float) W_act_tilt_el_speed);
      printf("W_act_tilt_az_const = %f \n",(float) W_act_tilt_az_const);
      printf("W_act_tilt_az_speed = %f \n",(float) W_act_tilt_az_speed);
      printf("W_act_theta_const = %f \n",(float) W_act_theta_const);
      printf("W_act_theta_speed = %f \n",(float) W_act_theta_speed);
      printf("W_act_phi_const = %f \n",(float) W_act_phi_const);
      printf("W_act_phi_speed = %f \n",(float) W_act_phi_speed);
      printf("W_act_ailerons_const = %f \n",(float) W_act_ailerons_const);
      printf("W_act_ailerons_speed = %f \n",(float) W_act_ailerons_speed);
      printf("W_dv_1 = %f \n",(float) W_dv_1);
      printf("W_dv_2 = %f \n",(float) W_dv_2);
      printf("W_dv_3 = %f \n",(float) W_dv_3);
      printf("W_dv_4 = %f \n",(float) W_dv_4);
      printf("W_dv_5 = %f \n",(float) W_dv_5);
      printf("W_dv_6 = %f \n",(float) W_dv_6);
      printf("max_omega = %f \n",(float) max_omega);
      printf("min_omega = %f \n",(float) min_omega);
      printf("max_b = %f \n",(float) max_b);
      printf("min_b = %f \n",(float) min_b);
      printf("max_g = %f \n",(float) max_g);
      printf("min_g = %f \n",(float) min_g);
      printf("max_theta = %f \n",(float) max_theta);
      printf("min_theta = %f \n",(float) min_theta);
      printf("max_phi = %f \n",(float) max_phi);
      printf("max_delta_ailerons = %f \n",(float) max_delta_ailerons);
      printf("min_delta_ailerons = %f \n",(float) min_delta_ailerons);
      for (int i=0; i<6; i++){
        printf("dv[%d] = %f \n",i,(float) dv[i]);
      }
      printf("p = %f \n",(float) p);
      printf("q = %f \n",(float) q);
      printf("r = %f \n",(float) r);
      printf("Cm_zero = %f \n",(float) Cm_zero);
      printf("Cl_alpha = %f \n",(float) Cl_alpha);
      printf("Cd_zero = %f \n",(float) Cd_zero);
      printf("K_Cd = %f \n",(float) K_Cd);
      printf("Cm_alpha = %f \n",(float) Cm_alpha);
      printf("CL_aileron = %f \n",(float) CL_aileron);
      printf("rho = %f \n",(float) rho);
      printf("V = %f \n",(float) V);
      printf("S = %f \n",(float) S);
      printf("wing_chord = %f \n",(float) wing_chord);
      printf("flight_path_angle = %f \n",(float) flight_path_angle);
      printf("max_alpha = %f \n",(float) max_alpha);
      printf("min_alpha = %f \n",(float) min_alpha);
      printf("Beta = %f \n",(float) Beta);
      printf("gamma_quadratic_du = %f \n",(float) gamma_quadratic_du);
      printf("desired_motor_value = %f \n",(float) desired_motor_value);
      printf("desired_el_value = %f \n",(float) desired_el_value);
      printf("desired_az_value = %f \n",(float) desired_az_value);
      printf("desired_theta_value = %f \n",(float) desired_theta_value);
      printf("desired_phi_value = %f \n",(float) desired_phi_value);
      printf("desired_ailerons_value = %f \n",(float) desired_ailerons_value);
      printf("k_alt_tilt_constraint = %f \n",(float) k_alt_tilt_constraint);
      printf("min_alt_tilt_constraint = %f \n",(float) min_alt_tilt_constraint);
      printf("lidar_alt_corrected = %f \n",(float) lidar_alt_corrected);
      printf("approach_mode = %f \n",(float) approach_mode);
      printf("aoa_protection_speed = %f \n",(float) aoa_protection_speed);
      printf("vert_acc_margin = %f \n",(float) vert_acc_margin);
      printf("max_airspeed = %f \n",(float) max_airspeed);
      printf("p_body_current = %f \n",(float) p_body_current);
      printf("q_body_current = %f \n",(float) q_body_current);
      printf("r_body_current = %f \n",(float) r_body_current);
      printf("p_dot_current = %f \n",(float) p_dot_current);
      printf("q_dot_current = %f \n",(float) q_dot_current);
      printf("r_dot_current = %f \n",(float) r_dot_current);
      printf("phi_current = %f \n",(float) phi_current);
      printf("theta_current = %f \n",(float) theta_current);
      for (int i=0; i<3; i++){
        printf("angular_proportional_gains[%d] = %f \n",i,(float) angular_proportional_gains[i]);
      }
      for (int i=0; i<3; i++){
        printf("angular_derivative_gains[%d] = %f \n",i,(float) angular_derivative_gains[i]);
      }
      printf("theta_hard_max = %f \n",(float) theta_hard_max);
      printf("theta_hard_min = %f \n",(float) theta_hard_min);
      printf("phi_hard_max = %f \n",(float) phi_hard_max);
      printf("phi_hard_min = %f \n",(float) phi_hard_min);
      printf("k_d_airspeed = %f \n",(float) k_d_airspeed);
      printf("des_psi_dot = %f \n",(float) des_psi_dot);
      for (int i=0; i<6; i++){
        printf("current_accelerations[%d] = %f \n",i,(float) current_accelerations[i]);
      }
      printf("use_u_init = %f \n",(float) use_u_init);
      for (int i=0; i<NUM_ACT_IN_U_IN; i++){
        printf("u_init[%d] = %f \n",i,(float) u_init[i]);
      }
      printf("W_act_motor_du = %f \n",(float) W_act_motor_du);
      printf("W_act_tilt_el_du = %f \n",(float) W_act_tilt_el_du);
      printf("W_act_tilt_az_du = %f \n",(float) W_act_tilt_az_du);
      printf("W_act_theta_du = %f \n",(float) W_act_theta_du);
      printf("W_act_phi_du = %f \n",(float) W_act_phi_du);
      printf("W_act_ailerons_du = %f \n",(float) W_act_ailerons_du);
      printf("gamma_quadratic_du2 = %f \n",(float) gamma_quadratic_du2);
      printf("induced_failure = %f \n",(float) induced_failure);
      printf("W_act_motor_failure = %f \n",(float) W_act_motor_failure);
      printf("W_act_tilt_el_failure = %f \n",(float) W_act_tilt_el_failure);
      printf("W_act_tilt_az_failure = %f \n",(float) W_act_tilt_az_failure);
      printf("W_act_theta_failure = %f \n",(float) W_act_theta_failure);
      printf("W_act_phi_failure = %f \n",(float) W_act_phi_failure);
      printf("W_act_ailerons_failure = %f \n",(float) W_act_ailerons_failure);
      printf("W_dv_1_failure = %f \n",(float) W_dv_1_failure);
      printf("W_dv_2_failure = %f \n",(float) W_dv_2_failure);
      printf("W_dv_3_failure = %f \n",(float) W_dv_3_failure);
      printf("W_dv_4_failure = %f \n",(float) W_dv_4_failure);
      printf("W_dv_5_failure = %f \n",(float) W_dv_5_failure);
      printf("W_dv_6_failure = %f \n",(float) W_dv_6_failure);
      printf("gamma_quadratic_du_failure = %f \n",(float) gamma_quadratic_du_failure);
      printf("use_new_aero_model = %f \n",(float) mydata_in_optimizer_copy.use_new_aero_model);
    }

    if(mydata_in_optimizer_copy.use_new_aero_model > 0.5f){
      Nonlinear_controller_w_ail_new_aero_outer_loop(
          m, I_xx, I_yy, I_zz, l_1, l_2,
          l_3, l_4, l_z, Phi, Theta,
          Omega_1, Omega_2, Omega_3, Omega_4, b_1,
          b_2, b_3, b_4, g_1, g_2, g_3,
          g_4, delta_ailerons, W_act_motor_const,
          W_act_motor_speed, W_act_tilt_el_const,
          W_act_tilt_el_speed, W_act_tilt_az_const,
          W_act_tilt_az_speed, W_act_theta_const,
          W_act_theta_speed, W_act_phi_const, W_act_phi_speed,
          W_act_ailerons_const, W_act_ailerons_speed, W_dv_1,
          W_dv_2, W_dv_3, W_dv_4, W_dv_5, W_dv_6,
          max_omega, min_omega, max_b, min_b,
          max_g, min_g, max_theta, min_theta,
          max_phi, max_delta_ailerons, min_delta_ailerons,
          dv, p, q, r, Cm_zero,
          Cl_alpha, Cd_zero, K_Cd, Cm_alpha,
          CL_aileron, rho, V, S, wing_chord,
          flight_path_angle, max_alpha, min_alpha, Beta,
          gamma_quadratic_du, desired_motor_value,
          desired_el_value, desired_az_value,
          desired_theta_value, desired_phi_value,
          desired_ailerons_value, k_alt_tilt_constraint,
          min_alt_tilt_constraint, lidar_alt_corrected,
          approach_mode, verbose, aoa_protection_speed,
          vert_acc_margin, p_body_current, q_body_current,
          r_body_current, p_dot_current, q_dot_current,
          r_dot_current, phi_current, theta_current,
          angular_proportional_gains, angular_derivative_gains,
          theta_hard_max, theta_hard_min, phi_hard_max,
          phi_hard_min, k_d_airspeed, des_psi_dot,
          current_accelerations, u_init,
          use_u_init, induced_failure, W_act_motor_failure,
          W_act_tilt_el_failure, W_act_tilt_az_failure,
          W_act_theta_failure, W_act_phi_failure,
          W_act_ailerons_failure, W_dv_1_failure, W_dv_2_failure,
          W_dv_3_failure, W_dv_4_failure, W_dv_5_failure,
          W_dv_6_failure, gamma_quadratic_du_failure, power_Cd_0,
          power_Cd_a, prop_R, prop_Cd_0, prop_Cl_0,
          prop_Cd_a, prop_Cl_a, prop_delta, prop_sigma,
          prop_theta, max_airspeed, wing_span, u_out,
          dv_pqr_dot_target, acc_decrement_aero,
          residuals, &elapsed_time, &N_iterations,
          &N_evaluation, &exitflag);
    }
    else{   
      Nonlinear_controller_w_ail_basic_aero_outer_loop(
          K_p_T, K_p_M, m, I_xx, I_yy, I_zz,
          l_1, l_2, l_3, l_4, l_z, Phi,
          Theta, Omega_1, Omega_2, Omega_3,
          Omega_4, b_1, b_2, b_3, b_4, g_1,
          g_2, g_3, g_4, delta_ailerons,
          W_act_motor_const, W_act_motor_speed,
          W_act_tilt_el_const, W_act_tilt_el_speed,
          W_act_tilt_az_const, W_act_tilt_az_speed,
          W_act_theta_const, W_act_theta_speed, W_act_phi_const,
          W_act_phi_speed, W_act_ailerons_const,
          W_act_ailerons_speed, W_dv_1, W_dv_2, W_dv_3,
          W_dv_4, W_dv_5, W_dv_6, max_omega,
          min_omega, max_b, min_b, max_g, min_g,
          max_theta, min_theta, max_phi,
          max_delta_ailerons, min_delta_ailerons, dv,
          p, q, r, Cm_zero, Cl_alpha,
          Cd_zero, K_Cd, Cm_alpha, CL_aileron, rho,
          V, S, wing_chord, flight_path_angle,
          max_alpha, min_alpha, Beta, gamma_quadratic_du,
          desired_motor_value, desired_el_value,
          desired_az_value, desired_theta_value,
          desired_phi_value, desired_ailerons_value,
          k_alt_tilt_constraint, min_alt_tilt_constraint,
          lidar_alt_corrected, approach_mode, verbose,
          aoa_protection_speed, vert_acc_margin, p_body_current,
          q_body_current, r_body_current, p_dot_current,
          q_dot_current, r_dot_current, phi_current,
          theta_current, angular_proportional_gains,
          angular_derivative_gains, theta_hard_max,
          theta_hard_min, phi_hard_max, phi_hard_min,
          k_d_airspeed, des_psi_dot,
          current_accelerations, u_init,
          use_u_init, W_act_motor_du, W_act_tilt_el_du,
          W_act_tilt_az_du, W_act_theta_du, W_act_phi_du,
          W_act_ailerons_du, gamma_quadratic_du2,
          induced_failure, W_act_motor_failure,
          W_act_tilt_el_failure, W_act_tilt_az_failure,
          W_act_theta_failure, W_act_phi_failure,
          W_act_ailerons_failure, W_dv_1_failure, W_dv_2_failure,
          W_dv_3_failure, W_dv_4_failure, W_dv_5_failure,
          W_dv_6_failure, gamma_quadratic_du_failure, u_out,
          dv_pqr_dot_target, acc_decrement_aero, residuals, &elapsed_time,
          &N_iterations, &N_evaluation, &exitflag);
    }


    //Set as u_init_outer the u_out for the next iteration: 
    for (int i=0; i< NUM_ACT_IN_U_IN; i++){
      u_init_outer[i] = u_out[i];
    }

    //Fill the data strtucture to be sent to the inner loop
    struct outer_loop_output myouter_loop_output_copy;
    myouter_loop_output_copy.motor_1_cmd_rad_s = u_out[0];
    myouter_loop_output_copy.motor_2_cmd_rad_s = u_out[1];
    myouter_loop_output_copy.motor_3_cmd_rad_s = u_out[2];
    myouter_loop_output_copy.motor_4_cmd_rad_s = u_out[3];
    myouter_loop_output_copy.el_1_cmd_rad = u_out[4];
    myouter_loop_output_copy.el_2_cmd_rad = u_out[5];
    myouter_loop_output_copy.el_3_cmd_rad = u_out[6];
    myouter_loop_output_copy.el_4_cmd_rad = u_out[7];
    myouter_loop_output_copy.az_1_cmd_rad = u_out[8];
    myouter_loop_output_copy.az_2_cmd_rad = u_out[9];
    myouter_loop_output_copy.az_3_cmd_rad = u_out[10];
    myouter_loop_output_copy.az_4_cmd_rad = u_out[11];
    myouter_loop_output_copy.theta_cmd_rad = u_out[12];
    myouter_loop_output_copy.phi_cmd_rad = u_out[13];
    myouter_loop_output_copy.ailerons_cmd_rad = u_out[14];
    myouter_loop_output_copy.p_dot_cmd_rad_s = dv_pqr_dot_target[0];
    myouter_loop_output_copy.q_dot_cmd_rad_s = dv_pqr_dot_target[1];
    myouter_loop_output_copy.r_dot_cmd_rad_s = dv_pqr_dot_target[2];
    myouter_loop_output_copy.residual_ax = residuals[0];
    myouter_loop_output_copy.residual_ay = residuals[1];
    myouter_loop_output_copy.residual_az = residuals[2];
    myouter_loop_output_copy.residual_p_dot = residuals[3];
    myouter_loop_output_copy.residual_q_dot = residuals[4];
    myouter_loop_output_copy.residual_r_dot = residuals[5];
    myouter_loop_output_copy.exit_flag = exitflag;
    myouter_loop_output_copy.n_iterations = N_iterations;
    myouter_loop_output_copy.n_evaluations = N_evaluation;
    myouter_loop_output_copy.elapsed_time = elapsed_time;
    myouter_loop_output_copy.acc_decrement_aero_ax = acc_decrement_aero[0];
    myouter_loop_output_copy.acc_decrement_aero_ay = acc_decrement_aero[1];
    myouter_loop_output_copy.acc_decrement_aero_az = acc_decrement_aero[2];
    myouter_loop_output_copy.acc_decrement_aero_p_dot = acc_decrement_aero[3];
    myouter_loop_output_copy.acc_decrement_aero_q_dot = acc_decrement_aero[4];
    myouter_loop_output_copy.acc_decrement_aero_r_dot = acc_decrement_aero[5];

    if(verbose_outer_loop){
      printf("\n OUTER LOOP OUTPUTS ------------------------------------------------------ \n"); 
      printf("motor_1_cmd_rad_s = %f \n", (float) myouter_loop_output_copy.motor_1_cmd_rad_s);
      printf("motor_2_cmd_rad_s = %f \n", (float) myouter_loop_output_copy.motor_2_cmd_rad_s);
      printf("motor_3_cmd_rad_s = %f \n", (float) myouter_loop_output_copy.motor_3_cmd_rad_s);
      printf("motor_4_cmd_rad_s = %f \n", (float) myouter_loop_output_copy.motor_4_cmd_rad_s);
      printf("el_1_cmd_rad = %f \n", (float) myouter_loop_output_copy.el_1_cmd_rad);
      printf("el_2_cmd_rad = %f \n", (float) myouter_loop_output_copy.el_2_cmd_rad);
      printf("el_3_cmd_rad = %f \n", (float) myouter_loop_output_copy.el_3_cmd_rad);
      printf("el_4_cmd_rad = %f \n", (float) myouter_loop_output_copy.el_4_cmd_rad);
      printf("az_1_cmd_rad = %f \n", (float) myouter_loop_output_copy.az_1_cmd_rad);
      printf("az_2_cmd_rad = %f \n", (float) myouter_loop_output_copy.az_2_cmd_rad);
      printf("az_3_cmd_rad = %f \n", (float) myouter_loop_output_copy.az_3_cmd_rad);
      printf("az_4_cmd_rad = %f \n", (float) myouter_loop_output_copy.az_4_cmd_rad);
      printf("theta_cmd_rad = %f \n", (float) myouter_loop_output_copy.theta_cmd_rad);
      printf("phi_cmd_rad = %f \n", (float) myouter_loop_output_copy.phi_cmd_rad);
      printf("ailerons_cmd_rad = %f \n", (float) myouter_loop_output_copy.ailerons_cmd_rad);
      printf("p_dot_cmd_rad_s = %f \n", (float) myouter_loop_output_copy.p_dot_cmd_rad_s);
      printf("q_dot_cmd_rad_s = %f \n", (float) myouter_loop_output_copy.q_dot_cmd_rad_s);
      printf("r_dot_cmd_rad_s = %f \n", (float) myouter_loop_output_copy.r_dot_cmd_rad_s);
      printf("residual_ax = %f \n", (float) myouter_loop_output_copy.residual_ax);
      printf("residual_ay = %f \n", (float) myouter_loop_output_copy.residual_ay);
      printf("residual_az = %f \n", (float) myouter_loop_output_copy.residual_az);
      printf("residual_p_dot = %f \n", (float) myouter_loop_output_copy.residual_p_dot);
      printf("residual_q_dot = %f \n", (float) myouter_loop_output_copy.residual_q_dot);
      printf("residual_r_dot = %f \n", (float) myouter_loop_output_copy.residual_r_dot);
      printf("exit_flag = %f \n", (float) myouter_loop_output_copy.exit_flag);
      printf("n_iterations = %f \n", (float) myouter_loop_output_copy.n_iterations);
      printf("n_evaluations = %f \n", (float) myouter_loop_output_copy.n_evaluations);
      printf("elapsed_time = %f \n", (float) myouter_loop_output_copy.elapsed_time);
      printf("acc_decrement_aero_ax = %f \n", (float) myouter_loop_output_copy.acc_decrement_aero_ax);
      printf("acc_decrement_aero_ay = %f \n", (float) myouter_loop_output_copy.acc_decrement_aero_ay);
      printf("acc_decrement_aero_az = %f \n", (float) myouter_loop_output_copy.acc_decrement_aero_az);
      printf("acc_decrement_aero_p_dot = %f \n", (float) myouter_loop_output_copy.acc_decrement_aero_p_dot);
      printf("acc_decrement_aero_q_dot = %f \n", (float) myouter_loop_output_copy.acc_decrement_aero_q_dot);
      printf("acc_decrement_aero_r_dot = %f \n", (float) myouter_loop_output_copy.acc_decrement_aero_r_dot);
    }

    //Send the data to the inner loop
    pthread_mutex_lock(&mutex_outer_loop_output);
    memcpy(&myouter_loop_output, &myouter_loop_output_copy, sizeof(struct outer_loop_output));
    pthread_mutex_unlock(&mutex_outer_loop_output);

    //Wait until time is not at least refresh_time_outer_loop
    gettimeofday(&current_time, NULL);
    while(((current_time.tv_sec*1e6 + current_time.tv_usec) - (time_last_outer_loop.tv_sec*1e6 + time_last_outer_loop.tv_usec)) < refresh_time_outer_loop*1e6){
      usleep(5);
      gettimeofday(&current_time, NULL);
    }
    //Print performances if needed
    if(verbose_runtime){
      printf(" Effective refresh time outer loop = %f \n",(float) ((current_time.tv_sec*1e6 + current_time.tv_usec) - (time_last_outer_loop.tv_sec*1e6 + time_last_outer_loop.tv_usec)));
      fflush(stdout);
    }
    //Update time_last_outer_loop:
    gettimeofday(&time_last_outer_loop, NULL);
  }

}

void* fourth_thread() //Run the inner loop of the optimization code 
{

  while(1){ 

    //Retrieve the data from the filtering and assign variables thread
    struct data_in_optimizer mydata_in_optimizer_copy;
    pthread_mutex_lock(&mutex_optimizer_input);
    memcpy(&mydata_in_optimizer_copy, &mydata_in_optimizer, sizeof(struct data_in_optimizer));
    pthread_mutex_unlock(&mutex_optimizer_input);

    //Retrieve data output from outr loop
    struct outer_loop_output myouter_loop_output_copy;
    pthread_mutex_lock(&mutex_outer_loop_output);
    memcpy(&myouter_loop_output_copy, &myouter_loop_output, sizeof(struct outer_loop_output));
    pthread_mutex_unlock(&mutex_outer_loop_output);

    double verbose = verbose_inner_loop;

    //Check if u_init_inner was not initialized yet:
    if(u_init_inner[0] < 1){
      u_init_inner[0] = mydata_in_optimizer_copy.motor_1_state_filtered;
      u_init_inner[1] = mydata_in_optimizer_copy.motor_2_state_filtered;
      u_init_inner[2] = mydata_in_optimizer_copy.motor_3_state_filtered;
      u_init_inner[3] = mydata_in_optimizer_copy.motor_4_state_filtered;
      u_init_inner[4] = mydata_in_optimizer_copy.el_1_state_filtered;
      u_init_inner[5] = mydata_in_optimizer_copy.el_2_state_filtered;
      u_init_inner[6] = mydata_in_optimizer_copy.el_3_state_filtered;
      u_init_inner[7] = mydata_in_optimizer_copy.el_4_state_filtered;
      u_init_inner[8] = mydata_in_optimizer_copy.az_1_state_filtered;
      u_init_inner[9] = mydata_in_optimizer_copy.az_2_state_filtered;
      u_init_inner[10] = mydata_in_optimizer_copy.az_3_state_filtered;
      u_init_inner[11] = mydata_in_optimizer_copy.az_4_state_filtered;
      u_init_inner[12] = mydata_in_optimizer_copy.ailerons_state_filtered;
    }

    //Assign u_init from u_init_inner
    double u_init[NUM_ACT_IN_U_IN_INNER];
    for (int i=0; i< NUM_ACT_IN_U_IN_INNER; i++){
      u_init[i] = u_init_inner[i];
    }

    //Prepare input variables from the mydata_in_optimizer_copy structure: 
    double K_p_T =(double) mydata_in_optimizer_copy.K_p_T;
    double K_p_M =(double) mydata_in_optimizer_copy.K_p_M;
    double m =(double) mydata_in_optimizer_copy.m;
    double I_xx =(double) mydata_in_optimizer_copy.I_xx;
    double I_yy =(double) mydata_in_optimizer_copy.I_yy;
    double I_zz =(double) mydata_in_optimizer_copy.I_zz;
    double l_1 =(double) mydata_in_optimizer_copy.l_1;
    double l_2 =(double) mydata_in_optimizer_copy.l_2;
    double l_3 =(double) mydata_in_optimizer_copy.l_3;
    double l_4 =(double) mydata_in_optimizer_copy.l_4;
    double l_z =(double) mydata_in_optimizer_copy.l_z;
    double Phi =(double) mydata_in_optimizer_copy.phi_state_filtered;
    double Theta =(double) mydata_in_optimizer_copy.theta_state_filtered;
    double Omega_1 =(double) mydata_in_optimizer_copy.motor_1_state_filtered;
    double Omega_2 =(double) mydata_in_optimizer_copy.motor_2_state_filtered;
    double Omega_3 =(double) mydata_in_optimizer_copy.motor_3_state_filtered;
    double Omega_4 =(double) mydata_in_optimizer_copy.motor_4_state_filtered;
    double b_1 =(double) mydata_in_optimizer_copy.el_1_state_filtered;
    double b_2 =(double) mydata_in_optimizer_copy.el_2_state_filtered;
    double b_3 =(double) mydata_in_optimizer_copy.el_3_state_filtered;
    double b_4 =(double) mydata_in_optimizer_copy.el_4_state_filtered;
    double g_1 =(double) mydata_in_optimizer_copy.az_1_state_filtered;
    double g_2 =(double) mydata_in_optimizer_copy.az_2_state_filtered;
    double g_3 =(double) mydata_in_optimizer_copy.az_3_state_filtered;
    double g_4 =(double) mydata_in_optimizer_copy.az_4_state_filtered;
    double delta_ailerons =(double) mydata_in_optimizer_copy.ailerons_state_filtered;
    double W_act_motor_const =(double) mydata_in_optimizer_copy.W_act_motor_const;
    double W_act_motor_speed =(double) mydata_in_optimizer_copy.W_act_motor_speed;
    double W_act_tilt_el_const =(double) mydata_in_optimizer_copy.W_act_tilt_el_const;
    double W_act_tilt_el_speed =(double) mydata_in_optimizer_copy.W_act_tilt_el_speed;
    double W_act_tilt_az_const =(double) mydata_in_optimizer_copy.W_act_tilt_az_const;
    double W_act_tilt_az_speed =(double) mydata_in_optimizer_copy.W_act_tilt_az_speed;
    double W_act_ailerons_const =(double) mydata_in_optimizer_copy.W_act_ailerons_const;
    double W_act_ailerons_speed =(double) mydata_in_optimizer_copy.W_act_ailerons_speed;
    double W_dv_1 =(double) mydata_in_optimizer_copy.W_dv_1;
    double W_dv_2 =(double) mydata_in_optimizer_copy.W_dv_2;
    double W_dv_3 =(double) mydata_in_optimizer_copy.W_dv_3;
    double W_dv_4 =(double) mydata_in_optimizer_copy.W_dv_4;
    double W_dv_5 =(double) mydata_in_optimizer_copy.W_dv_5;
    double W_dv_6 =(double) mydata_in_optimizer_copy.W_dv_6;
    double max_omega =(double) mydata_in_optimizer_copy.max_omega;
    double min_omega =(double) mydata_in_optimizer_copy.min_omega;
    double max_b =(double) mydata_in_optimizer_copy.max_b;
    double min_b =(double) mydata_in_optimizer_copy.min_b;
    double max_g =(double) mydata_in_optimizer_copy.max_g;
    double min_g =(double) mydata_in_optimizer_copy.min_g;
    double max_theta =(double) mydata_in_optimizer_copy.max_theta;
    double min_theta =(double) mydata_in_optimizer_copy.min_theta;
    double max_phi =(double) mydata_in_optimizer_copy.max_phi;
    double max_delta_ailerons =(double) mydata_in_optimizer_copy.max_delta_ailerons;
    double min_delta_ailerons =(double) mydata_in_optimizer_copy.min_delta_ailerons;
    double dv[6] = {(double) mydata_in_optimizer_copy.pseudo_control_ax,
                    (double) mydata_in_optimizer_copy.pseudo_control_ay,
                    (double) mydata_in_optimizer_copy.pseudo_control_az,
                    (double) mydata_in_optimizer_copy.pseudo_control_p_dot,
                    (double) mydata_in_optimizer_copy.pseudo_control_q_dot,
                    (double) mydata_in_optimizer_copy.pseudo_control_r_dot};
    double p =(double) mydata_in_optimizer_copy.p_state_filtered;
    double q =(double) mydata_in_optimizer_copy.q_state_filtered;
    double r =(double) mydata_in_optimizer_copy.r_state_filtered;
    double Cm_zero =(double) mydata_in_optimizer_copy.Cm_zero;
    double Cl_alpha =(double) mydata_in_optimizer_copy.Cl_alpha;
    double Cd_zero =(double) mydata_in_optimizer_copy.Cd_zero;
    double K_Cd =(double) mydata_in_optimizer_copy.K_Cd;
    double Cm_alpha =(double) mydata_in_optimizer_copy.Cm_alpha;
    double CL_aileron =(double) mydata_in_optimizer_copy.CL_aileron;
    double rho =(double) mydata_in_optimizer_copy.rho;
    double V =(double) mydata_in_optimizer_copy.airspeed_state_filtered;
    double S =(double) mydata_in_optimizer_copy.S;
    double wing_chord =(double) mydata_in_optimizer_copy.wing_chord;
    double flight_path_angle =(double) mydata_in_optimizer_copy.gamma_state_filtered;
    double max_alpha =(double) mydata_in_optimizer_copy.max_alpha;
    double min_alpha =(double) mydata_in_optimizer_copy.min_alpha;
    double Beta = (double) mydata_in_optimizer_copy.beta_state_filtered;
    double gamma_quadratic_du =(double) mydata_in_optimizer_copy.gamma_quadratic_du;
    double desired_motor_value =(double) mydata_in_optimizer_copy.desired_motor_value;
    double desired_el_value =(double) mydata_in_optimizer_copy.desired_el_value;
    double desired_az_value =(double) mydata_in_optimizer_copy.desired_az_value;
    double desired_ailerons_value =(double) mydata_in_optimizer_copy.desired_ailerons_value;
    double k_alt_tilt_constraint =(double) mydata_in_optimizer_copy.k_alt_tilt_constraint;
    double min_alt_tilt_constraint =(double) mydata_in_optimizer_copy.min_alt_tilt_constraint;
    double lidar_alt_corrected =(double) mydata_in_optimizer_copy.lidar_alt_corrected;
    double approach_mode =(double) mydata_in_optimizer_copy.approach_boolean;
    double aoa_protection_speed =(double) mydata_in_optimizer_copy.aoa_protection_speed;
    double vert_acc_margin =(double) mydata_in_optimizer_copy.vert_acc_margin;
    double disable_acc_decrement_inner_loop =(double) mydata_in_optimizer_copy.disable_acc_decrement_inner_loop;

    double max_airspeed = (double) mydata_in_optimizer_copy.max_airspeed;
    double power_Cd_0 =(double) mydata_in_optimizer_copy.power_Cd_0;
    double power_Cd_a =(double) mydata_in_optimizer_copy.power_Cd_a;
    double prop_R =(double) mydata_in_optimizer_copy.prop_R;
    double prop_Cd_0 =(double) mydata_in_optimizer_copy.prop_Cd_0;
    double prop_Cl_0 =(double) mydata_in_optimizer_copy.prop_Cl_0;
    double prop_Cd_a =(double) mydata_in_optimizer_copy.prop_Cd_a;
    double prop_Cl_a =(double) mydata_in_optimizer_copy.prop_Cl_a;
    double prop_delta =(double) mydata_in_optimizer_copy.prop_delta;
    double prop_sigma =(double) mydata_in_optimizer_copy.prop_sigma;
    double prop_theta =(double) mydata_in_optimizer_copy.prop_theta;
    double wing_span =(double) mydata_in_optimizer_copy.wing_span;

    double current_accelerations[6] = {(double) mydata_in_optimizer_copy.modeled_ax_filtered,
                                       (double) mydata_in_optimizer_copy.modeled_ay_filtered,
                                       (double) mydata_in_optimizer_copy.modeled_az_filtered,
                                       (double) mydata_in_optimizer_copy.modeled_p_dot_filtered,
                                       (double) mydata_in_optimizer_copy.modeled_q_dot_filtered,
                                       (double) mydata_in_optimizer_copy.modeled_r_dot_filtered};

    //Remove aerodynamic accelerations if needed:
    if(disable_acc_decrement_inner_loop < 0.5f){
      current_accelerations[0] -= myouter_loop_output_copy.acc_decrement_aero_ax;
      current_accelerations[1] -= myouter_loop_output_copy.acc_decrement_aero_ay;
      current_accelerations[2] -= myouter_loop_output_copy.acc_decrement_aero_az;
      current_accelerations[3] -= myouter_loop_output_copy.acc_decrement_aero_p_dot;
      current_accelerations[4] -= myouter_loop_output_copy.acc_decrement_aero_q_dot;
      current_accelerations[5] -= myouter_loop_output_copy.acc_decrement_aero_r_dot;
    }

    double induced_failure =(double) mydata_in_optimizer_copy.failure_mode;

    double use_u_init =(double) mydata_in_optimizer_copy.use_u_init_inner_loop;

    //Retrieve pqr_dot_outer_loop from the outer loop structure: 
    double pqr_dot_outer_loop[3] = {(double) myouter_loop_output_copy.p_dot_cmd_rad_s, 
                                    (double) myouter_loop_output_copy.q_dot_cmd_rad_s, 
                                    (double) myouter_loop_output_copy.r_dot_cmd_rad_s};

    if(mydata_in_optimizer_copy.use_received_ang_ref_in_inner_loop > 0.5f){
      pqr_dot_outer_loop[0] = mydata_in_optimizer_copy.pseudo_control_p_dot;
      pqr_dot_outer_loop[1] = mydata_in_optimizer_copy.pseudo_control_q_dot;
      pqr_dot_outer_loop[2] = mydata_in_optimizer_copy.pseudo_control_r_dot;
    }                              

    //Assign the du values manually: 
    double W_act_motor_du = (double) 1.0f; 
    double W_act_tilt_el_du = (double) 1.0f;
    double W_act_tilt_az_du = (double) 1.0f;
    double W_act_ailerons_du = (double) 1.0f;
    double gamma_quadratic_du2 = (double) 0.0f;

    //Assign the failure gains manually: 
    double W_act_motor_failure = (double) 10.0f; 
    double W_act_tilt_el_failure = (double) 0.0f;
    double W_act_tilt_az_failure = (double) 0.0f;
    double W_act_ailerons_failure = (double) 0.5f;

    double W_dv_1_failure = (double) 0.01f;
    double W_dv_2_failure = (double) 0.01f;
    double W_dv_3_failure = (double) 0.01f;
    double W_dv_4_failure = (double) 0.1f;
    double W_dv_5_failure = (double) 0.1f;
    double W_dv_6_failure = (double) 0.1f;
    double gamma_quadratic_du_failure = (double) 5e-7f;

    //Prepare output variables: 
    double u_out[NUM_ACT_IN_U_IN_INNER];
    double residuals[6];
    double elapsed_time;
    double N_iterations;
    double N_evaluation;
    double exitflag;

    //if verbose_inner_loop is activated, print the input variables:
    if(verbose_inner_loop){
      printf("\n INNER LOOP INPUTS ------------------------------------------------------ \n"); 
      printf("K_p_T = %f \n",(float) K_p_T);
      printf("K_p_M = %f \n",(float) K_p_M);
      printf("m = %f \n",(float) m);
      printf("I_xx = %f \n",(float) I_xx);
      printf("I_yy = %f \n",(float) I_yy);
      printf("I_zz = %f \n",(float) I_zz);
      printf("l_1 = %f \n",(float) l_1);
      printf("l_2 = %f \n",(float) l_2);
      printf("l_3 = %f \n",(float) l_3);
      printf("l_4 = %f \n",(float) l_4);
      printf("l_z = %f \n",(float) l_z);
      printf("Phi = %f \n",(float) Phi);
      printf("Theta = %f \n",(float) Theta);
      printf("Omega_1 = %f \n",(float) Omega_1);
      printf("Omega_2 = %f \n",(float) Omega_2);
      printf("Omega_3 = %f \n",(float) Omega_3);
      printf("Omega_4 = %f \n",(float) Omega_4);
      printf("b_1 = %f \n",(float) b_1);
      printf("b_2 = %f \n",(float) b_2);
      printf("b_3 = %f \n",(float) b_3);
      printf("b_4 = %f \n",(float) b_4);
      printf("g_1 = %f \n",(float) g_1);
      printf("g_2 = %f \n",(float) g_2);
      printf("g_3 = %f \n",(float) g_3);
      printf("g_4 = %f \n",(float) g_4);
      printf("delta_ailerons = %f \n",(float) delta_ailerons);
      printf("W_act_motor_const = %f \n",(float) W_act_motor_const);
      printf("W_act_motor_speed = %f \n",(float) W_act_motor_speed);
      printf("W_act_tilt_el_const = %f \n",(float) W_act_tilt_el_const);
      printf("W_act_tilt_el_speed = %f \n",(float) W_act_tilt_el_speed);
      printf("W_act_tilt_az_const = %f \n",(float) W_act_tilt_az_const);
      printf("W_act_tilt_az_speed = %f \n",(float) W_act_tilt_az_speed);
      printf("W_act_ailerons_const = %f \n",(float) W_act_ailerons_const);
      printf("W_act_ailerons_speed = %f \n",(float) W_act_ailerons_speed);
      printf("W_dv_1 = %f \n",(float) W_dv_1);
      printf("W_dv_2 = %f \n",(float) W_dv_2);
      printf("W_dv_3 = %f \n",(float) W_dv_3);
      printf("W_dv_4 = %f \n",(float) W_dv_4);
      printf("W_dv_5 = %f \n",(float) W_dv_5);
      printf("W_dv_6 = %f \n",(float) W_dv_6);
      printf("max_omega = %f \n",(float) max_omega);
      printf("min_omega = %f \n",(float) min_omega);
      printf("max_b = %f \n",(float) max_b);
      printf("min_b = %f \n",(float) min_b);
      printf("max_g = %f \n",(float) max_g);
      printf("min_g = %f \n",(float) min_g);
      printf("max_theta = %f \n",(float) max_theta);
      printf("min_theta = %f \n",(float) min_theta);
      printf("max_phi = %f \n",(float) max_phi);
      printf("max_delta_ailerons = %f \n",(float) max_delta_ailerons);
      printf("min_delta_ailerons = %f \n",(float) min_delta_ailerons); 
      for (int i=0; i<6; i++){
        printf("dv[%d] = %f \n",i,(float) dv[i]);
      }
      printf("p = %f \n",(float) p);
      printf("q = %f \n",(float) q);
      printf("r = %f \n",(float) r);
      printf("Cm_zero = %f \n",(float) Cm_zero);
      printf("Cl_alpha = %f \n",(float) Cl_alpha);
      printf("Cd_zero = %f \n",(float) Cd_zero);
      printf("K_Cd = %f \n",(float) K_Cd);
      printf("Cm_alpha = %f \n",(float) Cm_alpha);
      printf("CL_aileron = %f \n",(float) CL_aileron);
      printf("rho = %f \n",(float) rho);
      printf("V = %f \n",(float) V);
      printf("S = %f \n",(float) S);
      printf("wing_chord = %f \n",(float) wing_chord);
      printf("flight_path_angle = %f \n",(float) flight_path_angle);
      printf("max_alpha = %f \n",(float) max_alpha);
      printf("min_alpha = %f \n",(float) min_alpha);
      printf("Beta = %f \n",(float) Beta);
      printf("gamma_quadratic_du = %f \n",(float) gamma_quadratic_du);
      printf("desired_motor_value = %f \n",(float) desired_motor_value);
      printf("desired_el_value = %f \n",(float) desired_el_value);
      printf("desired_az_value = %f \n",(float) desired_az_value);
      printf("desired_ailerons_value = %f \n",(float) desired_ailerons_value);
      printf("k_alt_tilt_constraint = %f \n",(float) k_alt_tilt_constraint);
      printf("min_alt_tilt_constraint = %f \n",(float) min_alt_tilt_constraint);
      printf("lidar_alt_corrected = %f \n",(float) lidar_alt_corrected);
      printf("approach_mode = %f \n",(float) approach_mode);
      printf("verbose = %f \n",(float) verbose);
      printf("vert_acc_margin = %f \n",(float) vert_acc_margin);
      printf("Current accelerations, eventually corrected by already achieved aerodynamic accelerations: \n");
      for (int i=0; i<6; i++){
        printf("current_accelerations[%d] = %f \n",i,(float) current_accelerations[i]);
      }
      printf("u_init = ");
      for (int i=0; i< NUM_ACT_IN_U_IN_INNER; i++){
        printf("%f ",(float) u_init[i]);
      }
      printf("\n");
      printf("use_u_init = %f \n",(float) use_u_init);
      printf("pqr_dot_outer_loop = ");
      for (int i=0; i<3; i++){
        printf("%f ",(float) pqr_dot_outer_loop[i]);
      }
      printf("\n");
      printf("W_act_motor_du = %f \n",(float) W_act_motor_du);
      printf("W_act_tilt_el_du = %f \n",(float) W_act_tilt_el_du);
      printf("W_act_tilt_az_du = %f \n",(float) W_act_tilt_az_du);
      printf("W_act_ailerons_du = %f \n",(float) W_act_ailerons_du);
      printf("gamma_quadratic_du2 = %f \n",(float) gamma_quadratic_du2);
      printf("induced_failure = %f \n",(float) induced_failure);
      printf("W_act_motor_failure = %f \n",(float) W_act_motor_failure);
      printf("W_act_tilt_el_failure = %f \n",(float) W_act_tilt_el_failure);
      printf("W_act_tilt_az_failure = %f \n",(float) W_act_tilt_az_failure);
      printf("W_act_ailerons_failure = %f \n",(float) W_act_ailerons_failure);
      printf("W_dv_1_failure = %f \n",(float) W_dv_1_failure);
      printf("W_dv_2_failure = %f \n",(float) W_dv_2_failure);
      printf("W_dv_3_failure = %f \n",(float) W_dv_3_failure);
      printf("W_dv_4_failure = %f \n",(float) W_dv_4_failure);
      printf("W_dv_5_failure = %f \n",(float) W_dv_5_failure);
      printf("W_dv_6_failure = %f \n",(float) W_dv_6_failure);
      printf("gamma_quadratic_du_failure = %f \n",(float) gamma_quadratic_du_failure);
      printf("use_new_aero_model = %f \n",(float) mydata_in_optimizer_copy.use_new_aero_model);
      printf("power_Cd_0 = %f \n",(float) power_Cd_0);
      printf("power_Cd_a = %f \n",(float) power_Cd_a);
      printf("prop_R = %f \n",(float) prop_R);
      printf("prop_Cd_0 = %f \n",(float) prop_Cd_0);
      printf("prop_Cl_0 = %f \n",(float) prop_Cl_0);
      printf("prop_Cd_a = %f \n",(float) prop_Cd_a);
      printf("prop_Cl_a = %f \n",(float) prop_Cl_a);
      printf("prop_delta = %f \n",(float) prop_delta);
      printf("prop_sigma = %f \n",(float) prop_sigma);
      printf("prop_theta = %f \n",(float) prop_theta);
      printf("max_airspeed = %f \n",(float) max_airspeed);
    }

    // Check if the control allocation is running in single loop mode:
    if(mydata_in_optimizer_copy.single_loop_controller > 0.5f){
      //Assign u_out from u_out of the outer loop:
      u_out[0] = myouter_loop_output_copy.motor_1_cmd_rad_s;
      u_out[1] = myouter_loop_output_copy.motor_2_cmd_rad_s;
      u_out[2] = myouter_loop_output_copy.motor_3_cmd_rad_s;
      u_out[3] = myouter_loop_output_copy.motor_4_cmd_rad_s;
      u_out[4] = myouter_loop_output_copy.el_1_cmd_rad;
      u_out[5] = myouter_loop_output_copy.el_2_cmd_rad;
      u_out[6] = myouter_loop_output_copy.el_3_cmd_rad;
      u_out[7] = myouter_loop_output_copy.el_4_cmd_rad;
      u_out[8] = myouter_loop_output_copy.az_1_cmd_rad;
      u_out[9] = myouter_loop_output_copy.az_2_cmd_rad;
      u_out[10] = myouter_loop_output_copy.az_3_cmd_rad;
      u_out[11] = myouter_loop_output_copy.az_4_cmd_rad;
      u_out[12] = myouter_loop_output_copy.ailerons_cmd_rad;
      //Assign residuals from outer loop: 
      residuals[0] = myouter_loop_output_copy.residual_ax;
      residuals[1] = myouter_loop_output_copy.residual_ay;
      residuals[2] = myouter_loop_output_copy.residual_az;
      residuals[3] = myouter_loop_output_copy.residual_p_dot;
      residuals[4] = myouter_loop_output_copy.residual_q_dot;
      residuals[5] = myouter_loop_output_copy.residual_r_dot;
      //Assign the elapsed time, number of iterations and number of evaluations from the outer loop:
      elapsed_time = myouter_loop_output_copy.elapsed_time;
      N_iterations = myouter_loop_output_copy.n_iterations;
      N_evaluation = myouter_loop_output_copy.n_evaluations;
      exitflag = myouter_loop_output_copy.exit_flag;
    }
    //If the control allocation is not running in single loop mode, run the inner loop:
    else{   
      if(mydata_in_optimizer_copy.use_new_aero_model > 0.5f){
        Nonlinear_controller_w_ail_new_aero_inner_loop(
            m, I_xx, I_yy, I_zz, l_1, l_2, l_3, l_4, l_z, Phi, Theta,
            Omega_1, Omega_2, Omega_3, Omega_4, b_1, b_2, b_3, b_4, g_1, g_2, g_3,
            g_4, delta_ailerons, W_act_motor_const, W_act_motor_speed,
            W_act_tilt_el_const, W_act_tilt_el_speed, W_act_tilt_az_const,
            W_act_tilt_az_speed, W_act_ailerons_const, W_act_ailerons_speed,
            W_dv_1, W_dv_2, W_dv_3, W_dv_4, W_dv_5, W_dv_6, max_omega, min_omega,
            max_b, min_b, max_g, min_g, max_delta_ailerons, min_delta_ailerons,
            dv, p, q, r, Cm_zero, Cl_alpha, Cd_zero, K_Cd, Cm_alpha, CL_aileron,
            rho, V, S, wing_chord, flight_path_angle, Beta, gamma_quadratic_du,
            desired_motor_value, desired_el_value, desired_az_value,
            desired_ailerons_value, k_alt_tilt_constraint, min_alt_tilt_constraint,
            lidar_alt_corrected, approach_mode, verbose, vert_acc_margin,
            current_accelerations, u_init, use_u_init, pqr_dot_outer_loop,
            induced_failure, W_act_motor_failure, W_act_tilt_el_failure,
            W_act_tilt_az_failure, W_act_ailerons_failure, W_dv_1_failure,
            W_dv_2_failure, W_dv_3_failure, W_dv_4_failure, W_dv_5_failure,
            W_dv_6_failure, gamma_quadratic_du_failure, power_Cd_0, power_Cd_a,
            prop_R, prop_Cd_0, prop_Cl_0, prop_Cd_a, prop_Cl_a, prop_delta,
            prop_sigma, prop_theta, max_airspeed, wing_span, u_out, residuals,
            &elapsed_time, &N_iterations, &N_evaluation, &exitflag);
      }
      else{
        Nonlinear_controller_w_ail_basic_aero_inner_loop(
            K_p_T, K_p_M, m, I_xx, I_yy, I_zz,
            l_1, l_2, l_3, l_4, l_z, Phi,
            Theta, Omega_1, Omega_2, Omega_3,
            Omega_4, b_1, b_2, b_3, b_4, g_1,
            g_2, g_3, g_4, delta_ailerons,
            W_act_motor_const, W_act_motor_speed,
            W_act_tilt_el_const, W_act_tilt_el_speed,
            W_act_tilt_az_const, W_act_tilt_az_speed,
            W_act_ailerons_const, W_act_ailerons_speed, W_dv_1,
            W_dv_2, W_dv_3, W_dv_4, W_dv_5, W_dv_6,
            max_omega, min_omega, max_b, min_b,
            max_g, min_g, max_delta_ailerons,
            min_delta_ailerons, dv, p, q, r,
            Cm_zero, Cl_alpha, Cd_zero, K_Cd,
            Cm_alpha, CL_aileron, rho, V, S,
            wing_chord, flight_path_angle, Beta,
            gamma_quadratic_du, desired_motor_value,
            desired_el_value, desired_az_value,
            desired_ailerons_value, k_alt_tilt_constraint,
            min_alt_tilt_constraint, lidar_alt_corrected,
            approach_mode, verbose, vert_acc_margin,
            current_accelerations, u_init,
            use_u_init, pqr_dot_outer_loop,
            W_act_motor_du, W_act_tilt_el_du, W_act_tilt_az_du,
            W_act_ailerons_du, gamma_quadratic_du2,
            induced_failure, W_act_motor_failure,
            W_act_tilt_el_failure, W_act_tilt_az_failure,
            W_act_ailerons_failure, W_dv_1_failure, W_dv_2_failure,
            W_dv_3_failure, W_dv_4_failure, W_dv_5_failure,
            W_dv_6_failure, gamma_quadratic_du_failure, u_out,
            residuals, &elapsed_time, &N_iterations,
            &N_evaluation, &exitflag);
      }
    }

    //Set as u_init_inner the u_out for the next iteration: 
    for (int i=0; i< NUM_ACT_IN_U_IN_INNER; i++){
      u_init_inner[i] = u_out[i];
    }

    //Fill the data strtucture to be sent to the autopilot
    struct am7_data_out myam7_data_out_copy;
    //Convert the function output into integer to be transmitted to the autopilot again: 
    myam7_data_out_copy.motor_1_cmd_int = (int16_T) (u_out[0]*1e1);
    myam7_data_out_copy.motor_2_cmd_int = (int16_T) (u_out[1]*1e1);
    myam7_data_out_copy.motor_3_cmd_int = (int16_T) (u_out[2]*1e1);
    myam7_data_out_copy.motor_4_cmd_int = (int16_T) (u_out[3]*1e1);
    myam7_data_out_copy.el_1_cmd_int = (int16_T) (u_out[4]*1e2*180/M_PI);
    myam7_data_out_copy.el_2_cmd_int = (int16_T) (u_out[5]*1e2*180/M_PI);
    myam7_data_out_copy.el_3_cmd_int = (int16_T) (u_out[6]*1e2*180/M_PI);
    myam7_data_out_copy.el_4_cmd_int = (int16_T) (u_out[7]*1e2*180/M_PI);
    myam7_data_out_copy.az_1_cmd_int = (int16_T) (u_out[8]*1e2*180/M_PI);
    myam7_data_out_copy.az_2_cmd_int = (int16_T) (u_out[9]*1e2*180/M_PI);
    myam7_data_out_copy.az_3_cmd_int = (int16_T) (u_out[10]*1e2*180/M_PI);
    myam7_data_out_copy.az_4_cmd_int = (int16_T) (u_out[11]*1e2*180/M_PI);
    myam7_data_out_copy.ailerons_cmd_int = (int16_T) (u_out[12]*1e2*180/M_PI);
    myam7_data_out_copy.theta_cmd_int = (int16_T) (myouter_loop_output_copy.theta_cmd_rad*1e2*180/M_PI);
    myam7_data_out_copy.phi_cmd_int = (int16_T) (myouter_loop_output_copy.phi_cmd_rad*1e2*180/M_PI);
    myam7_data_out_copy.residual_ax_int = (int16_T) (residuals[0]*1e2);
    myam7_data_out_copy.residual_ay_int = (int16_T) (residuals[1]*1e2);
    myam7_data_out_copy.residual_az_int = (int16_T) (residuals[2]*1e2);
    myam7_data_out_copy.residual_p_dot_int = (int16_T) (residuals[3]*1e1*180/M_PI);
    myam7_data_out_copy.residual_q_dot_int = (int16_T) (residuals[4]*1e1*180/M_PI);
    myam7_data_out_copy.residual_r_dot_int = (int16_T) (residuals[5]*1e1*180/M_PI);
    //Add optimization info from outer loop (outer loop structure): 
    myam7_data_out_copy.exit_flag_optimizer_outer = (int16_T) (myouter_loop_output_copy.exit_flag);
    myam7_data_out_copy.elapsed_time_us_outer = (uint16_T) (myouter_loop_output_copy.elapsed_time * 1e6);
    myam7_data_out_copy.n_iteration_outer = (uint16_T) (myouter_loop_output_copy.n_iterations);
    myam7_data_out_copy.n_evaluation_outer = (uint16_T) (myouter_loop_output_copy.n_evaluations);
    //Add optimization info from inner loop (inner loop structure):
    myam7_data_out_copy.exit_flag_optimizer_inner = (int16_T) (exitflag);
    myam7_data_out_copy.elapsed_time_us_inner = (uint16_T) (elapsed_time * 1e6);
    myam7_data_out_copy.n_iteration_inner = (uint16_T) (N_iterations);
    myam7_data_out_copy.n_evaluation_inner = (uint16_T) (N_evaluation);

    //Add to the structure the evaluated modeled values:
    myam7_data_out_copy.modeled_ax_int = (int16_T) (mydata_in_optimizer_copy.modeled_ax_filtered*1e2);
    myam7_data_out_copy.modeled_ay_int = (int16_T) (mydata_in_optimizer_copy.modeled_ay_filtered*1e2);
    myam7_data_out_copy.modeled_az_int = (int16_T) (mydata_in_optimizer_copy.modeled_az_filtered*1e2);
    myam7_data_out_copy.modeled_p_dot_int = (int16_T) (mydata_in_optimizer_copy.modeled_p_dot_filtered*1e1*180/M_PI);
    myam7_data_out_copy.modeled_q_dot_int = (int16_T) (mydata_in_optimizer_copy.modeled_q_dot_filtered*1e1*180/M_PI);
    myam7_data_out_copy.modeled_r_dot_int = (int16_T) (mydata_in_optimizer_copy.modeled_r_dot_filtered*1e1*180/M_PI);

    //Print submitted data if needed
    if(verbose_submitted_data){
      printf("\n REAL TIME VARIABLES OUT------------------------------------------------------ \n"); 
      printf(" motor_1_cmd_rad_s = %f \n",(float) myam7_data_out_copy.motor_1_cmd_int*1e-1);
      printf(" motor_2_cmd_rad_s = %f \n",(float) myam7_data_out_copy.motor_2_cmd_int*1e-1);
      printf(" motor_3_cmd_rad_s = %f \n",(float) myam7_data_out_copy.motor_3_cmd_int*1e-1);
      printf(" motor_4_cmd_rad_s = %f \n",(float) myam7_data_out_copy.motor_4_cmd_int*1e-1);
      printf(" el_1_cmd_deg = %f \n",(float) myam7_data_out_copy.el_1_cmd_int*1e-2);
      printf(" el_2_cmd_deg = %f \n",(float) myam7_data_out_copy.el_2_cmd_int*1e-2);
      printf(" el_3_cmd_deg = %f \n",(float) myam7_data_out_copy.el_3_cmd_int*1e-2);
      printf(" el_4_cmd_deg = %f \n",(float) myam7_data_out_copy.el_4_cmd_int*1e-2);
      printf(" az_1_cmd_deg = %f \n",(float) myam7_data_out_copy.az_1_cmd_int*1e-2);
      printf(" az_2_cmd_deg = %f \n",(float) myam7_data_out_copy.az_2_cmd_int*1e-2);
      printf(" az_3_cmd_deg = %f \n",(float) myam7_data_out_copy.az_3_cmd_int*1e-2);
      printf(" az_4_cmd_deg = %f \n",(float) myam7_data_out_copy.az_4_cmd_int*1e-2);
      printf(" ailerons_cmd_deg = %f \n",(float) myam7_data_out_copy.ailerons_cmd_int*1e-2);
      printf("modeled_ax = %f \n",(float) myam7_data_out_copy.modeled_ax_int*1e-2);
      printf("modeled_ay = %f \n",(float) myam7_data_out_copy.modeled_ay_int*1e-2);
      printf("modeled_az = %f \n",(float) myam7_data_out_copy.modeled_az_int*1e-2);
      printf("modeled_p_dot = %f \n",(float) myam7_data_out_copy.modeled_p_dot_int*1e-1);
      printf("modeled_q_dot = %f \n",(float) myam7_data_out_copy.modeled_q_dot_int*1e-1);
      printf("modeled_r_dot = %f \n",(float) myam7_data_out_copy.modeled_r_dot_int*1e-1);
      printf("theta_cmd_deg = %f \n",(float) myam7_data_out_copy.theta_cmd_int*1e-2);
      printf("phi_cmd_deg = %f \n",(float) myam7_data_out_copy.phi_cmd_int*1e-2);
      printf("residual_ax = %f \n",(float) myam7_data_out_copy.residual_ax_int*1e-2);
      printf("residual_ay = %f \n",(float) myam7_data_out_copy.residual_ay_int*1e-2);
      printf("residual_az = %f \n",(float) myam7_data_out_copy.residual_az_int*1e-2);
      printf("residual_p_dot = %f \n",(float) myam7_data_out_copy.residual_p_dot_int*1e-1);
      printf("residual_q_dot = %f \n",(float) myam7_data_out_copy.residual_q_dot_int*1e-1);
      printf("residual_r_dot = %f \n",(float) myam7_data_out_copy.residual_r_dot_int*1e-1);
      printf("exit_flag_optimizer_outer = %d \n",(int) myam7_data_out_copy.exit_flag_optimizer_outer);
      printf("elapsed_time_us_outer = %d \n",(int) myam7_data_out_copy.elapsed_time_us_outer);
      printf("n_iteration_outer = %d \n",(int) myam7_data_out_copy.n_iteration_outer);
      printf("n_evaluation_outer = %d \n",(int) myam7_data_out_copy.n_evaluation_outer);
      printf("exit_flag_optimizer_inner = %d \n",(int) myam7_data_out_copy.exit_flag_optimizer_inner);
      printf("elapsed_time_us_inner = %d \n",(int) myam7_data_out_copy.elapsed_time_us_inner);
      printf("n_iteration_inner = %d \n",(int) myam7_data_out_copy.n_iteration_inner);
      printf("n_evaluation_inner = %d \n",(int) myam7_data_out_copy.n_evaluation_inner);
      printf(" \n\n\n");
      fflush(stdout);
    }

    //Assign the rolling messages to the appropriate value [dummy]
    float extra_data_out_copy[255];
    extra_data_out_copy[0] = -1.0; 
    extra_data_out_copy[1] = -1.0; 
    
    //Retrieve detection from aruco and assign to the output structure
    struct marker_detection_t aruco_detection_copy; 
  
    pthread_mutex_lock(&mutex_aruco);
    memcpy(&aruco_detection_copy, &aruco_detection, sizeof(struct marker_detection_t));
    double last_ping_time_aruco_local = last_ping_time_aruco;
    pthread_mutex_unlock(&mutex_aruco); 

    //Compare the time of the last ping with the current time to check if the system is still alive:
    struct timespec ts;
    clock_gettime(CLOCK_BOOTTIME, &ts);
    double current_timestamp = ts.tv_sec + ts.tv_nsec*1e-9;
    if((current_timestamp - last_ping_time_aruco_local) > 3.0f){
      //If the system does not communicate anything for more than 3 seconds, set the system status to -1.
      aruco_detection_copy.system_status = -10;
      if(verbose_aruco){
        fprintf(stderr,"Aruco system is not communicating or it is inoperative. \n");
      }
    }
    myam7_data_out_copy.aruco_detection_timestamp = aruco_detection_copy.timestamp_detection;
    myam7_data_out_copy.aruco_NED_pos_x = aruco_detection_copy.NED_pos_x;
    myam7_data_out_copy.aruco_NED_pos_y = aruco_detection_copy.NED_pos_y;
    myam7_data_out_copy.aruco_NED_pos_z = aruco_detection_copy.NED_pos_z;
    myam7_data_out_copy.aruco_system_status = aruco_detection_copy.system_status;
    
    //Retrieve detection from sixdof and assign to the output structure
    struct marker_detection_t sixdof_detection_copy;
    pthread_mutex_lock(&mutex_sixdof);
    memcpy(&sixdof_detection_copy, &sixdof_detection, sizeof(struct marker_detection_t));
    double last_ping_time_sixdof_local = last_ping_time_sixdof; 
    pthread_mutex_unlock(&mutex_sixdof);

    //Compare the time of the last ping with the current time to check if the system is still alive:
    clock_gettime(CLOCK_BOOTTIME, &ts);
    current_timestamp = ts.tv_sec + ts.tv_nsec*1e-9; 
    if((current_timestamp - last_ping_time_sixdof_local) > 3.0f){
      //If the system does not communicate anything for more than 3 seconds, set the system status to -1.
      sixdof_detection_copy.system_status = -10; 
      if(verbose_sixdof){
        fprintf(stderr,"Sixdof system is not communicating or it is inoperative. \n");
      }
    }

    myam7_data_out_copy.sixdof_detection_timestamp = sixdof_detection_copy.timestamp_detection;
    myam7_data_out_copy.sixdof_NED_pos_x = sixdof_detection_copy.NED_pos_x;
    myam7_data_out_copy.sixdof_NED_pos_y = sixdof_detection_copy.NED_pos_y;
    myam7_data_out_copy.sixdof_NED_pos_z = sixdof_detection_copy.NED_pos_z;
    myam7_data_out_copy.sixdof_relative_phi = (int16_t) (sixdof_detection_copy.relative_phi_rad * 180/M_PI * 1e2);
    myam7_data_out_copy.sixdof_relative_theta = (int16_t) (sixdof_detection_copy.relative_theta_rad * 180/M_PI * 1e2);
    myam7_data_out_copy.sixdof_relative_psi = (int16_t) (sixdof_detection_copy.relative_psi_rad * 180/M_PI * 1e2);
    myam7_data_out_copy.sixdof_system_status = sixdof_detection_copy.system_status;
 
    //Copy out structure
    pthread_mutex_lock(&mutex_am7);
    memcpy(&myam7_data_out, &myam7_data_out_copy, sizeof(struct am7_data_out));
    memcpy(&extra_data_out, &extra_data_out_copy, sizeof(extra_data_out));
    pthread_mutex_unlock(&mutex_am7);   

    //Wait until time is not at least refresh_time_optimizer
    gettimeofday(&current_time, NULL);
    while(((current_time.tv_sec*1e6 + current_time.tv_usec) - (time_last_inner_loop.tv_sec*1e6 + time_last_inner_loop.tv_usec)) < refresh_time_inner_loop*1e6){
      usleep(5);
      gettimeofday(&current_time, NULL);
    }
    //Print performances if needed
    if(verbose_runtime){
      printf(" Effective refresh time inner loop = %f \n",(float) ((current_time.tv_sec*1e6 + current_time.tv_usec) - (time_last_inner_loop.tv_sec*1e6 + time_last_inner_loop.tv_usec)));
      fflush(stdout);
    }
    //Update time_last_inner_loop:
    gettimeofday(&time_last_inner_loop, NULL);

  }
}

static void sixdof_current_mode_callback(IvyClientPtr app, void *user_data, int argc, char *argv[])
{
  if (argc != 2)
  {
    fprintf(stderr,"ERROR: invalid message length SIXDOF_SYSTEM_CURRENT_MODE \n");
  }
  else{
    double timestamp_d = atof(argv[0]); 
    int8_t current_sixdof_mode_local = (int) atof(argv[1]); 
    if(verbose_sixdof){
      fprintf(stderr,"Received SIXDOF_SYSTEM_CURRENT_MODE - Timestamp = %.5f, current_mode = %d; \n",timestamp_d,current_sixdof_mode_local);
    }
    pthread_mutex_lock(&mutex_sixdof);
    int desired_sixdof_mode_local = desired_sixdof_mode;
    current_sixdof_mode = current_sixdof_mode_local;
    last_ping_time_sixdof = timestamp_d;  
    pthread_mutex_unlock(&mutex_sixdof);
    //1 -->rel beacon pos; 2 -->rel beacon angle; 3-->sixdof mode. Default is 1. 
    if(current_sixdof_mode_local != desired_sixdof_mode_local){
      //Change sixdof mode 
      IvySendMsg("SET_SIXDOF_SYS_MODE %d", desired_sixdof_mode_local);
    }
  }
}

static void sixdof_beacon_pos_callback(IvyClientPtr app, void *user_data, int argc, char *argv[])
{
  if (argc != 5)
  {
    fprintf(stderr,"ERROR: invalid message length RELATIVE_BEACON_POS \n");
  }
  else{
    float beacon_rel_pos[3];
    double timestamp_d = atof(argv[0]); 
    int beacon_id = (int) atof(argv[1]); 
    beacon_rel_pos[0] = (float) atof(argv[2]); 
    beacon_rel_pos[1] = (float) atof(argv[3]); 
    beacon_rel_pos[2] = (float) atof(argv[4]); 
    if(verbose_sixdof){
      fprintf(stderr,"Received beacon position - Timestamp = %.5f, ID = %d; Posx = %.3f Posy = %.3f; Posz = %.3f; \n",timestamp_d,beacon_id,beacon_rel_pos[0],beacon_rel_pos[1],beacon_rel_pos[2]);
    }

    //use mutex: 
    pthread_mutex_lock(&mutex_sixdof);
    int beacon_tracking_id_local = beacon_tracking_id;
    int8_t current_sixdof_mode_local = current_sixdof_mode; 
    pthread_mutex_unlock(&mutex_sixdof);
    
    //If beacon number is the one we want to track then transpose the position in NED frame and send it to the UAV: 
    if(beacon_id == beacon_tracking_id_local){
      //Get current euler angles and UAV position from serial connection through mutex:     
      struct am7_data_in myam7_data_in_copy;
      pthread_mutex_lock(&mutex_am7);
      memcpy(&myam7_data_in_copy, &myam7_data_in, sizeof(struct am7_data_in));
      pthread_mutex_unlock(&mutex_am7); 
      float UAV_NED_pos[3] = {myam7_data_in_copy.UAV_NED_pos_x, myam7_data_in_copy.UAV_NED_pos_y, myam7_data_in_copy.UAV_NED_pos_z}; 
      float UAV_euler_angles_rad[3] = {(float) myam7_data_in_copy.phi_state_int*1e-2*M_PI/180,
                                      (float) myam7_data_in_copy.theta_state_int*1e-2*M_PI/180,
                                      (float) myam7_data_in_copy.psi_state_int*1e-2*M_PI/180};

      //Transpose relative position to target position for the UAV: 
      float beacon_absolute_ned_pos[3];
      from_body_to_earth(&beacon_absolute_ned_pos[0], &beacon_rel_pos[0], UAV_euler_angles_rad[0], UAV_euler_angles_rad[1], UAV_euler_angles_rad[2]);
      //Sun current UAV position to have the real abs marker value: 
      for(int i = 0; i < 3; i++){
        beacon_absolute_ned_pos[i] += UAV_NED_pos[i]; 
      }

      //Copy absolute position to 
      struct marker_detection_t sixdof_detection_copy; 
      sixdof_detection_copy.timestamp_detection = timestamp_d;
      sixdof_detection_copy.NED_pos_x = beacon_absolute_ned_pos[0]; 
      sixdof_detection_copy.NED_pos_y = beacon_absolute_ned_pos[1];  
      sixdof_detection_copy.NED_pos_z  = beacon_absolute_ned_pos[2];  
      sixdof_detection_copy.system_status = current_sixdof_mode_local;
      
      if(verbose_sixdof){
        printf("Sixdof timestamp = %f \n", sixdof_detection_copy.timestamp_detection); 
        printf("Sixdof NED pos_x = %f \n",(float) sixdof_detection_copy.NED_pos_x ); 
        printf("Sixdof NED pos_y = %f \n",(float) sixdof_detection_copy.NED_pos_y ); 
        printf("Sixdof NED pos_z  = %f \n",(float) sixdof_detection_copy.NED_pos_z ); 
        printf("Sixdof BODY pos_x = %f \n",(float) beacon_rel_pos[0] ); 
        printf("Sixdof BODY pos_y = %f \n",(float) beacon_rel_pos[1] ); 
        printf("Sixdof BODY pos_z  = %f \n \n",(float) beacon_rel_pos[2] ); 
      }

      pthread_mutex_lock(&mutex_sixdof);
      memcpy(&sixdof_detection, &sixdof_detection_copy, sizeof(struct marker_detection_t));
      pthread_mutex_unlock(&mutex_sixdof);
    }

  }
}

static void sixdof_beacon_angle_callback(IvyClientPtr app, void *user_data, int argc, char *argv[])
{
  if (argc != 6)
  {
    fprintf(stderr,"ERROR: invalid message length RELATIVE_BEACON_ANGLE\n");
  }
  else{
    double timestamp_d = atof(argv[0]); 
    int beacon_id = (int) atof(argv[1]); 
    float XAngle_deg = atof(argv[2]); 
    float YAngle_deg = atof(argv[3]); 
    float Intensity = atof(argv[4]); 
    float Width = atof(argv[5]); 
    if(verbose_sixdof){
      fprintf(stderr,"Received beacon relative angle - Timestamp = %.5f, ID = %d; XAngle_deg = %.3f YAngle_deg = %.3f; Intensity = %.3f; Width = %.3f; \n",timestamp_d,beacon_id,XAngle_deg,YAngle_deg,Intensity,Width);
    }
  }

  //DO something (TODO)
}

static void sixdof_mode_callback(IvyClientPtr app, void *user_data, int argc, char *argv[])
{
  if (argc != 13)
  {
    fprintf(stderr,"ERROR: invalid message length SIXDOF_TRACKING\n");
  }
  else{
    double timestamp_d = atof(argv[0]); 
    float X_pos = atof(argv[1]); 
    float Y_pos = atof(argv[2]); 
    float Z_pos = atof(argv[3]); 
    float Phi_rad = atof(argv[4]); 
    float Theta_rad = atof(argv[5]); 
    float Psi_rad = atof(argv[6]); 
    float var_x = atof(argv[7]); 
    float var_y = atof(argv[8]); 
    float var_z = atof(argv[9]); 
    float var_phi = atof(argv[10]); 
    float var_theta = atof(argv[11]); 
    float var_psi = atof(argv[12]);  

    if(verbose_sixdof){
      fprintf(stderr,"Received sixdof packet - Timestamp = %.5f, X_pos = %.3f, Y_pos = %.3f, Z_pos = %.3f, Phi = %.3f, Theta = %.3f, Psi = %.3f, var_x = %.3f, var_y = %.3f, var_z = %.3f, var_phi = %.3f, var_theta = %.3f, var_psi = %.3f;\n",timestamp_d,X_pos,Y_pos,Z_pos,Phi_rad*180/M_PI,Theta_rad*180/M_PI,Psi_rad*180/M_PI,var_x,var_y,var_z,var_phi,var_theta,var_psi);
    }

    if(sqrtf(var_x*var_x + var_y*var_y + var_z*var_z) < max_tolerance_variance_sixdof){
      float local_pos_target_body_rf[3] = {X_pos, Y_pos, Z_pos}; 
        //Get current euler angles and UAV position from serial connection through mutex:     
      struct am7_data_in myam7_data_in_copy;
      pthread_mutex_lock(&mutex_am7);
      memcpy(&myam7_data_in_copy, &myam7_data_in, sizeof(struct am7_data_in));
      pthread_mutex_unlock(&mutex_am7); 
      float UAV_NED_pos[3] = {myam7_data_in_copy.UAV_NED_pos_x, myam7_data_in_copy.UAV_NED_pos_y, myam7_data_in_copy.UAV_NED_pos_z}; 
      float UAV_euler_angles_rad[3] = {(float) myam7_data_in_copy.phi_state_int*1e-2*M_PI/180,
                                      (float) myam7_data_in_copy.theta_state_int*1e-2*M_PI/180,
                                      (float) myam7_data_in_copy.psi_state_int*1e-2*M_PI/180};

      //Transpose relative position to target position for the UAV: 
      float beacon_absolute_ned_pos[3];
      // from_body_to_earth(&beacon_absolute_ned_pos[0], &local_pos_target_body_rf[0], UAV_euler_angles_rad[0], UAV_euler_angles_rad[1], UAV_euler_angles_rad[2]);
      // from_body_to_earth(&beacon_absolute_ned_pos[0], &local_pos_target_body_rf[0], 0, 0, 0);
      // beacon_absolute_ned_pos[0] = local_pos_target_body_rf[1]; 
      // beacon_absolute_ned_pos[1] = local_pos_target_body_rf[0];
      // beacon_absolute_ned_pos[2] = local_pos_target_body_rf[2];
      //Sum current UAV position to have the real abs marker value: 
      for(int i = 0; i < 3; i++){
        beacon_absolute_ned_pos[i] += UAV_NED_pos[i]; 
      }

      //Based on the euler angles, determine the relative euler angles of the target:



      //Retrieve system status from sixdof through mutex:
      pthread_mutex_lock(&mutex_sixdof);
      int8_t current_sixdof_mode_local = current_sixdof_mode;
      pthread_mutex_unlock(&mutex_sixdof);
      //Copy absolute position to sixdof struct
      struct marker_detection_t sixdof_detection_copy; 
      sixdof_detection_copy.timestamp_detection = timestamp_d;
      sixdof_detection_copy.NED_pos_x = beacon_absolute_ned_pos[0]; 
      sixdof_detection_copy.NED_pos_y = beacon_absolute_ned_pos[1];  
      sixdof_detection_copy.NED_pos_z  = beacon_absolute_ned_pos[2]; 
      sixdof_detection_copy.relative_phi_rad = Phi_rad;
      sixdof_detection_copy.relative_theta_rad = Theta_rad;
      sixdof_detection_copy.relative_psi_rad = Psi_rad;
      sixdof_detection_copy.system_status = current_sixdof_mode_local;
      
      if(verbose_sixdof){
        printf("Sixdof timestamp = %f \n", sixdof_detection_copy.timestamp_detection); 
        printf("Sixdof NED pos_x = %f \n",(float) sixdof_detection_copy.NED_pos_x ); 
        printf("Sixdof NED pos_y = %f \n",(float) sixdof_detection_copy.NED_pos_y ); 
        printf("Sixdof NED pos_z  = %f \n",(float) sixdof_detection_copy.NED_pos_z ); 
        printf("Sixdof BODY pos_x = %f \n",(float) local_pos_target_body_rf[0] ); 
        printf("Sixdof BODY pos_y = %f \n",(float) local_pos_target_body_rf[1] ); 
        printf("Sixdof BODY pos_z  = %f \n \n",(float) local_pos_target_body_rf[2] ); 
        //Add relative angles, variance and system status: 
        printf("Sixdof Phi = %f \n",(float) Phi_rad*180/M_PI );
        printf("Sixdof Theta = %f \n",(float) Theta_rad*180/M_PI );
        printf("Sixdof Psi = %f \n",(float) Psi_rad*180/M_PI );
        printf("Sixdof var_x = %f \n",(float) var_x ); 
        printf("Sixdof var_y = %f \n",(float) var_y );
        printf("Sixdof var_z = %f \n",(float) var_z );
        printf("Sixdof var_phi = %f \n",(float) var_phi );
        printf("Sixdof var_theta = %f \n",(float) var_theta );
        printf("Sixdof var_psi = %f \n",(float) var_psi );
        printf("Sixdof system status = %d \n", current_sixdof_mode_local);
      }

      pthread_mutex_lock(&mutex_sixdof);
      memcpy(&sixdof_detection, &sixdof_detection_copy, sizeof(struct marker_detection_t));
      pthread_mutex_unlock(&mutex_sixdof); 
    }
  }
}

static void aruco_position_report(IvyClientPtr app, void *user_data, int argc, char *argv[])
{
  if (argc != 4)
  {
    fprintf(stderr,"ERROR: invalid message length DESIRED_SP\n");
  }
  struct marker_detection_t aruco_detection_copy; 

  //Retrive and copy system status: 
  pthread_mutex_lock(&mutex_aruco);
  int aruco_system_status_local = aruco_detection.system_status;
  pthread_mutex_unlock(&mutex_aruco);

  gettimeofday(&aruco_time, NULL); 
  aruco_detection_copy.timestamp_detection = (aruco_time.tv_sec*1e6 - starting_time_program_execution.tv_sec*1e6 + aruco_time.tv_usec - starting_time_program_execution.tv_usec)*1e-6;
  aruco_detection_copy.NED_pos_x = atof(argv[1]); 
  aruco_detection_copy.NED_pos_y = atof(argv[2]);  
  aruco_detection_copy.NED_pos_z  = atof(argv[3]);  
  aruco_detection_copy.system_status = aruco_system_status_local;
  
  if(verbose_aruco){
    printf("\n Aruco timestamp = %f \n", aruco_detection_copy.timestamp_detection); 
    printf("\n Aruco NED pos_x = %f \n",(float) aruco_detection_copy.NED_pos_x ); 
    printf("\n Aruco NED pos_y = %f \n",(float) aruco_detection_copy.NED_pos_y ); 
    printf("\n Aruco NED pos_z  = %f \n",(float) aruco_detection_copy.NED_pos_z ); 
  }

  pthread_mutex_lock(&mutex_aruco);
  memcpy(&aruco_detection, &aruco_detection_copy, sizeof(struct marker_detection_t));
  pthread_mutex_unlock(&mutex_aruco); 

}

static void aruco_position_callback(IvyClientPtr app, void *user_data, int argc, char *argv[])
{
  if (argc != 5)
  {
    fprintf(stderr,"ERROR: invalid message length ARUCO_RELATIVE_POS\n");
  }
  else{
    float beacon_rel_pos[3];
    double timestamp_d = atof(argv[0]);
    int aruco_marker_id = (int) atof(argv[1]);
    beacon_rel_pos[0] = (float) atof(argv[2]);
    beacon_rel_pos[1] = (float) atof(argv[3]);
    beacon_rel_pos[2] = (float) atof(argv[4]);
    //Get current euler angles and UAV position from serial connection through mutex:     
    struct am7_data_in myam7_data_in_copy;
    pthread_mutex_lock(&mutex_am7);
    memcpy(&myam7_data_in_copy, &myam7_data_in, sizeof(struct am7_data_in));
    pthread_mutex_unlock(&mutex_am7); 
    float UAV_NED_pos[3] = {myam7_data_in_copy.UAV_NED_pos_x, myam7_data_in_copy.UAV_NED_pos_y, myam7_data_in_copy.UAV_NED_pos_z};
    float UAV_euler_angles_rad[3] = {(float) myam7_data_in_copy.phi_state_int*1e-2*M_PI/180,
                                    (float) myam7_data_in_copy.theta_state_int*1e-2*M_PI/180,
                                    (float) myam7_data_in_copy.psi_state_int*1e-2*M_PI/180};
    
    //Transpose relative position to target position for the UAV:
    float beacon_absolute_ned_pos[3];
    from_body_to_earth(&beacon_absolute_ned_pos[0], &beacon_rel_pos[0], UAV_euler_angles_rad[0], UAV_euler_angles_rad[1], UAV_euler_angles_rad[2]);
    //Sum current UAV position to have the real abs marker value:
    for(int i = 0; i < 3; i++){
      beacon_absolute_ned_pos[i] += UAV_NED_pos[i];
    }

    //Retrive and copy system status: 
    pthread_mutex_lock(&mutex_aruco);
    int aruco_system_status_local = aruco_detection.system_status;
    pthread_mutex_unlock(&mutex_aruco);

    //Copy absolute position to aruco struct
    struct marker_detection_t aruco_detection_copy;
    aruco_detection_copy.timestamp_detection = timestamp_d;
    aruco_detection_copy.NED_pos_x = beacon_absolute_ned_pos[0];
    aruco_detection_copy.NED_pos_y = beacon_absolute_ned_pos[1];
    aruco_detection_copy.NED_pos_z = beacon_absolute_ned_pos[2];
    aruco_detection_copy.system_status = (int8_t) aruco_system_status_local;

    if(verbose_aruco){
      printf("Aruco timestamp = %f \n", aruco_detection_copy.timestamp_detection);
      printf("Aruco index = %d \n", aruco_marker_id);
      printf("Aruco NED pos_x = %f \n",(float) aruco_detection_copy.NED_pos_x );
      printf("Aruco NED pos_y = %f \n",(float) aruco_detection_copy.NED_pos_y );
      printf("Aruco NED pos_z  = %f \n",(float) aruco_detection_copy.NED_pos_z );
      printf("Aruco BODY pos_x = %f \n",(float) beacon_rel_pos[0] );
      printf("Aruco BODY pos_y = %f \n",(float) beacon_rel_pos[1] );
      printf("Aruco BODY pos_z  = %f \n \n",(float) beacon_rel_pos[2] );
      printf("Aruco system status = %d \n", aruco_system_status_local);
    }

    pthread_mutex_lock(&mutex_aruco);
    memcpy(&aruco_detection, &aruco_detection_copy, sizeof(struct marker_detection_t));
    pthread_mutex_unlock(&mutex_aruco);

  }

}

static void aruco_heartbit_callback(IvyClientPtr app, void *user_data, int argc, char *argv[])
{
  if (argc != 2)
  {
    fprintf(stderr,"ERROR: invalid message length HEARTBEAT_ARUCO\n");
  }
  else{
    double timestamp_d = atof(argv[0]);
    int aruco_system_status_local = (int) atof(argv[1]);
    if(verbose_aruco){
      fprintf(stderr,"Received ARUCO_HEARTBEAT - Timestamp = %.5f, system_status = %d; \n",timestamp_d,aruco_system_status);
    }
    pthread_mutex_lock(&mutex_aruco);
    aruco_system_status = aruco_system_status_local;
    last_ping_time_aruco = timestamp_d;
    pthread_mutex_unlock(&mutex_aruco);
  }
}

void main() {

  //Initialize the module 
  am7_init();

  //Init 
  #ifdef __APPLE__
    char* ivy_bus = "224.255.255.255";
  #else
    char *ivy_bus = "127.255.255.255";
  #endif

  GMainLoop *ml =  g_main_loop_new(NULL, FALSE);

  IvyInit ("NonlinearCA", "NonlinearCA READY", NULL, NULL, NULL, NULL);
  IvyStart(ivy_bus);

  #ifdef USE_ARUCO
    IvyBindMsg(aruco_position_report, NULL, "^ground DESIRED_SP %s (\\S*) (\\S*) (\\S*) (\\S*)", "1");
    IvyBindMsg(aruco_position_callback, NULL, "ARUCO_RELATIVE_POS (\\S*) (\\S*) (\\S*) (\\S*) (\\S*)");
    IvyBindMsg(aruco_heartbit_callback, NULL, "HEARTBEAT_ARUCO (\\S*) (\\S*)");
  #endif

  #ifdef USE_SIXDOF
    IvyBindMsg(sixdof_beacon_pos_callback, NULL, "RELATIVE_BEACON_POS (\\S*) (\\S*) (\\S*) (\\S*) (\\S*)");
    IvyBindMsg(sixdof_beacon_angle_callback, NULL, "RELATIVE_BEACON_ANGLE (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*)");
    IvyBindMsg(sixdof_mode_callback, NULL, "SIXDOF_TRACKING (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*)");
    IvyBindMsg(sixdof_current_mode_callback, NULL, "SIXDOF_SYSTEM_CURRENT_MODE (\\S*) (\\S*)");
  #endif

  pthread_t thread1, thread2, thread3, thread4;

  // make threads
  pthread_create(&thread1, NULL, first_thread, NULL);
  pthread_create(&thread2, NULL, second_thread, NULL);
  pthread_create(&thread3, NULL, third_thread, NULL);
  pthread_create(&thread4, NULL, fourth_thread, NULL);

  g_main_loop_run(ml);

  //Close the serial and clean the variables 
  fflush (stdout);
  close(serial_port);
}
