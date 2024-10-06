#include "am7x.h"
#include <pthread.h>
#include "MATLAB_generated_files/Nonlinear_controller_w_ail_basic_aero_outer_loop.h"
#include "MATLAB_generated_files/Nonlinear_controller_w_ail_basic_aero_inner_loop.h"
#include "MATLAB_generated_files/compute_acc_control_rf_basic.h"
#include "MATLAB_generated_files/rt_nonfinite.h"
#include <string.h>
#include <glib.h>
#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>
#include "filters/low_pass_filter.h"


#define USE_ARUCO
#define USE_SIXDOF

//Variable for current acceleration filtering:
Butterworth2LowPass modeled_accelerations_filter[6];
Butterworth2LowPass body_rates_filter[3], u_in_filter[NUM_ACT_IN_U_IN]; 
Butterworth2LowPass airspeed_filter, flight_path_angle_filter; 

int filter_cutoff_frequency;
float tau_indi;

double u_init_outer[NUM_ACT_IN_U_IN];
double u_init_inner[NUM_ACT_IN_U_IN_INNER];

//To test the controller with random variables:
// #define TEST_CONTROLLER

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

int verbose_sixdof_com = 0;
int verbose_sixdof_position = 1;
int verbose_connection = 0;
int verbose_optimizer = 0;
int verbose_runtime = 0; 
int verbose_data_in = 0; 
int verbose_submitted_data = 0; 
int verbose_ivy_bus = 0; 
int verbose_aruco = 0; 
int verbose_compute_accel = 0; 
int verbose_filters = 0; 

//LIDAR VARIABLES
int16_t lidar_dist_cm = -1; 
int16_t lidar_signal_strength = -1; 

//COMPUTER VISION 
struct marker_detection_t aruco_detection;
pthread_mutex_t mutex_aruco;

//SIXDOF VARIABLES: 
int default_desired_sixdof_mode = 1; //1 -->rel beacon pos; 2 -->rel beacon angle; 3-->sixdof mode. Default is 1. 
int default_beacon_tracking_id = 1640; 
float max_tolerance_variance_sixdof = 1; //meters
int current_sixdof_mode; 
struct marker_detection_t sixdof_detection;
pthread_mutex_t mutex_sixdof;

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
      usleep(20);
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

void* first_thread() //Receive and send messages to pixhawk
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

    float beacon_tracking_id = extra_data_in_copy[87];
    float desired_sixdof_mode = extra_data_in_copy[88];

    float use_u_init_outer_loop = extra_data_in_copy[89];
    float use_u_init_inner_loop = extra_data_in_copy[90];

    float K_t_airspeed = extra_data_in_copy[91];

    //Exceptions: 
    if(beacon_tracking_id < 0.1f){
      beacon_tracking_id = default_beacon_tracking_id;
    }
    if(desired_sixdof_mode < 0.1f){
      desired_sixdof_mode = default_desired_sixdof_mode;
    }
    if(filter_cutoff_frequency_telem < 0.1f){
      filter_cutoff_frequency_telem = (int)filter_cutoff_frequency_init;
    }


    ///////////////////////////////////////////////////////////////////////////////////////////////// Define real time variables:
    float Phi = (myam7_data_in_copy.phi_state_int*1e-2 * M_PI/180);
    float Theta = (myam7_data_in_copy.theta_state_int*1e-2 * M_PI/180);
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

    //Bound motor values to be within min and max value to avoid NaN
    Bound(Omega_1,min_omega,max_omega);
    Bound(Omega_2,min_omega,max_omega);
    Bound(Omega_3,min_omega,max_omega);
    Bound(Omega_4,min_omega,max_omega);

    /////////////////////////////////////////////////Reset filters in case we want a different omega than the one inputted initially
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
    }

    //////////////////////////////////////////////////////////////////////////////////////////Compute modeled accellerations:
    float K_p_T_airspeed_corrected = K_p_T * (1 - V*K_t_airspeed);
    float K_p_M_airspeed_corrected = K_p_M * (1 - V*K_t_airspeed);
    // double gain_motor = (max_omega - min_omega)/2;
    // double Omega_1_scaled = Omega_1 / gain_motor;
    // double Omega_2_scaled = Omega_2 / gain_motor;
    // double V_scaled = V/max_airspeed;
    double modeled_accelerations[6];

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

    //FILTER VALUES:
    //////////////////////////////////////////////////////////////////////////////////////Filtered current modeled acc
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

    //////////////////////////////////////////////////////////////////////////////////////////////Filtered body rates 
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
      body_rates_array_filtered_double[i] = body_rates_filter[i].o[0]; 
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////Filtered u_in
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

    ///////////////////////////////////////////////////////////////////////////////////////////////////Filtered airspeed
    update_butterworth_2_low_pass(&airspeed_filter, (float) V);
    //Check for NaN in filter and reset it: 
    if(airspeed_filter.o[0] != airspeed_filter.o[0]){
      init_butterworth_2_low_pass(&airspeed_filter, tau_indi, refresh_time_filters, 0.0);
      if(verbose_filters){
        printf("WARNING, airspeed filter REINITIALIZED!!!! \n");
      }
    }
    float airspeed_filtered = airspeed_filter.o[0]; 

    //////////////////////////////////////////////////////////////////////////////////////// //Filtered flight path angle
    update_butterworth_2_low_pass(&flight_path_angle_filter, (float) flight_path_angle);
    //Check for NaN in filter and reset it: 
    if(flight_path_angle_filter.o[0] != flight_path_angle_filter.o[0]){
      init_butterworth_2_low_pass(&flight_path_angle_filter, tau_indi, refresh_time_filters, 0.0);
      if(verbose_filters){
        printf("WARNING, flight path angle filter REINITIALIZED!!!! \n");
      }
    }
    float flight_path_angle_filtered = flight_path_angle_filter.o[0];

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
    mydata_in_optimizer_copy.beacon_tracking_id = beacon_tracking_id;
    mydata_in_optimizer_copy.desired_sixdof_mode = desired_sixdof_mode;
    mydata_in_optimizer_copy.use_u_init_outer_loop = use_u_init_outer_loop;
    mydata_in_optimizer_copy.use_u_init_inner_loop = use_u_init_inner_loop;

    //Print received data if needed
    if(verbose_data_in){

      printf("\n ROLLING MESSAGE VARIABLES IN-------------------------------------------------- \n"); 
      printf("\n K_p_T * 1e-5 = %f \n",(float) mydata_in_optimizer_copy.K_p_T * 1e5); 
      printf(" K_p_M * 1e-7 = %f \n",(float) mydata_in_optimizer_copy.K_p_M * 1e7); 
      printf(" m = %f \n",(float) mydata_in_optimizer_copy.m); 
      printf(" I_xx = %f \n",(float) mydata_in_optimizer_copy.I_xx); 
      printf(" I_yy = %f \n",(float) mydata_in_optimizer_copy.I_yy); 
      printf(" I_zz = %f \n",(float) mydata_in_optimizer_copy.I_zz); 
      printf(" l_1 = %f \n",(float) mydata_in_optimizer_copy.l_1); 
      printf(" l_2 = %f \n",(float) mydata_in_optimizer_copy.l_2); 
      printf(" l_3 = %f \n",(float) mydata_in_optimizer_copy.l_3); 
      printf(" l_4 = %f \n",(float) mydata_in_optimizer_copy.l_4); 
      printf(" l_z = %f \n",(float) mydata_in_optimizer_copy.l_z); 
      printf(" max_omega = %f \n",(float) mydata_in_optimizer_copy.max_omega); 
      printf(" min_omega = %f \n",(float) mydata_in_optimizer_copy.min_omega); 
      printf(" max_b = %f \n",(float) mydata_in_optimizer_copy.max_b); 
      printf(" min_b = %f \n",(float) mydata_in_optimizer_copy.min_b); 
      printf(" max_g = %f \n",(float) mydata_in_optimizer_copy.max_g); 
      printf(" min_g = %f \n",(float) mydata_in_optimizer_copy.min_g); 
      printf(" max_theta = %f \n",(float) mydata_in_optimizer_copy.max_theta); 
      printf(" min_theta = %f \n",(float) mydata_in_optimizer_copy.min_theta); 
      printf(" max_alpha = %f \n",(float) mydata_in_optimizer_copy.max_alpha*180/M_PI); 
      printf(" min_alpha = %f \n",(float) mydata_in_optimizer_copy.min_alpha*180/M_PI); 
      printf(" max_phi = %f \n",(float) mydata_in_optimizer_copy.max_phi); 
      printf(" Cm_zero = %f \n",(float) mydata_in_optimizer_copy.Cm_zero);
      printf(" Cm_alpha = %f \n",(float) mydata_in_optimizer_copy.Cm_alpha);
      printf(" Cl_alpha = %f \n",(float) mydata_in_optimizer_copy.Cl_alpha);
      printf(" Cd_zero = %f \n",(float) mydata_in_optimizer_copy.Cd_zero);
      printf(" K_Cd = %f \n",(float) mydata_in_optimizer_copy.K_Cd);
      printf(" S = %f \n",(float) mydata_in_optimizer_copy.S);
      printf(" wing_chord = %f \n",(float) mydata_in_optimizer_copy.wing_chord);
      printf(" rho = %f \n",(float) mydata_in_optimizer_copy.rho);
      printf(" W_act_motor_const = %f \n",(float) mydata_in_optimizer_copy.W_act_motor_const);
      printf(" W_act_motor_speed = %f \n",(float) mydata_in_optimizer_copy.W_act_motor_speed);
      printf(" W_act_tilt_el_const = %f \n",(float) mydata_in_optimizer_copy.W_act_tilt_el_const);
      printf(" W_act_tilt_el_speed = %f \n",(float) mydata_in_optimizer_copy.W_act_tilt_el_speed);
      printf(" W_act_tilt_az_const = %f \n",(float) mydata_in_optimizer_copy.W_act_tilt_az_const);
      printf(" W_act_tilt_az_speed = %f \n",(float) mydata_in_optimizer_copy.W_act_tilt_az_speed);
      printf(" W_act_theta_const = %f \n",(float) mydata_in_optimizer_copy.W_act_theta_const);
      printf(" W_act_theta_speed = %f \n",(float) mydata_in_optimizer_copy.W_act_theta_speed);
      printf(" W_act_phi_const = %f \n",(float) mydata_in_optimizer_copy.W_act_phi_const);
      printf(" W_act_phi_speed = %f \n",(float) mydata_in_optimizer_copy.W_act_phi_speed);
      printf(" W_dv_1 = %f \n",(float) mydata_in_optimizer_copy.W_dv_1);
      printf(" W_dv_2 = %f \n",(float) mydata_in_optimizer_copy.W_dv_2);
      printf(" W_dv_3 = %f \n",(float) mydata_in_optimizer_copy.W_dv_3);
      printf(" W_dv_4 = %f \n",(float) mydata_in_optimizer_copy.W_dv_4);
      printf(" W_dv_5 = %f \n",(float) mydata_in_optimizer_copy.W_dv_5);
      printf(" W_dv_6 = %f \n",(float) mydata_in_optimizer_copy.W_dv_6);
      printf(" gamma_quadratic = %f \n",(float) mydata_in_optimizer_copy.gamma_quadratic_du);
      printf(" Cy_beta = %f \n",(float) mydata_in_optimizer_copy.Cy_beta);
      printf(" Cl_beta = %f \n",(float) mydata_in_optimizer_copy.Cl_beta);
      printf(" wing_span = %f \n",(float) mydata_in_optimizer_copy.wing_span);
      printf(" aoa_protection_speed = %f \n",(float) mydata_in_optimizer_copy.aoa_protection_speed);
      printf(" W_act_ailerons_const = %f \n",(float) mydata_in_optimizer_copy.W_act_ailerons_const);
      printf(" W_act_ailerons_speed = %f \n",(float) mydata_in_optimizer_copy.W_act_ailerons_speed);
      printf(" min_delta_ailerons = %f \n",(float) mydata_in_optimizer_copy.min_delta_ailerons);
      printf(" max_delta_ailerons = %f \n",(float) mydata_in_optimizer_copy.max_delta_ailerons);
      printf(" CL_aileron = %f \n",(float) mydata_in_optimizer_copy.CL_aileron);
      printf(" k_alt_tilt_constraint = %f \n",(float) mydata_in_optimizer_copy.k_alt_tilt_constraint);
      printf(" min_alt_tilt_constraint = %f \n",(float) mydata_in_optimizer_copy.min_alt_tilt_constraint);
      printf(" transition_speed = %f \n",(float) mydata_in_optimizer_copy.transition_speed);
      printf(" desired_motor_value = %f \n",(float) mydata_in_optimizer_copy.desired_motor_value);
      printf(" desired_el_value_deg = %f \n",(float) mydata_in_optimizer_copy.desired_el_value*180/M_PI);
      printf(" desired_az_value_deg = %f \n",(float) mydata_in_optimizer_copy.desired_az_value*180/M_PI);
      printf(" desired_ailerons_value = %f \n",(float) mydata_in_optimizer_copy.desired_ailerons_value);
      printf(" min_theta_hard = %f \n",(float) mydata_in_optimizer_copy.min_theta_hard*180/M_PI);
      printf(" max_theta_hard = %f \n",(float) mydata_in_optimizer_copy.max_theta_hard*180/M_PI);
      printf(" min_phi_hard = %f \n",(float) mydata_in_optimizer_copy.min_phi_hard*180/M_PI);
      printf(" max_phi_hard = %f \n",(float) mydata_in_optimizer_copy.max_phi_hard*180/M_PI);
      printf(" disable_acc_decrement_inner_loop = %f \n",(float) mydata_in_optimizer_copy.disable_acc_decrement_inner_loop);
      printf(" filter_cutoff_frequency_telem = %f \n",(float) mydata_in_optimizer_copy.filter_cutoff_frequency_telem);
      printf(" max_airspeed = %f \n",(float) mydata_in_optimizer_copy.max_airspeed);
      printf(" vert_acc_margin = %f \n",(float) mydata_in_optimizer_copy.vert_acc_margin);
      printf(" power_Cd_0 = %f \n",(float) mydata_in_optimizer_copy.power_Cd_0);
      printf(" power_Cd_a = %f \n",(float) mydata_in_optimizer_copy.power_Cd_a);
      printf(" prop_R = %f \n",(float) mydata_in_optimizer_copy.prop_R);
      printf(" prop_Cd_0 = %f \n",(float) mydata_in_optimizer_copy.prop_Cd_0);
      printf(" prop_Cl_0 = %f \n",(float) mydata_in_optimizer_copy.prop_Cl_0);
      printf(" prop_Cd_a = %f \n",(float) mydata_in_optimizer_copy.prop_Cd_a);
      printf(" prop_Cl_a = %f \n",(float) mydata_in_optimizer_copy.prop_Cl_a);
      printf(" prop_delta = %f \n",(float) mydata_in_optimizer_copy.prop_delta);
      printf(" prop_sigma = %f \n",(float) mydata_in_optimizer_copy.prop_sigma);
      printf(" prop_theta = %f \n",(float) mydata_in_optimizer_copy.prop_theta);
      printf(" beacon_tracking_id = %f \n",(float) mydata_in_optimizer_copy.beacon_tracking_id);
      printf(" desired_sixdof_mode = %f \n",(float) mydata_in_optimizer_copy.desired_sixdof_mode); 
      printf(" use_u_init_outer_loop = %f \n",(float) mydata_in_optimizer_copy.use_u_init_outer_loop); 
      printf(" use_u_init_inner_loop = %f \n",(float) mydata_in_optimizer_copy.use_u_init_inner_loop); 
      

      printf("\n REAL TIME VARIABLES IN------------------------------------------------------ \n"); 

      printf(" Phi_deg = %f \n",(float) mydata_in_optimizer_copy.Phi*180/M_PI);
      printf(" Theta_deg = %f \n",(float) mydata_in_optimizer_copy.Theta*180/M_PI);
      printf(" delta_ailerons_deg = %f \n",(float) mydata_in_optimizer_copy.delta_ailerons*180/M_PI);
      printf(" Omega_1_rad_s = %f \n",(float) mydata_in_optimizer_copy.Omega_1);
      printf(" Omega_2_rad_s = %f \n",(float) mydata_in_optimizer_copy.Omega_2);
      printf(" Omega_3_rad_s = %f \n",(float) mydata_in_optimizer_copy.Omega_3);
      printf(" Omega_4_rad_s = %f \n",(float) mydata_in_optimizer_copy.Omega_4);
      printf(" b_1_deg = %f \n",(float) mydata_in_optimizer_copy.b_1*180/M_PI);
      printf(" b_2_deg = %f \n",(float) mydata_in_optimizer_copy.b_2*180/M_PI);
      printf(" b_3_deg = %f \n",(float) mydata_in_optimizer_copy.b_3*180/M_PI);
      printf(" b_4_deg = %f \n",(float) mydata_in_optimizer_copy.b_4*180/M_PI);
      printf(" g_1_deg = %f \n",(float) mydata_in_optimizer_copy.g_1*180/M_PI);
      printf(" g_2_deg = %f \n",(float) mydata_in_optimizer_copy.g_2*180/M_PI);
      printf(" g_3_deg = %f \n",(float) mydata_in_optimizer_copy.g_3*180/M_PI);
      printf(" g_4_deg = %f \n",(float) mydata_in_optimizer_copy.g_4*180/M_PI);
      printf(" p_deg_s = %f \n",(float) mydata_in_optimizer_copy.p*180/M_PI);
      printf(" q_deg_s = %f \n",(float) mydata_in_optimizer_copy.q*180/M_PI);
      printf(" r_deg_s = %f \n",(float) mydata_in_optimizer_copy.r*180/M_PI);
      printf(" V_m_s = %f \n",(float) mydata_in_optimizer_copy.V);
      printf(" flight_path_angle_deg = %f \n",(float) mydata_in_optimizer_copy.flight_path_angle*180/M_PI);
      printf(" Beta_deg = %f \n",(float) mydata_in_optimizer_copy.Beta*180/M_PI);
      printf(" desired_theta_value_deg = %f \n",(float) mydata_in_optimizer_copy.desired_theta_value*180/M_PI);
      printf(" desired_phi_value_deg = %f \n",(float) mydata_in_optimizer_copy.desired_phi_value*180/M_PI);

      printf(" approach_mode = %f \n",(float) mydata_in_optimizer_copy.approach_mode);
      printf(" lidar_alt_corrected = %f \n",(float) mydata_in_optimizer_copy.lidar_alt_corrected);

      printf(" pseudo_control_ax = %f \n",(float) mydata_in_optimizer_copy.pseudo_control_ax);
      printf(" pseudo_control_ay = %f \n",(float) mydata_in_optimizer_copy.pseudo_control_ay);
      printf(" pseudo_control_az = %f \n",(float) mydata_in_optimizer_copy.pseudo_control_az);
      printf(" pseudo_control_p_dot = %f \n",(float) mydata_in_optimizer_copy.pseudo_control_p_dot);
      printf(" pseudo_control_q_dot = %f \n",(float) mydata_in_optimizer_copy.pseudo_control_q_dot);
      printf(" pseudo_control_r_dot = %f \n",(float) mydata_in_optimizer_copy.pseudo_control_r_dot);
      printf(" desired_theta_value = %f \n",(float) mydata_in_optimizer_copy.desired_theta_value);
      printf(" desired_phi_value = %f \n",(float) mydata_in_optimizer_copy.desired_phi_value);

      printf("\n MODELED FILTERED ACCELLERATIONS------------------------------------------------------ \n"); 

      printf(" Modeled ax filtered = %f \n",(float) mydata_in_optimizer_copy.modeled_ax_filtered);
      printf(" Modeled ay filtered = %f \n",(float) mydata_in_optimizer_copy.modeled_ay_filtered);
      printf(" Modeled az filtered = %f \n",(float) mydata_in_optimizer_copy.modeled_az_filtered);
      printf(" Modeled p_dot filtered = %f \n",(float) mydata_in_optimizer_copy.modeled_p_dot_filtered);
      printf(" Modeled q_dot filtered = %f \n",(float) mydata_in_optimizer_copy.modeled_q_dot_filtered);
      printf(" Modeled r_dot filtered = %f \n",(float) mydata_in_optimizer_copy.modeled_r_dot_filtered);

      fflush(stdout);
    }

    #ifdef TEST_CONTROLLER
    #warning "You are using the testing variable, watch out!"
      float pi = M_PI;
      mydata_in_optimizer_copy.K_p_T = 1.106465e-5;
      mydata_in_optimizer_copy.K_p_M = 1.835091e-7;
      mydata_in_optimizer_copy.m = 2.45; 
      mydata_in_optimizer_copy.I_xx = 0.156548;
      mydata_in_optimizer_copy.I_yy = 0.161380; 
      mydata_in_optimizer_copy.I_zz = 0.258662;
      mydata_in_optimizer_copy.l_1 = 0.228;
      mydata_in_optimizer_copy.l_2 = 0.228;
      mydata_in_optimizer_copy.l_3 = 0.37;
      mydata_in_optimizer_copy.l_4 = 0.37;
      mydata_in_optimizer_copy.l_z = 0;

      mydata_in_optimizer_copy.aoa_protection_speed = 3;
      mydata_in_optimizer_copy.transition_speed = 7;

      mydata_in_optimizer_copy.Beta = 0 * pi/180;
      mydata_in_optimizer_copy.flight_path_angle = 0 * pi/180;
      mydata_in_optimizer_copy.V = 0;
      mydata_in_optimizer_copy.Phi = 0 * pi/180;
      mydata_in_optimizer_copy.Theta = 0 * pi/180;
      mydata_in_optimizer_copy.Omega_1 = 800;
      mydata_in_optimizer_copy.Omega_2 = 600;
      mydata_in_optimizer_copy.Omega_3 = 600;
      mydata_in_optimizer_copy.Omega_4 = 600;
      mydata_in_optimizer_copy.b_1 = 0 * pi/180;
      mydata_in_optimizer_copy.b_2 = 0 * pi/180;
      mydata_in_optimizer_copy.b_3 = 0 * pi/180;
      mydata_in_optimizer_copy.b_4 = 0 * pi/180;
      mydata_in_optimizer_copy.g_1 = 0 * pi/180;
      mydata_in_optimizer_copy.g_2 = 0 * pi/180;
      mydata_in_optimizer_copy.g_3 = 0 * pi/180;
      mydata_in_optimizer_copy.g_4 = 0 * pi/180;
      mydata_in_optimizer_copy.delta_ailerons = 0 * pi/180;

      mydata_in_optimizer_copy.W_act_motor_const = 10;
      mydata_in_optimizer_copy.W_act_motor_speed = 0; 
      mydata_in_optimizer_copy.W_act_tilt_el_const = 0;
      mydata_in_optimizer_copy.W_act_tilt_el_speed = 0;
      mydata_in_optimizer_copy.W_act_tilt_az_const = 0; 
      mydata_in_optimizer_copy.W_act_tilt_az_speed = 10;
      mydata_in_optimizer_copy.W_act_theta_const = 100;
      mydata_in_optimizer_copy.W_act_theta_speed = -15;
      mydata_in_optimizer_copy.W_act_phi_const = 100;
      mydata_in_optimizer_copy.W_act_phi_speed = -15;
      mydata_in_optimizer_copy.W_act_ailerons_const = .5;
      mydata_in_optimizer_copy.W_act_ailerons_speed = 0; 

      mydata_in_optimizer_copy.W_dv_1 = 0.01;
      mydata_in_optimizer_copy.W_dv_2 = 0.01;
      mydata_in_optimizer_copy.W_dv_3 = 0.05; 
      mydata_in_optimizer_copy.W_dv_4 = 0.1; 
      mydata_in_optimizer_copy.W_dv_5 = 0.1; 
      mydata_in_optimizer_copy.W_dv_6 = 0.1;
      mydata_in_optimizer_copy.gamma_quadratic_du = .1e-6; 

      mydata_in_optimizer_copy.max_omega = 1000; 
      mydata_in_optimizer_copy.min_omega = 100;
      mydata_in_optimizer_copy.max_b = 25; 
      mydata_in_optimizer_copy.min_b = -130; 
      mydata_in_optimizer_copy.max_g = 60; 
      mydata_in_optimizer_copy.min_g = -60;  
      mydata_in_optimizer_copy.max_theta = 60;
      mydata_in_optimizer_copy.min_theta = -15;
      mydata_in_optimizer_copy.max_phi = 80; 
      mydata_in_optimizer_copy.max_delta_ailerons = 25;
      mydata_in_optimizer_copy.min_delta_ailerons = -25; 

      mydata_in_optimizer_copy.pseudo_control_ax = 0;
      mydata_in_optimizer_copy.pseudo_control_ay = 0;
      mydata_in_optimizer_copy.pseudo_control_az = 0;
      mydata_in_optimizer_copy.pseudo_control_p_dot = 0;
      mydata_in_optimizer_copy.pseudo_control_q_dot = 0;
      mydata_in_optimizer_copy.pseudo_control_r_dot = 0;

      mydata_in_optimizer_copy.p = 0 * pi/180; 
      mydata_in_optimizer_copy.q = 0 * pi/180; 
      mydata_in_optimizer_copy.r = 0 * pi/180; 
      mydata_in_optimizer_copy.Cm_zero = 0.05; 
      mydata_in_optimizer_copy.Cl_alpha = 3.5; 
      mydata_in_optimizer_copy.Cd_zero = 0.2; 
      mydata_in_optimizer_copy.K_Cd = 0.08;
      mydata_in_optimizer_copy.Cm_alpha = -0.1; 
      mydata_in_optimizer_copy.CL_aileron = 0.1; 
      mydata_in_optimizer_copy.rho = 1.225; 
      mydata_in_optimizer_copy.S = 0.43;
      mydata_in_optimizer_copy.wing_chord = 0.3; 
      mydata_in_optimizer_copy.wing_span = 1.4;
      mydata_in_optimizer_copy.max_alpha = 15 * pi/180; 
      mydata_in_optimizer_copy.min_alpha = 2 * pi/180; 

      mydata_in_optimizer_copy.desired_motor_value = 0; 
      mydata_in_optimizer_copy.desired_el_value = 0 * pi/180;
      mydata_in_optimizer_copy.desired_az_value = 0 * pi/180;
      mydata_in_optimizer_copy.desired_theta_value = 0 * pi/180; 
      mydata_in_optimizer_copy.desired_phi_value = 0 * pi/180; 
      mydata_in_optimizer_copy.desired_ailerons_value = 0 * pi/180;

      mydata_in_optimizer_copy.k_alt_tilt_constraint = 55;
      mydata_in_optimizer_copy.min_alt_tilt_constraint = 0.2;
      mydata_in_optimizer_copy.lidar_alt_corrected = 1;
      mydata_in_optimizer_copy.approach_mode = 0; 

      mydata_in_optimizer_copy.max_theta_hard = 45 * pi/180; 
      mydata_in_optimizer_copy.min_theta_hard = -45 * pi/180; 
      mydata_in_optimizer_copy.max_phi_hard = 45 * pi/180; 
      mydata_in_optimizer_copy.min_phi_hard = -45 * pi/180; 

      // Aero model: 
      mydata_in_optimizer_copy.prop_Cl_0 = 0.0;
      mydata_in_optimizer_copy.prop_Cl_a = 3.46;
      mydata_in_optimizer_copy.prop_Cd_0 = 0.05;
      mydata_in_optimizer_copy.prop_Cd_a = 0.36;
      mydata_in_optimizer_copy.prop_sigma = 0.0652;
      mydata_in_optimizer_copy.prop_delta = 0.2;
      mydata_in_optimizer_copy.prop_theta = 0.2188;
      mydata_in_optimizer_copy.prop_R = 0.1270;
      mydata_in_optimizer_copy.power_Cd_0 = mydata_in_optimizer_copy.prop_Cd_0;
      mydata_in_optimizer_copy.power_Cd_a = mydata_in_optimizer_copy.prop_Cd_a;

      mydata_in_optimizer_copy.max_airspeed = 15; 
      mydata_in_optimizer_copy.vert_acc_margin = 2.5; 

      mydata_in_optimizer_copy.use_u_init_outer_loop = 1;
      mydata_in_optimizer_copy.use_u_init_inner_loop = 1;

    #endif 

    pthread_mutex_lock(&mutex_optimizer_input);
    memcpy(&mydata_in_optimizer, &mydata_in_optimizer_copy, sizeof(struct data_in_optimizer));
    pthread_mutex_unlock(&mutex_optimizer_input);

    //Wait until time is not at least refresh_time_filters
    while(((current_time.tv_sec*1e6 + current_time.tv_usec) - (time_last_filt.tv_sec*1e6 + time_last_filt.tv_usec)) < refresh_time_filters*1e6){
      gettimeofday(&current_time, NULL);
      usleep(10);
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

    double verbose = verbose_optimizer;

    //Check if u_init_outer was not initialized yet:
    if(u_init_outer[0] < 1){
      u_init_outer = {mydata_in_optimizer_copy.motor_1_state_filtered, mydata_in_optimizer_copy.motor_2_state_filtered, mydata_in_optimizer_copy.motor_3_state_filtered, mydata_in_optimizer_copy.motor_4_state_filtered,
                      mydata_in_optimizer_copy.el_1_state_filtered, mydata_in_optimizer_copy.el_2_state_filtered, mydata_in_optimizer_copy.el_3_state_filtered, mydata_in_optimizer_copy.el_4_state_filtered,
                      mydata_in_optimizer_copy.az_1_state_filtered, mydata_in_optimizer_copy.az_2_state_filtered, mydata_in_optimizer_copy.az_3_state_filtered, mydata_in_optimizer_copy.az_4_state_filtered, 
                      mydata_in_optimizer_copy.theta_state_filtered, mydata_in_optimizer_copy.phi_state_filtered, mydata_in_optimizer_copy.ailerons_state_filtered};
    }



    //RUN MAIN CODE HERE, USING mydata_in_optimizer_copy as input



    //Set as u_init_outer the u_out for the next iteration: 
    for (int i=0; i< NUM_ACT_IN_U_IN_INNER; i++){
      u_init_outer[i] = u_out[i];
    }

    //Fill the data strtucture to be sent to the inner loop
    struct outer_loop_output myouter_loop_output_copy;
    myouter_loop_output_copy.p_dot_cmd_rad_s = pqr_dot_array_target[0];
    myouter_loop_output_copy.q_dot_cmd_rad_s = pqr_dot_array_target[1];
    myouter_loop_output_copy.r_dot_cmd_rad_s = pqr_dot_array_target[2];
    myouter_loop_output_copy.theta_cmd_rad = theta_cmd;
    myouter_loop_output_copy.phi_cmd_rad = phi_cmd;

    //Send the data to the inner loop
    pthread_mutex_lock(&mutex_outer_loop_output);
    memcpy(&myouter_loop_output, &myouter_loop_output_copy, sizeof(struct outer_loop_output));
    pthread_mutex_unlock(&mutex_outer_loop_output);

    //Wait until time is not at least refresh_time_outer_loop
    while(((current_time.tv_sec*1e6 + current_time.tv_usec) - (time_last_outer_loop.tv_sec*1e6 + time_last_outer_loop.tv_usec)) < refresh_time_outer_loop*1e6){
      gettimeofday(&current_time, NULL);
      usleep(10);
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

    double verbose = verbose_optimizer;

    //Check if u_init_inner was not initialized yet:
    if(u_init_inner[0] < 1){
      u_init_inner = {mydata_in_optimizer_copy.motor_1_state_filtered, mydata_in_optimizer_copy.motor_2_state_filtered, mydata_in_optimizer_copy.motor_3_state_filtered, mydata_in_optimizer_copy.motor_4_state_filtered,
                      mydata_in_optimizer_copy.el_1_state_filtered, mydata_in_optimizer_copy.el_2_state_filtered, mydata_in_optimizer_copy.el_3_state_filtered, mydata_in_optimizer_copy.el_4_state_filtered,
                      mydata_in_optimizer_copy.az_1_state_filtered, mydata_in_optimizer_copy.az_2_state_filtered, mydata_in_optimizer_copy.az_3_state_filtered, mydata_in_optimizer_copy.az_4_state_filtered, 
                      mydata_in_optimizer_copy.ailerons_state_filtered};
    }

    //Run main code here using mydata_in_optimizer_copy as input and supplying as 
    // output u_out, exitflag, N_iterations, N_evaluations, exitflag and residuals.


 
    
    //Wait until time is not at least refresh_time_optimizer
    gettimeofday(&current_time, NULL);
    while(((current_time.tv_sec*1e6 + current_time.tv_usec) - (time_last_inner_loop.tv_sec*1e6 + time_last_inner_loop.tv_usec)) < refresh_time_inner_loop*1e6){
      gettimeofday(&current_time, NULL);
      usleep(10);
    }
    //Print performances if needed
    if(verbose_runtime){
      printf(" Effective refresh time inner loop = %f \n",(float) ((current_time.tv_sec*1e6 + current_time.tv_usec) - (time_last_inner_loop.tv_sec*1e6 + time_last_inner_loop.tv_usec)));
      fflush(stdout);
    }
    //Update time_last_inner_loop:
    gettimeofday(&time_last_inner_loop, NULL);

    //Set as u_init_inner the u_out for the next iteration: 
    for (int i=0; i< NUM_ACT_IN_U_IN_INNER; i++){
      u_init_inner[i] = u_out[i];
    }
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
    myam7_data_out_copy.exit_flag_optimizer = (int16_T) (exitflag);
    myam7_data_out_copy.elapsed_time_us = (uint16_T) (elapsed_time);
    myam7_data_out_copy.n_iteration = (uint16_T) (N_iterations);
    myam7_data_out_copy.n_evaluation = (uint16_T) (N_evaluations);

    //Add to the structure the evaluated modelled values:
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
      printf(" el_1_cmd_deg = %f \n",(float) myam7_data_out_copy.el_1_cmd_int*1e-2*180/M_PI);
      printf(" el_2_cmd_deg = %f \n",(float) myam7_data_out_copy.el_2_cmd_int*1e-2*180/M_PI);
      printf(" el_3_cmd_deg = %f \n",(float) myam7_data_out_copy.el_3_cmd_int*1e-2*180/M_PI);
      printf(" el_4_cmd_deg = %f \n",(float) myam7_data_out_copy.el_4_cmd_int*1e-2*180/M_PI);
      printf(" az_1_cmd_deg = %f \n",(float) myam7_data_out_copy.az_1_cmd_int*1e-2*180/M_PI);
      printf(" az_2_cmd_deg = %f \n",(float) myam7_data_out_copy.az_2_cmd_int*1e-2*180/M_PI);
      printf(" az_3_cmd_deg = %f \n",(float) myam7_data_out_copy.az_3_cmd_int*1e-2*180/M_PI);
      printf(" az_4_cmd_deg = %f \n",(float) myam7_data_out_copy.az_4_cmd_int*1e-2*180/M_PI);
      printf(" ailerons_cmd_deg = %f \n",(float) myam7_data_out_copy.ailerons_cmd_int*1e-2*180/M_PI);
      printf("modeled_ax = %f \n",(float) mydata_in_optimizer_copy.modeled_ax_filtered*1e-2);
      printf("modeled_ay = %f \n",(float) mydata_in_optimizer_copy.modeled_ay_filtered*1e-2);
      printf("modeled_az = %f \n",(float) mydata_in_optimizer_copy.modeled_az_filtered*1e-2);
      printf("modeled_p_dot = %f \n",(float) mydata_in_optimizer_copy.modeled_p_dot_filtered*1e-1*180/M_PI);
      printf("modeled_q_dot = %f \n",(float) mydata_in_optimizer_copy.modeled_q_dot_filtered*1e-1*180/M_PI);
      printf("modeled_r_dot = %f \n",(float) mydata_in_optimizer_copy.modeled_r_dot_filtered*1e-1*180/M_PI);
      printf(" N_iterations = %d \n",(int) N_iterations);
      printf(" N_evaluations = %d \n",(int) N_evaluations);
      printf(" exit_flag_optimizer = %d \n",(int) exitflag);
      printf(" elapsed_time_uS = %d \n",(int) (elapsed_time));
      printf(" \n\n\n");

      fflush(stdout);
    }


    //Wait until time is not at least refresh_time_inner_loop
    while(((current_time.tv_sec*1e6 + current_time.tv_usec) - (time_last_opt_run.tv_sec*1e6 + time_last_opt_run.tv_usec)) < refresh_time_inner_loop*1e6){
      gettimeofday(&current_time, NULL);
      usleep(10);
    }

    //Print performances if needed
    if(verbose_runtime){
      printf(" Effective refresh time inner loop = %f \n",(float) ((current_time.tv_sec*1e6 + current_time.tv_usec) - (time_last_opt_run.tv_sec*1e6 + time_last_opt_run.tv_usec)));
      fflush(stdout);
    }

    //Reset waiting timer: 
    gettimeofday(&time_last_opt_run, NULL);

    //Assign the rolling messages to the appropriate value [dummy]
    float extra_data_out_copy[255];
    extra_data_out_copy[0] = -1.0; 
    extra_data_out_copy[1] = -1.0; 
    
    //Retrieve detection from aruco and assign to the output structure
    struct marker_detection_t aruco_detection_copy; 
  
    pthread_mutex_lock(&mutex_aruco);
    memcpy(&aruco_detection_copy, &aruco_detection, sizeof(struct marker_detection_t));
    pthread_mutex_unlock(&mutex_aruco); 

    myam7_data_out_copy.aruco_detection_timestamp = aruco_detection_copy.timestamp_detection;
    myam7_data_out_copy.aruco_NED_pos_x = aruco_detection_copy.NED_pos_x;
    myam7_data_out_copy.aruco_NED_pos_y = aruco_detection_copy.NED_pos_y;
    myam7_data_out_copy.aruco_NED_pos_z = aruco_detection_copy.NED_pos_z;
    myam7_data_out_copy.aruco_system_status = aruco_detection_copy.system_status;
    
    //Retrieve detection from sixdof and assign to the output structure
    struct marker_detection_t sixdof_detection_copy;
    pthread_mutex_lock(&mutex_sixdof);
    memcpy(&sixdof_detection_copy, &sixdof_detection, sizeof(struct marker_detection_t));
    pthread_mutex_unlock(&mutex_sixdof);

    myam7_data_out_copy.sixdof_detection_timestamp = sixdof_detection_copy.timestamp_detection;
    myam7_data_out_copy.sixdof_NED_pos_x = sixdof_detection_copy.NED_pos_x;
    myam7_data_out_copy.sixdof_NED_pos_y = sixdof_detection_copy.NED_pos_y;
    myam7_data_out_copy.sixdof_NED_pos_z = sixdof_detection_copy.NED_pos_z;
    myam7_data_out_copy.sixdof_system_status = sixdof_detection_copy.system_status;

    //Copy out structure
    pthread_mutex_lock(&mutex_am7);
    memcpy(&myam7_data_out, &myam7_data_out_copy, sizeof(struct am7_data_out));
    memcpy(&extra_data_out, &extra_data_out_copy, sizeof(extra_data_out));
    pthread_mutex_unlock(&mutex_am7);   

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
    current_sixdof_mode = (int) atof(argv[1]); 
    if(verbose_sixdof_com){
      fprintf(stderr,"Received SIXDOF_SYSTEM_CURRENT_MODE - Timestamp = %.5f, current_mode = %d; \n",timestamp_d,current_sixdof_mode);
    }
    //1 -->rel beacon pos; 2 -->rel beacon angle; 3-->sixdof mode. Default is 1. 
    if(current_sixdof_mode != desired_sixdof_mode){
      //Change sixdof mode 
      IvySendMsg("SET_SIXDOF_SYS_MODE %d", desired_sixdof_mode);
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
    if(verbose_sixdof_com){
      fprintf(stderr,"Received beacon position - Timestamp = %.5f, ID = %d; Posx = %.3f Posy = %.3f; Posz = %.3f; \n",timestamp_d,beacon_id,beacon_rel_pos[0],beacon_rel_pos[1],beacon_rel_pos[2]);
    }

    //If beacon number is the one we want to track then transpose the position in NED frame and send it to the UAV: 
    if(beacon_id == beacon_tracking_id){
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

      gettimeofday(&sixdof_time, NULL); 
      sixdof_detection_copy.timestamp_detection = (sixdof_time.tv_sec*1e6 - starting_time_program_execution.tv_sec*1e6 + sixdof_time.tv_usec - starting_time_program_execution.tv_usec)*1e-6;
      sixdof_detection_copy.NED_pos_x = beacon_absolute_ned_pos[0]; 
      sixdof_detection_copy.NED_pos_y = beacon_absolute_ned_pos[1];  
      sixdof_detection_copy.NED_pos_z  = beacon_absolute_ned_pos[2];  
      sixdof_detection_copy.system_status = (int8_t) current_sixdof_mode;
      
      if(verbose_sixdof_position){
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
    if(verbose_sixdof_com){
      fprintf(stderr,"Received beacon relative angle - Timestamp = %.5f, ID = %d; XAngle_deg = %.3f YAngle_deg = %.3f; Intensity = %.3f; Width = %.3f; \n",timestamp_d,beacon_id,XAngle_deg,YAngle_deg,Intensity,Width);
    }
  }
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

    if(verbose_sixdof_com){
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
      from_body_to_earth(&beacon_absolute_ned_pos[0], &local_pos_target_body_rf[0], UAV_euler_angles_rad[0], UAV_euler_angles_rad[1], UAV_euler_angles_rad[2]);
      //Sun current UAV position to have the real abs marker value: 
      for(int i = 0; i < 3; i++){
        beacon_absolute_ned_pos[i] += UAV_NED_pos[i]; 
      }

      //Copy absolute position to sixdof struct
      struct marker_detection_t sixdof_detection_copy; 
      gettimeofday(&sixdof_time, NULL); 
      sixdof_detection_copy.timestamp_detection = (sixdof_time.tv_sec*1e6 - starting_time_program_execution.tv_sec*1e6 + sixdof_time.tv_usec - starting_time_program_execution.tv_usec)*1e-6;
      sixdof_detection_copy.NED_pos_x = beacon_absolute_ned_pos[0]; 
      sixdof_detection_copy.NED_pos_y = beacon_absolute_ned_pos[1];  
      sixdof_detection_copy.NED_pos_z  = beacon_absolute_ned_pos[2];  
      sixdof_detection_copy.system_status = (int8_t) current_sixdof_mode;
      
      if(verbose_sixdof_position){
        printf("Sixdof timestamp = %f \n", sixdof_detection_copy.timestamp_detection); 
        printf("Sixdof NED pos_x = %f \n",(float) sixdof_detection_copy.NED_pos_x ); 
        printf("Sixdof NED pos_y = %f \n",(float) sixdof_detection_copy.NED_pos_y ); 
        printf("Sixdof NED pos_z  = %f \n",(float) sixdof_detection_copy.NED_pos_z ); 
        printf("Sixdof BODY pos_x = %f \n",(float) local_pos_target_body_rf[0] ); 
        printf("Sixdof BODY pos_y = %f \n",(float) local_pos_target_body_rf[1] ); 
        printf("Sixdof BODY pos_z  = %f \n \n",(float) local_pos_target_body_rf[2] ); 
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

  gettimeofday(&aruco_time, NULL); 
  aruco_detection_copy.timestamp_detection = (aruco_time.tv_sec*1e6 - starting_time_program_execution.tv_sec*1e6 + aruco_time.tv_usec - starting_time_program_execution.tv_usec)*1e-6;
  aruco_detection_copy.NED_pos_x = atof(argv[1]); 
  aruco_detection_copy.NED_pos_y = atof(argv[2]);  
  aruco_detection_copy.NED_pos_z  = atof(argv[3]);  
  
  if(verbose_aruco){
    printf("\n Aruco timestamp = %f \n", aruco_detection_copy.timestamp_detection); 
    printf("\n Aruco NED pos_x = %f \n",(float) aruco_detection_copy.NED_pos_x ); 
    printf("\n Aruco NED pos_y = %f \n",(float) aruco_detection_copy.NED_pos_y ); 
    printf("\n Aruco NED pos_z  = %f \n",(float) aruco_detection_copy.NED_pos_z ); 
  }

  pthread_mutex_lock(&mutex_aruco);
  memcpy(&aruco_detection, &aruco_detection_copy, sizeof(struct aruco_detection_t));
  pthread_mutex_unlock(&mutex_aruco); 

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
