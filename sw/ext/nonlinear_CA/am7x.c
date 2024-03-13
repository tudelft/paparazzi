#include "am7x.h"
#include <pthread.h>
#include "MATLAB_generated_files/Nonlinear_controller_w_ailerons_rebuttal.h"
#include "MATLAB_generated_files/rt_nonfinite.h"
#include <string.h>
#include "softServo.h"
#include "filters/low_pass_filter.h"
#include "MATLAB_generated_files/compute_acc_nonlinear_control_rf_w_ailerons.h"

// #define TEST_CONTROLLER

struct am7_data_out myam7_data_out;
struct am7_data_out myam7_data_out_copy;
struct am7_data_out myam7_data_out_copy_internal;
struct am7_data_in myam7_data_in;
struct am7_data_in myam7_data_in_copy;
float extra_data_in[255], extra_data_in_copy[255];
float extra_data_out[255], extra_data_out_copy[255];
uint16_T buffer_in_counter;
char am7_msg_buf_in[sizeof(struct am7_data_in)*2]  __attribute__((aligned));
char am7_msg_buf_out[sizeof(struct am7_data_out)]  __attribute__((aligned));
uint32_T received_packets = 0, received_packets_tot = 0;
uint32_T sent_packets = 0, sent_packets_tot = 0;
uint32_T missed_packets = 0;
uint16_T sent_msg_id = 0, received_msg_id = 0;
int serial_port;
float ca7_message_frequency_RX, ca7_message_frequency_TX;
struct timeval current_time, last_time, last_sent_msg_time, time_last_opt_run, time_last_filt;

pthread_mutex_t mutex_am7;

int verbose_connection = 0;
int verbose_optimizer = 0;
int verbose_servo = 0; 
int verbose_runtime = 0; 

int verbose_filters = 0; 

float angle_1_pwm = 1000; 
float angle_2_pwm = 1000; 

//Variable for current acceleration filtering:
Butterworth2LowPass current_accelerations_filtered[6]; //Filter of current accellerations


void am7_init(){

  //Init serial port
  if ((serial_port = serialOpen ("/dev/ttyS0", BAUDRATE_AM7)) < 0){
    fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno)) ;
  }
  if (wiringPiSetup () == -1){
    fprintf (stdout, "Unable to start wiringPi: %s\n", strerror (errno)) ;
  }
  sent_msg_id = 0;

  //Initialize the servo writing feature
  softServoSetup (0, 1, 2, 3, 4, 5, 6, 7);

  //Initialize the extra messages value 
  for(int i = 0; i < (sizeof(extra_data_in)/sizeof(float)); i++ ){
    extra_data_in[i] = 0.f;
  }
    for(int i = 0; i < (sizeof(extra_data_out_copy)/sizeof(float)); i++ ){
    extra_data_out_copy[i] = 0.f;
  }

  //Init filters for current accelerations
  float tau_indi = 1.0f / (filter_cutoff_frequency);
  for (int i = 0; i < 6; i++) {
    init_butterworth_2_low_pass(&current_accelerations_filtered[i], tau_indi, refresh_time_optimizer, 0.0);
  }

  //init waiting timer: 
  gettimeofday(&time_last_opt_run, NULL);
  gettimeofday(&time_last_filt, NULL);

}

void am7_parse_msg_in(){

  pthread_mutex_lock(&mutex_am7);
  memcpy(&myam7_data_in, &am7_msg_buf_in[1], sizeof(struct am7_data_in));
  received_msg_id = myam7_data_in.rolling_msg_in_id;
  extra_data_in[received_msg_id] = myam7_data_in.rolling_msg_in;
  //Save the pwm for the servos: 
  angle_1_pwm =  (myam7_data_in.pwm_servo_1_int);
  angle_2_pwm =  (myam7_data_in.pwm_servo_2_int);
  pthread_mutex_unlock(&mutex_am7); 
}

void writing_routine(){

    pthread_mutex_lock(&mutex_am7);
    memcpy(&myam7_data_out, &myam7_data_out_copy, sizeof(struct am7_data_out));
    memcpy(&extra_data_out, &extra_data_out_copy, sizeof(extra_data_out));
    pthread_mutex_unlock(&mutex_am7);
    myam7_data_out.rolling_msg_out_id = sent_msg_id;
    
    uint8_T *buf_out = (uint8_T *)&myam7_data_out;
    //Calculating the checksum
    uint8_T checksum_out_local = 0;

    for(uint16_T i = 0; i < sizeof(struct am7_data_out) - 1; i++){
      checksum_out_local += buf_out[i];
    }
    myam7_data_out.checksum_out = checksum_out_local;
    //Send bytes
    serialPutchar(serial_port, START_BYTE);
    for(int i = 0; i < sizeof(struct am7_data_out); i++){
      serialPutchar(serial_port, buf_out[i]);
    }
    sent_packets++;
    sent_packets_tot++;
    gettimeofday(&last_sent_msg_time, NULL);

    //Increase the counter to track the sending messages:
    sent_msg_id++;
    if(sent_msg_id == 255){
        sent_msg_id = 0;
    }
}

void reading_routine(){
    uint8_T am7_byte_in;
    while(serialDataAvail(serial_port)){
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
          am7_parse_msg_in();
          received_packets++;
          received_packets_tot++;
        }
        else {
          missed_packets++;
        }
      }
    }
} 

void print_statistics(){
  gettimeofday(&current_time, NULL); 
  if((current_time.tv_sec*1e6 + current_time.tv_usec) - (last_time.tv_sec*1e6 + last_time.tv_usec) > 2*1e6){
    received_packets = 0;
    sent_packets = 0;
    gettimeofday(&last_time, NULL);
    printf("Total received packets = %d \n",received_packets_tot);
    printf("Total sent packets = %d \n",sent_packets_tot);
    printf("Tracking sent message id = %d \n",sent_msg_id);
    printf("Tracking received message id = %d \n",received_msg_id);
    printf("Corrupted packet received = %d \n",missed_packets);
    printf("Average message RX frequency = %f \n",ca7_message_frequency_RX);
    printf("Average message TX frequency = %f \n",ca7_message_frequency_TX);
    fflush(stdout);
  }
  ca7_message_frequency_RX = (received_packets*1e6)/((current_time.tv_sec*1e6 + current_time.tv_usec) - (last_time.tv_sec*1e6 + last_time.tv_usec));
  ca7_message_frequency_TX = (sent_packets*1e6)/((current_time.tv_sec*1e6 + current_time.tv_usec) - (last_time.tv_sec*1e6 + last_time.tv_usec));
}

void send_receive_am7(){

  //Reading routine: 
  reading_routine();
  //Writing routine with protection to not exceed desired packet frequency:
  gettimeofday(&current_time, NULL); 
  if((current_time.tv_sec*1e6 + current_time.tv_usec) - (last_sent_msg_time.tv_sec*1e6 + last_sent_msg_time.tv_usec) > (1e6/MAX_FREQUENCY_MSG_OUT)){
    writing_routine();
  }

  //Print some stats
  if(verbose_connection){
    print_statistics();
  }

}

void write_to_servos(){

  softServoWrite (4,  angle_1_pwm - 1000) ;
  softServoWrite (5,  angle_2_pwm - 1000) ;

  //Print some stats
  if(verbose_servo){
    printf("Servo 1 PWM sent = %f \n",(angle_1_pwm+1000));
    printf("Servo 2 PWM sent = %f \n",(angle_2_pwm+1000));
    fflush(stdout);
  }

}

void* first_thread() //Receive and send messages to pixhawk
{
  while(1){ 
    send_receive_am7();
    write_to_servos();
  }
}

void* second_thread() //Run the optimization code 
{

  while(1){ 

    //Declaration of variables for filtering: 
    double current_estimated_accelerations_array[6]; 
    double current_estimated_accelerations_input[6]; 

    pthread_mutex_lock(&mutex_am7);   
    memcpy(&myam7_data_in_copy, &myam7_data_in, sizeof(struct am7_data_in));
    memcpy(&extra_data_in_copy, &extra_data_in, sizeof(extra_data_in));
    pthread_mutex_unlock(&mutex_am7);

    //Do your things with myam7_data_in_copy and extra_data_in_copy as input and myam7_data_out_copy_internal and extra_data_out_copy as outputs

    //Init the variables for the function:
    double u_out[15];
    double dv[6];
    double residuals[6];
    double elapsed_time; 
    double N_iterations; 
    double N_evaluation;
    double exitflag;

    //Assign variables coming from the communication and physical system properties

	  // Float msg variables
    float K_p_T = extra_data_in_copy[0], K_p_M = extra_data_in_copy[1], m = extra_data_in_copy[2], I_xx = extra_data_in_copy[3];
    float I_yy = extra_data_in_copy[4], I_zz = extra_data_in_copy[5], l_1 = extra_data_in_copy[6], l_2 = extra_data_in_copy[7];
    float l_3 = extra_data_in_copy[8], l_4 = extra_data_in_copy[9], l_z = extra_data_in_copy[10];
    float max_omega = extra_data_in_copy[11], min_omega = extra_data_in_copy[12], max_b = extra_data_in_copy[13], min_b = extra_data_in_copy[14];
    float max_g = extra_data_in_copy[15], min_g = extra_data_in_copy[16], max_theta = extra_data_in_copy[17], min_theta = extra_data_in_copy[18];
    float max_alpha = extra_data_in_copy[19]* M_PI/180, min_alpha = extra_data_in_copy[20]* M_PI/180, max_phi = extra_data_in_copy[21];
    float Cm_zero = extra_data_in_copy[22], Cm_alpha = extra_data_in_copy[23], Cl_alpha = extra_data_in_copy[24], Cd_zero = extra_data_in_copy[25];
    float K_Cd = extra_data_in_copy[26], S = extra_data_in_copy[27], wing_chord = extra_data_in_copy[28], rho = extra_data_in_copy[29];

    float W_act_motor_const = extra_data_in_copy[30], W_act_motor_speed = extra_data_in_copy[31]; 
    float W_act_tilt_el_const = extra_data_in_copy[32], W_act_tilt_el_speed = extra_data_in_copy[33]; 
    float W_act_tilt_az_const = extra_data_in_copy[34], W_act_tilt_az_speed = extra_data_in_copy[35]; 
    float W_act_theta_const = extra_data_in_copy[36], W_act_theta_speed = extra_data_in_copy[37]; 
    float W_act_phi_const = extra_data_in_copy[38], W_act_phi_speed = extra_data_in_copy[39]; 
    float W_dv_1 = extra_data_in_copy[40], W_dv_2 = extra_data_in_copy[41], W_dv_3 = extra_data_in_copy[42];
    float W_dv_4 = extra_data_in_copy[43], W_dv_5 = extra_data_in_copy[44], W_dv_6 = extra_data_in_copy[45];
    float gamma_quadratic = extra_data_in_copy[46];
	
	  float Cy_beta = extra_data_in_copy[47], Cl_beta = extra_data_in_copy[48], wing_span = extra_data_in_copy[49];

    float speed_aoa_protection = extra_data_in_copy[50];

    float W_act_ailerons_const = extra_data_in_copy[51], W_act_ailerons_speed = extra_data_in_copy[52]; 
    float min_delta_ailerons = extra_data_in_copy[53], max_delta_ailerons = extra_data_in_copy[54];
    float CL_aileron = extra_data_in_copy[55];
    float transition_airspeed = extra_data_in_copy[56];
    float use_acc_filtering = extra_data_in_copy[57];

	  // Real time variables:
    double Phi = (myam7_data_in_copy.phi_state_int*1e-2 * M_PI/180);
    double Theta = (myam7_data_in_copy.theta_state_int*1e-2 * M_PI/180);
    double delta_ailerons = (myam7_data_in_copy.ailerons_state_int*1e-2 * M_PI/180);
    double Omega_1 = (myam7_data_in_copy.motor_1_state_int*1e-1), Omega_2 = (myam7_data_in_copy.motor_2_state_int*1e-1);
    double Omega_3 = (myam7_data_in_copy.motor_3_state_int*1e-1), Omega_4 = (myam7_data_in_copy.motor_4_state_int*1e-1);
    double b_1 = (myam7_data_in_copy.el_1_state_int*1e-2 * M_PI/180), b_2 = (myam7_data_in_copy.el_2_state_int*1e-2 * M_PI/180);
    double b_3 = (myam7_data_in_copy.el_3_state_int*1e-2 * M_PI/180), b_4 = (myam7_data_in_copy.el_4_state_int*1e-2 * M_PI/180);
    double g_1 = (myam7_data_in_copy.az_1_state_int*1e-2 * M_PI/180), g_2 = (myam7_data_in_copy.az_2_state_int*1e-2 * M_PI/180);
    double g_3 = (myam7_data_in_copy.az_3_state_int*1e-2 * M_PI/180), g_4 = (myam7_data_in_copy.az_4_state_int*1e-2 * M_PI/180);
    double p = (myam7_data_in_copy.p_state_int*1e-1 * M_PI/180), q = (myam7_data_in_copy.q_state_int*1e-1 * M_PI/180); 
    double r = (myam7_data_in_copy.r_state_int*1e-1 * M_PI/180), V = (myam7_data_in_copy.airspeed_state_int*1e-2);
    double flight_path_angle = (myam7_data_in_copy.gamma_state_int*1e-2 * M_PI/180);
	  double Beta = (myam7_data_in_copy.beta_state_int*1e-2 * M_PI/180);
    double desired_motor_value = (myam7_data_in_copy.desired_motor_value_int*1e-1), desired_el_value = (myam7_data_in_copy.desired_el_value_int*1e-2 * M_PI/180);
    double desired_az_value = (myam7_data_in_copy.desired_az_value_int*1e-2 * M_PI/180), desired_theta_value = (myam7_data_in_copy.desired_theta_value_int*1e-2 * M_PI/180);
    double desired_phi_value = (myam7_data_in_copy.desired_phi_value_int*1e-2 * M_PI/180);
    double desired_ailerons_value = (myam7_data_in_copy.desired_ailerons_value_int*1e-2 * M_PI/180);
    dv[0] = (myam7_data_in_copy.pseudo_control_ax_int*1e-2); dv[1] = (myam7_data_in_copy.pseudo_control_ay_int*1e-2);
    dv[2] = (myam7_data_in_copy.pseudo_control_az_int*1e-2); dv[3] = (myam7_data_in_copy.pseudo_control_p_dot_int*1e-1 * M_PI/180);
    dv[4] = (myam7_data_in_copy.pseudo_control_q_dot_int*1e-1 * M_PI/180); dv[5] = (myam7_data_in_copy.pseudo_control_r_dot_int*1e-1 * M_PI/180);

    #ifdef TEST_CONTROLLER
    #warning "You are using the testing variable, watch out!"
      float pi = M_PI;
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

      speed_aoa_protection = 3;
      
      Beta = 0 * pi/180;
      flight_path_angle = 0 * pi/180;
      V = 10;
      Phi = 0 * pi/180;
      Theta = 0 * pi/180;
      Omega_1 = 600;
      Omega_2 = 600;
      Omega_3 = 600;
      Omega_4 = 600;
      b_1 = 0 * pi/180;
      b_2 = 0 * pi/180;
      b_3 = 0 * pi/180;
      b_4 = 0 * pi/180;
      g_1 = 0 * pi/180;
      g_2 = 0 * pi/180;
      g_3 = 0 * pi/180;
      g_4 = 0 * pi/180;
      delta_ailerons = 0 * pi/180;

      W_act_motor_const = 3;
      W_act_motor_speed = 0; 
      W_act_tilt_el_const = 0;
      W_act_tilt_el_speed = 0;
      W_act_tilt_az_const = 0; 
      W_act_tilt_az_speed = 0;
      W_act_theta_const = 100;
      W_act_theta_speed = -15;
      W_act_phi_const = 100;
      W_act_phi_speed = -15;
      W_act_ailerons_const = 10;
      W_act_ailerons_speed = -3; 

      W_dv_1 = 0.03;
      W_dv_2 = 0.03;
      W_dv_3 = 0.06; 
      W_dv_4 = 0.09; 
      W_dv_5 = 0.09; 
      W_dv_6 = 0.09;
      gamma_quadratic = .5e-5; 

      max_omega = 1000; 
      min_omega = 100;
      max_b = 25; 
      min_b = -130; 
      max_g = 45; 
      min_g = -45;  
      max_theta = 60;
      min_theta = -15;
      max_phi = 70; 
      max_delta_ailerons = 25;
      min_delta_ailerons = -25; 

      dv[0] = 0;
      dv[1] = 5;
      dv[2] = -5;
      dv[3] = 5;
      dv[4] = 5;
      dv[5] = 5;

      p = 0 * pi/180; 
      q = 0 * pi/180; 
      r = 0 * pi/180; 
      Cm_zero = 0.05; 
      Cl_alpha = 3.5; 
      Cd_zero = 0.25; 
      K_Cd = 0.08;
      Cm_alpha = -0.1; 
      CL_aileron = 0.1; 
      rho = 1.225; 
      S = 0.43;
      wing_chord = 0.3; 
      max_alpha = 15 * pi/180; 
      min_alpha = 1 * pi/180; 

      desired_motor_value = 100; 
      desired_el_value = 0 * pi/180;
      desired_az_value = 0 * pi/180;
      desired_theta_value = 0 * pi/180; 
      desired_phi_value = 0 * pi/180; 
      desired_ailerons_value = 0 * pi/180;

      transition_airspeed = 9; 

    #endif 

    double cost_value;

    //Compute modeled acelerations: 
    double u_in[15] = {Omega_1, Omega_2, Omega_3, Omega_4, b_1, b_2, b_3, b_4, g_1, g_2, g_3, g_4, Theta, Phi, delta_ailerons};

    c_compute_acc_nonlinear_control( u_in,  p,  q,  r,  K_p_T,
     K_p_M,  m,  I_xx,  I_yy,  I_zz,  l_1,
     l_2,  l_3,  l_4,  l_z,  Cl_alpha,
     Cd_zero,  K_Cd,  Cm_alpha,  Cm_zero,
     CL_aileron,  rho,  V,  S,  wing_chord,
     flight_path_angle,  Beta, current_estimated_accelerations_array);

    if(use_acc_filtering >= 0.5f){
      //Apply filtering to modeled accelerations:
      for(int i = 0; i < 6; i++){

        update_butterworth_2_low_pass(&current_accelerations_filtered[i], (float) current_estimated_accelerations_array[i]);


        if(current_accelerations_filtered[i].o[0] != current_accelerations_filtered[i].o[0]){
          float tau_indi = 1.0f / (filter_cutoff_frequency);
          init_butterworth_2_low_pass(&current_accelerations_filtered[i], tau_indi, refresh_time_optimizer, 0.0);
          if(verbose_filters){
            printf("WARNING, FILTERS %d REINITIALIZED!!!! \n",i);
          }
          
        }

        current_estimated_accelerations_input[i] = (double) current_accelerations_filtered[i].o[0];
      }
    }
    else{
      //Don't filter modeled accelerations output:
      for(int i = 0; i < 6; i++){
        current_estimated_accelerations_input[i] = (double) current_estimated_accelerations_array[i];
      }
    }



    Nonlinear_controller_w_ailerons_rebuttal(
        K_p_T,  K_p_M,  m,  I_xx,  I_yy,  I_zz,
        l_1,  l_2,  l_3,  l_4,  l_z,  Phi,
        Theta,  Omega_1,  Omega_2,  Omega_3,
        Omega_4,  b_1,  b_2,  b_3,  b_4,  g_1,
        g_2,  g_3,  g_4,  delta_ailerons,
        W_act_motor_const,  W_act_motor_speed,
        W_act_tilt_el_const,  W_act_tilt_el_speed,
        W_act_tilt_az_const,  W_act_tilt_az_speed,
        W_act_theta_const,  W_act_theta_speed,  W_act_phi_const,
        W_act_phi_speed,  W_act_ailerons_const,
        W_act_ailerons_speed,  W_dv_1,  W_dv_2,  W_dv_3,
        W_dv_4,  W_dv_5,  W_dv_6,  max_omega,
        min_omega,  max_b,  min_b,  max_g,  min_g,
        max_theta,  min_theta,  max_phi,
        max_delta_ailerons,  min_delta_ailerons, dv,
        p,  q,  r,  Cm_zero,  Cl_alpha,
        Cd_zero,  K_Cd,  Cm_alpha,  CL_aileron,  rho,
        V,  S,  wing_chord,  flight_path_angle,
        max_alpha,  min_alpha,  Beta,  gamma_quadratic,
        desired_motor_value,  desired_el_value,
        desired_az_value,  desired_theta_value,
        desired_phi_value,  desired_ailerons_value, verbose_optimizer,
        speed_aoa_protection,  transition_airspeed,
        current_estimated_accelerations_input,  u_out,  residuals,
        &elapsed_time,  &N_iterations,  &N_evaluation,
        &cost_value,  &exitflag);


    //Convert the function output into integer to be transmitted to the pixhawk again: 
    myam7_data_out_copy_internal.motor_1_cmd_int = (int16_T) (u_out[0]*1e1) , myam7_data_out_copy_internal.motor_2_cmd_int = (int16_T) (u_out[1]*1e1);
    myam7_data_out_copy_internal.motor_3_cmd_int = (int16_T) (u_out[2]*1e1) , myam7_data_out_copy_internal.motor_4_cmd_int = (int16_T) (u_out[3]*1e1);
    myam7_data_out_copy_internal.el_1_cmd_int = (int16_T) (u_out[4]*1e2*180/M_PI), myam7_data_out_copy_internal.el_2_cmd_int = (int16_T) (u_out[5]*1e2*180/M_PI);
    myam7_data_out_copy_internal.el_3_cmd_int = (int16_T) (u_out[6]*1e2*180/M_PI), myam7_data_out_copy_internal.el_4_cmd_int = (int16_T) (u_out[7]*1e2*180/M_PI);
    myam7_data_out_copy_internal.az_1_cmd_int = (int16_T) (u_out[8]*1e2*180/M_PI), myam7_data_out_copy_internal.az_2_cmd_int = (int16_T) (u_out[9]*1e2*180/M_PI);
    myam7_data_out_copy_internal.az_3_cmd_int = (int16_T) (u_out[10]*1e2*180/M_PI), myam7_data_out_copy_internal.az_4_cmd_int = (int16_T) (u_out[11]*1e2*180/M_PI);
    myam7_data_out_copy_internal.theta_cmd_int = (int16_T) (u_out[12]*1e2*180/M_PI), myam7_data_out_copy_internal.phi_cmd_int = (int16_T) (u_out[13]*1e2*180/M_PI);
    myam7_data_out_copy_internal.ailerons_cmd_int = (int16_T) (u_out[14]*1e2*180/M_PI);
    #ifdef SEND_MODELED_ACC_ON_RESIDUALS
    myam7_data_out_copy_internal.residual_ax_int = (int16_T) (current_accelerations_filtered[0].o[0]*1e2), myam7_data_out_copy_internal.residual_ay_int = (int16_T) (current_accelerations_filtered[1].o[0]*1e2);
    myam7_data_out_copy_internal.residual_az_int = (int16_T) (current_accelerations_filtered[2].o[0]*1e2), myam7_data_out_copy_internal.residual_p_dot_int = (int16_T) (current_accelerations_filtered[3].o[0]*1e1*180/M_PI);
    myam7_data_out_copy_internal.residual_q_dot_int = (int16_T) (current_accelerations_filtered[4].o[0]*1e1*180/M_PI), myam7_data_out_copy_internal.residual_r_dot_int = (int16_T) (current_accelerations_filtered[5].o[0]*1e1*180/M_PI);
    #else
    myam7_data_out_copy_internal.residual_ax_int = (int16_T) (residuals[0]*1e2), myam7_data_out_copy_internal.residual_ay_int = (int16_T) (residuals[1]*1e2);
    myam7_data_out_copy_internal.residual_az_int = (int16_T) (residuals[2]*1e2), myam7_data_out_copy_internal.residual_p_dot_int = (int16_T) (residuals[3]*1e1*180/M_PI);
    myam7_data_out_copy_internal.residual_q_dot_int = (int16_T) (residuals[4]*1e1*180/M_PI), myam7_data_out_copy_internal.residual_r_dot_int = (int16_T) (residuals[5]*1e1*180/M_PI);
    #endif

    myam7_data_out_copy_internal.n_iteration = (uint16_T) (N_iterations), myam7_data_out_copy_internal.exit_flag_optimizer = (int16_T) (exitflag);
    myam7_data_out_copy_internal.elapsed_time_us = (uint16_T) (elapsed_time * 1e6), myam7_data_out_copy_internal.n_evaluation = (uint16_T) (N_evaluation);
    
    //Wait until time is not at least refresh_time_optimizer
    while(((current_time.tv_sec*1e6 + current_time.tv_usec) - (time_last_opt_run.tv_sec*1e6 + time_last_opt_run.tv_usec)) < refresh_time_optimizer*1e6){gettimeofday(&current_time, NULL);}

    //Print performances if needed
    if(verbose_runtime){
      printf("\n Elapsed time optimizer = %f \n",(float) elapsed_time); 
      printf("\n Elapsed time filters = %f \n", (float) ((current_time.tv_sec*1e6 + current_time.tv_usec) - (time_last_opt_run.tv_sec*1e6 + time_last_opt_run.tv_usec))); 
      fflush(stdout);
    }

    //Reset waiting timer: 
    gettimeofday(&time_last_opt_run, NULL);

    //Assign the rolling messages to the appropriate value [dummy]
    extra_data_out_copy[0] = 1.453; 
    extra_data_out_copy[1] = 1.23423; 
    
    pthread_mutex_lock(&mutex_am7);
    memcpy(&myam7_data_out_copy, &myam7_data_out_copy_internal, sizeof(struct am7_data_out));
    memcpy(&extra_data_out, &extra_data_out_copy, sizeof(struct am7_data_out));
    pthread_mutex_unlock(&mutex_am7);    
  }
}

void main() {

  //Initialize the serial 
  am7_init();

  pthread_t thread1, thread2;

  // make threads
  pthread_create(&thread1, NULL, first_thread, NULL);
  pthread_create(&thread2, NULL, second_thread, NULL);

  // wait for them to finish
  pthread_join(thread1, NULL);
  pthread_join(thread2, NULL); 

  //Close the serial and clean the variables 
  fflush (stdout);
  close(serial_port);
}
