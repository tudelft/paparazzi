#include "am7x.h"
#include <pthread.h>
#include <string.h>
#include <glib.h>
#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>


int filter_cutoff_frequency;

float tau_indi;

double p_old, q_old, r_old; 

double u_init[NUM_ACT_IN_U_IN];

//To test the controller with random variables:
// #define TEST_CONTROLLER

struct am7_data_out myam7_data_out;
struct am7_data_out myam7_data_out_copy;
struct am7_data_out myam7_data_out_copy_internal;
struct am7_data_in myam7_data_in;
struct am7_data_in myam7_data_in_copy;
struct am7_data_in myam7_data_in_sixdof_copy;
float extra_data_in[255], extra_data_in_copy[255];
float extra_data_out[255], extra_data_out_copy[255];
uint16_t buffer_in_counter;
char am7_msg_buf_in[sizeof(struct am7_data_in)*2]  __attribute__((aligned));
char am7_msg_buf_out[sizeof(struct am7_data_out)]  __attribute__((aligned));
uint32_t received_packets = 0, received_packets_tot = 0;
uint32_t sent_packets = 0, sent_packets_tot = 0;
uint32_t missed_packets = 0;
uint16_t sent_msg_id = 0, received_msg_id = 0;
int serial_port;
int serial_port_tf_mini;
float ca7_message_frequency_RX, ca7_message_frequency_TX;
struct timeval current_time, last_time, last_sent_msg_time, aruco_time, starting_time_program_execution, time_last_opt_run, time_last_filt;


pthread_mutex_t mutex_am7;

int verbose_sixdof_com = 0;
int verbose_sixdof_position = 0;
int verbose_connection = 1;
int verbose_optimizer = 0;
int verbose_runtime = 0; 
int verbose_received_data = 0; 
int verbose_ivy_bus = 0; 
int verbose_aruco = 1; 
int verbose_compute_accel = 0; 
int verbose_filters = 0; 

int16_t lidar_dist_cm = -1; 
int16_t lidar_signal_strength = -1; 

//COMPUTER VISION 
struct aruco_detection_t aruco_detection; 
pthread_mutex_t mutex_aruco;

//SIXDOF VARIABLES: 
int default_desired_sixdof_mode = 1, desired_sixdof_mode; //1 -->rel beacon pos; 2 -->rel beacon angle; 3-->sixdof mode. Default is 1. 
int default_beacon_tracking_id = 1640, beacon_tracking_id; 
float max_tolerance_variance_sixdof = 1; //meters
int current_sixdof_mode; 

float phi_dot_cmd, theta_dot_cmd, phi_cmd_old, theta_cmd_old, phi_dot_cmd_filt, theta_dot_cmd_filt; 
float tau_first_order_attitude_cmd_filtering = 0.1175; //1-exp(-cutoff_frequency/sampling_frequency)

// Transpose an array from body reference frame to earth reference frame
void from_body_to_earth(float * out_array, float * in_array, float Phi, float Theta, float Psi){

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

  //Init serial port for the communication
  if ((serial_port = serialOpen ("/dev/ttyS5", BAUDRATE_AM7)) < 0){
    fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno)) ;
  }
  if (wiringPiSetup () == -1){
    fprintf (stdout, "Unable to start wiringPi: %s\n", strerror (errno)) ;
  }
  sent_msg_id = 0;

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

  //Initialize the extra messages value 
  for(int i = 0; i < (sizeof(extra_data_in)/sizeof(float)); i++ ){
    extra_data_in[i] = 0.f;
  }
    for(int i = 0; i < (sizeof(extra_data_out_copy)/sizeof(float)); i++ ){
    extra_data_out_copy[i] = 0.f;
  }

  //init waiting timer: 
  gettimeofday(&time_last_opt_run, NULL);
  gettimeofday(&time_last_filt, NULL);
  gettimeofday(&starting_time_program_execution, NULL); 

}

void am7_parse_msg_in(){
  pthread_mutex_lock(&mutex_am7);
  memcpy(&myam7_data_in, &am7_msg_buf_in[1], sizeof(struct am7_data_in));
  received_msg_id = myam7_data_in.rolling_msg_in_id;
  extra_data_in[received_msg_id] = myam7_data_in.rolling_msg_in;
  pthread_mutex_unlock(&mutex_am7); 
}

void writing_routine(){

    pthread_mutex_lock(&mutex_am7);
    memcpy(&myam7_data_out, &myam7_data_out_copy, sizeof(struct am7_data_out));
    memcpy(&extra_data_out, &extra_data_out_copy, sizeof(extra_data_out));
    pthread_mutex_unlock(&mutex_am7);
    myam7_data_out.rolling_msg_out_id = sent_msg_id;
    
    //Add the lidar message to the data out value: 
    myam7_data_out.lidar_value_cm = lidar_dist_cm;
    myam7_data_out.lidar_strength = lidar_signal_strength;

    uint8_t *buf_out = (uint8_t *)&myam7_data_out;
    //Calculating the checksum
    uint8_t checksum_out_local = 0;

    for(uint16_t i = 0; i < sizeof(struct am7_data_out) - 1; i++){
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
    uint8_t am7_byte_in;
    if(serialDataAvail(serial_port) > 0){
      am7_byte_in = serialGetchar (serial_port);      
      if( (am7_byte_in == START_BYTE) || (buffer_in_counter > 0)){
        am7_msg_buf_in[buffer_in_counter] = am7_byte_in;
        buffer_in_counter ++;  
      }
      if (buffer_in_counter > sizeof(struct am7_data_in)){
        buffer_in_counter = 0;
        uint8_t checksum_in_local = 0;
        for(uint16_t i = 1; i < sizeof(struct am7_data_in) ; i++){
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
    else{
      usleep(20);
    }
} 

void send_states_on_ivy(){
    //Copy received message from UAV autopilot 
    struct am7_data_in myam7_data_in_copy_for_ivy;
    pthread_mutex_lock(&mutex_am7);   
    memcpy(&myam7_data_in_copy_for_ivy, &myam7_data_in, sizeof(struct am7_data_in));
    pthread_mutex_unlock(&mutex_am7);

    //Send messages over ivy bus for the python thread: 
    if(verbose_ivy_bus) printf("Sent received message from UAV on ivy bus\n");
    IvySendMsg("1 ROTORCRAFT_FP  %d %d %d  %d %d %d  %d %d %d  %d %d %d  %d %d %d",

            (int32_t) (0/0.0039063), // y
            (int32_t) (0/0.0039063), // x
            (int32_t) (0/0.0039063), // z

            (int32_t) (-1/0.0000019),
            (int32_t) (-1/0.0000019),
            (int32_t) (-1/0.0000019),

            (int32_t) ( (10*0.01) /0.0139882),
            (int32_t) ( (10*0.01) /0.0139882),
            (int32_t) ( (10*0.01) /0.0139882),

            (int32_t) (-1/0.0039063),
            (int32_t) (-1/0.0039063),
            (int32_t) (-1/0.0039063),

            (int32_t) (-1/0.0039063),
            (int32_t) (-1/0.0039063),
            (uint16_t) (-1/0.0039063));
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
    printf("Current lidar distance in cm = %d \n",lidar_dist_cm);
    printf("Current lidar strength = %d \n",lidar_signal_strength);
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
    send_states_on_ivy();
  }

  //Print some stats
  if(verbose_connection){
    print_statistics();
  }

}

void* first_thread() //Receive and send messages to pixhawk
{
  while(1){ 
    send_receive_am7();
    readLiDAR();
  }
}

void* second_thread() //Run the optimization code 
{

  while(1){ 

    pthread_mutex_lock(&mutex_am7);   
    memcpy(&myam7_data_in_copy, &myam7_data_in, sizeof(struct am7_data_in));
    memcpy(&extra_data_in_copy, &extra_data_in, sizeof(extra_data_in));
    pthread_mutex_unlock(&mutex_am7);

    //Do your things with myam7_data_in_copy and extra_data_in_copy as input and myam7_data_out_copy_internal and extra_data_out_copy as outputs

    usleep(20);

    struct aruco_detection_t aruco_detection_local; 
  
    pthread_mutex_lock(&mutex_aruco);
    memcpy(&aruco_detection_local, &aruco_detection, sizeof(struct aruco_detection_t));
    pthread_mutex_unlock(&mutex_aruco); 

    myam7_data_out_copy_internal.aruco_detection_timestamp = aruco_detection_local.timestamp_detection;
    myam7_data_out_copy_internal.aruco_NED_pos_x = aruco_detection_local.NED_pos_x;
    myam7_data_out_copy_internal.aruco_NED_pos_y = aruco_detection_local.NED_pos_y;
    myam7_data_out_copy_internal.aruco_NED_pos_z = aruco_detection_local.NED_pos_z;
    myam7_data_out_copy_internal.aruco_system_status = aruco_detection_local.system_status;
    
    //Copy out structure
    pthread_mutex_lock(&mutex_am7);
    memcpy(&myam7_data_out_copy, &myam7_data_out_copy_internal, sizeof(struct am7_data_out));
    memcpy(&extra_data_out, &extra_data_out_copy, sizeof(struct am7_data_out));
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
      struct am7_data_in myam7_data_in_sixdof_copy;
      pthread_mutex_lock(&mutex_am7);
      memcpy(&myam7_data_in_sixdof_copy, &myam7_data_in, sizeof(struct am7_data_in));
      pthread_mutex_unlock(&mutex_am7); 
      float UAV_NED_pos[3] = {myam7_data_in_sixdof_copy.UAV_NED_pos_x, myam7_data_in_sixdof_copy.UAV_NED_pos_y, myam7_data_in_sixdof_copy.UAV_NED_pos_z}; 
      float UAV_euler_angles_rad[3] = {(float) myam7_data_in_sixdof_copy.phi_state_int*1e-2*M_PI/180,
                                      (float) myam7_data_in_sixdof_copy.theta_state_int*1e-2*M_PI/180,
                                      (float) myam7_data_in_sixdof_copy.psi_state_int*1e-2*M_PI/180};

      //Transpose relative position to target position for the UAV: 
      float beacon_absolute_ned_pos[3];
      from_body_to_earth(&beacon_absolute_ned_pos[0], &beacon_rel_pos[0], UAV_euler_angles_rad[0], UAV_euler_angles_rad[1], UAV_euler_angles_rad[2]);
      //Sun current UAV position to have the real abs marker value: 
      for(int i = 0; i < 3; i++){
        beacon_absolute_ned_pos[i] += UAV_NED_pos[i]; 
      }

      //Copy absolute position to 
      struct aruco_detection_t aruco_detection_local; 

      gettimeofday(&aruco_time, NULL); 
      aruco_detection_local.timestamp_detection = (float) timestamp_d;
      aruco_detection_local.NED_pos_x = beacon_absolute_ned_pos[0]; 
      aruco_detection_local.NED_pos_y = beacon_absolute_ned_pos[1];  
      aruco_detection_local.NED_pos_z  = beacon_absolute_ned_pos[2];  
      aruco_detection_local.system_status = (int8_t) current_sixdof_mode;
      
      if(verbose_sixdof_position){
        printf("Sixdof timestamp = %f \n", aruco_detection_local.timestamp_detection); 
        printf("Sixdof NED pos_x = %f \n",(float) aruco_detection_local.NED_pos_x ); 
        printf("Sixdof NED pos_y = %f \n",(float) aruco_detection_local.NED_pos_y ); 
        printf("Sixdof NED pos_z  = %f \n",(float) aruco_detection_local.NED_pos_z ); 
        printf("Sixdof BODY pos_x = %f \n",(float) beacon_rel_pos[0] ); 
        printf("Sixdof BODY pos_y = %f \n",(float) beacon_rel_pos[1] ); 
        printf("Sixdof BODY pos_z  = %f \n \n",(float) beacon_rel_pos[2] ); 
      }

      pthread_mutex_lock(&mutex_aruco);
      memcpy(&aruco_detection, &aruco_detection_local, sizeof(struct aruco_detection_t));
      pthread_mutex_unlock(&mutex_aruco); 
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
      struct am7_data_in myam7_data_in_sixdof_copy;
      pthread_mutex_lock(&mutex_am7);
      memcpy(&myam7_data_in_sixdof_copy, &myam7_data_in, sizeof(struct am7_data_in));
      pthread_mutex_unlock(&mutex_am7); 
      float UAV_NED_pos[3] = {myam7_data_in_sixdof_copy.UAV_NED_pos_x, myam7_data_in_sixdof_copy.UAV_NED_pos_y, myam7_data_in_sixdof_copy.UAV_NED_pos_z}; 
      float UAV_euler_angles_rad[3] = {(float) myam7_data_in_sixdof_copy.phi_state_int*1e-2*M_PI/180,
                                      (float) myam7_data_in_sixdof_copy.theta_state_int*1e-2*M_PI/180,
                                      (float) myam7_data_in_sixdof_copy.psi_state_int*1e-2*M_PI/180};

      //Transpose relative position to target position for the UAV: 
      float beacon_absolute_ned_pos[3];
      from_body_to_earth(&beacon_absolute_ned_pos[0], &local_pos_target_body_rf[0], UAV_euler_angles_rad[0], UAV_euler_angles_rad[1], UAV_euler_angles_rad[2]);
      //Sun current UAV position to have the real abs marker value: 
      for(int i = 0; i < 3; i++){
        beacon_absolute_ned_pos[i] += UAV_NED_pos[i]; 
      }

      //Copy absolute position to aruco struct
      struct aruco_detection_t aruco_detection_local; 
      aruco_detection_local.timestamp_detection = (float) timestamp_d;
      aruco_detection_local.NED_pos_x = beacon_absolute_ned_pos[0]; 
      aruco_detection_local.NED_pos_y = beacon_absolute_ned_pos[1];  
      aruco_detection_local.NED_pos_z  = beacon_absolute_ned_pos[2];  
      aruco_detection_local.system_status = (int8_t) current_sixdof_mode;
      
      if(verbose_sixdof_position){
        printf("Sixdof timestamp = %f \n", aruco_detection_local.timestamp_detection); 
        printf("Sixdof NED pos_x = %f \n",(float) aruco_detection_local.NED_pos_x ); 
        printf("Sixdof NED pos_y = %f \n",(float) aruco_detection_local.NED_pos_y ); 
        printf("Sixdof NED pos_z  = %f \n",(float) aruco_detection_local.NED_pos_z ); 
        printf("Sixdof BODY pos_x = %f \n",(float) local_pos_target_body_rf[0] ); 
        printf("Sixdof BODY pos_y = %f \n",(float) local_pos_target_body_rf[1] ); 
        printf("Sixdof BODY pos_z  = %f \n \n",(float) local_pos_target_body_rf[2] ); 
      }

      pthread_mutex_lock(&mutex_aruco);
      memcpy(&aruco_detection, &aruco_detection_local, sizeof(struct aruco_detection_t));
      pthread_mutex_unlock(&mutex_aruco); 
    }
  }
}

static void aruco_position_report(IvyClientPtr app, void *user_data, int argc, char *argv[])
{
  if (argc != 4)
  {
    fprintf(stderr,"ERROR: invalid message length DESIRED_SP\n");
  }
  struct aruco_detection_t aruco_detection_local; 

  gettimeofday(&aruco_time, NULL); 
  aruco_detection_local.timestamp_detection = (aruco_time.tv_sec*1e6 - starting_time_program_execution.tv_sec*1e6 + aruco_time.tv_usec - starting_time_program_execution.tv_usec)*1e-6;
  aruco_detection_local.NED_pos_x = atof(argv[1]); 
  aruco_detection_local.NED_pos_y = atof(argv[2]);  
  aruco_detection_local.NED_pos_z  = atof(argv[3]);  
  
  if(verbose_aruco){
    printf("\n Aruco timestamp = %f \n", aruco_detection_local.timestamp_detection); 
    printf("\n Aruco NED pos_x = %f \n",(float) aruco_detection_local.NED_pos_x ); 
    printf("\n Aruco NED pos_y = %f \n",(float) aruco_detection_local.NED_pos_y ); 
    printf("\n Aruco NED pos_z  = %f \n",(float) aruco_detection_local.NED_pos_z ); 
  }

  pthread_mutex_lock(&mutex_aruco);
  memcpy(&aruco_detection, &aruco_detection_local, sizeof(struct aruco_detection_t));
  pthread_mutex_unlock(&mutex_aruco); 

}

void main() {

  //Initialize the serial 
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
  IvyBindMsg(aruco_position_report, NULL, "^ground DESIRED_SP %s (\\S*) (\\S*) (\\S*) (\\S*)", "1");
  // IvyBindMsg(sixdof_beacon_pos_callback, NULL, "RELATIVE_BEACON_POS (\\S*) (\\S*) (\\S*) (\\S*) (\\S*)");
  // IvyBindMsg(sixdof_beacon_angle_callback, NULL, "RELATIVE_BEACON_ANGLE (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*)");
  // IvyBindMsg(sixdof_mode_callback, NULL, "SIXDOF_TRACKING (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*)");
  // IvyBindMsg(sixdof_current_mode_callback, NULL, "SIXDOF_SYSTEM_CURRENT_MODE (\\S*) (\\S*)");

  pthread_t thread1, thread2;

  // make threads
  pthread_create(&thread1, NULL, first_thread, NULL);
  pthread_create(&thread2, NULL, second_thread, NULL);

  g_main_loop_run(ml);

  //Close the serial and clean the variables 
  fflush (stdout);
  close(serial_port);
}
