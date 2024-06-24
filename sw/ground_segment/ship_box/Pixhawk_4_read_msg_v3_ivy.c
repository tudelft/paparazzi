#include <stdlib.h>
#include <errno.h>
#include <sys/time.h>
#include <stddef.h>
#include <stdio.h>
#include <unistd.h>
#include <getopt.h>
#include <stdbool.h>
#include <stdint.h>
#include <arpa/inet.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <pthread.h>
#include <string.h>
#include <math.h>
#include <fcntl.h>
#include <termios.h>
#include <glib.h>
#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>
#include <arpa/inet.h>

struct timeval current_time, last_time_rx, last_time_tx;
struct timespec current_timespec;
double last_log_time = 0.0;


static bool verbose = false;
static uint8_t ac_id = 0;
static uint8_t ship_box_id = 0;

static bool save_on_log = false;
static bool send_data_on_tcp = false;
static bool send_data_on_udp = true;
static bool use_cb_on_tcp = false;
static bool use_cb_on_udp = false;

#define GENERATE_DUMMY_VALUES

//Provide path to save the log file: 
static char* log_file_path = "/media/sf_Shared_folder_virtual_machine/Exchange_ship_log/ship_box_log.csv";
#define LOG_FREQUENCY 8 //Hz, integer
#define MAX_LOG_TIME 10 //s, integer

float max_tx_freq = 1500.0; 

//Path to the ship coefficients file:
static char* ship_coeffs_path = "/media/sf_Shared_folder_virtual_machine/Exchange_ship_log/ship_coeffs.csv";

#define BUFFER_SIZE (LOG_FREQUENCY*MAX_LOG_TIME)
bool show_once = true;

#define SERVER_IP "192.168.227.1"
#define SERVER_PORT 8080 // Port number for the server

//Structure definition of the ship states to be logged for the lstm prediction: 
typedef struct {
    double timestamp;
    double speed_x; 
    double speed_x_control;
    double speed_y;
    double speed_y_control;
    double speed_z;
    double phi_dot_deg; 
    double theta_dot_deg;
    double heading_deg;
} ship_state_log_data;

//Structure definition of the circular buffer: 
typedef struct {
    ship_state_log_data buffer[BUFFER_SIZE];
    int start;
    int end;
    int count; 
    bool is_full;
} ship_state_log_buffer;

//Init the circular buffer:
ship_state_log_buffer cb = {
    .start = 0,
    .end = 0,
    .count = 0,
    .is_full = false
};

//Define the payload structure for the SHIP_INFO_MSG_GROUND message:
struct __attribute__((__packed__)) payload_ship_info_msg_ground {
  float phi; 
  float theta; 
  float psi; 
  float heading; 
  float course; 
  float phi_dot; 
  float theta_dot; 
  float psi_dot; 
  float x; 
  float y; 
  float z; 
  float lat; 
  float lon; 
  float alt;   
  float x_dot; 
  float y_dot; 
  float z_dot; 
  float x_ddot; 
  float y_ddot; 
  float z_ddot; 
};

//Function to add a data point to the circular buffer:
void add_data_point_to_log_buffer(ship_state_log_buffer *cb, ship_state_log_data data) {
    cb->buffer[cb->end] = data;
    
    cb->end = (cb->end + 1) % BUFFER_SIZE;
    if (cb->count == BUFFER_SIZE) {
        cb->start = (cb->start + 1) % BUFFER_SIZE;
        cb->is_full = true; // Set buffer full flag
    } else {
        cb->count++;
        if (cb->count == BUFFER_SIZE) {
            cb->is_full = true; // Set buffer full flag when full capacity is reached
        }
    }
}

//Function to send the data to the server through TCP/IP:
void send_to_tcp(ship_state_log_buffer *cb) {
    int sock = 0;
    struct sockaddr_in serv_addr;
    char buffer[1024] = {0};

    // Create socket
    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        perror("Socket creation error");
        return;
    }

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(SERVER_PORT);

    // Convert IP address from text to binary form
    if (inet_pton(AF_INET, SERVER_IP, &serv_addr.sin_addr) <= 0) {
        perror("Invalid address or address not supported");
        close(sock);
        return;
    }

    // Connect to the server
    if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
        perror("Connection failed");
        close(sock);
        return;
    }

    // Send data
    int index = cb->start;
    for (int i = 0; i < cb->count; i++) {
        ship_state_log_data *dp = &cb->buffer[index];
        snprintf(buffer, sizeof(buffer), "%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
                 dp->timestamp,
                 dp->speed_x,
                 dp->speed_x_control,
                 dp->speed_y,
                 dp->speed_y_control,
                 dp->speed_z,
                 dp->phi_dot_deg,
                 dp->theta_dot_deg,
                 dp->heading_deg);

        if(use_cb_on_tcp) send(sock, buffer, strlen(buffer), 0);

        index = (index + 1) % BUFFER_SIZE;
    }
    if(!use_cb_on_tcp) send(sock, buffer, strlen(buffer), 0);

    // Close socket
    close(sock);

    if (verbose) {
        printf("Data sent to server through TCP/IP \n");
        //Show also the last line of data sent:
        printf("Last data added:");
        //printf what is in buffer:
        printf(" %s\n", buffer); 
    }

    if (verbose || (cb->is_full && show_once)) {
        printf("Buffer full status: %s\n", cb->is_full ? "Full" : "Not Full");
        show_once = false;
    }
}

//Function to send the data to the server through UDP/IP:
void send_to_udp(ship_state_log_buffer *cb) {
    int sock;
    struct sockaddr_in server_addr;
    char buffer[1024];

    // Create UDP socket
    if ((sock = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("Socket creation error");
        return;
    }

    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(SERVER_PORT);
    server_addr.sin_addr.s_addr = inet_addr(SERVER_IP);

    // Send data
    int index = cb->start;
    for (int i = 0; i < cb->count; i++) {
        ship_state_log_data *dp = &cb->buffer[index];
        snprintf(buffer, sizeof(buffer), "%f,%f,%f,%f,%f,%f,%f,%f,%f",
                 dp->timestamp,
                 dp->speed_x,
                 dp->speed_x_control,
                 dp->speed_y,
                 dp->speed_y_control,
                 dp->speed_z,
                 dp->phi_dot_deg,
                 dp->theta_dot_deg,
                 dp->heading_deg);

        // Send data to server
        if(use_cb_on_udp) sendto(sock, buffer, strlen(buffer), 0, (struct sockaddr *)&server_addr, sizeof(server_addr));
        index = (index + 1) % BUFFER_SIZE;
    }

    if(!use_cb_on_udp) sendto(sock, buffer, strlen(buffer), 0, (struct sockaddr *)&server_addr, sizeof(server_addr));

    // Close socket
    close(sock);

    if (verbose) {
        printf("Data sent to server through UDP \n");
        //Show also the last line of data sent:
        printf("Last data added:");
        //printf what is in buffer:
        printf(" %s\n", buffer); 
    }
    
    if (verbose || (cb->is_full && show_once)) {
        printf("Buffer full status: %s\n", cb->is_full ? "Full" : "Not Full");
        show_once = false;
    }
}

//Function to write the data to a CSV file:
void write_csv(ship_state_log_buffer *cb, const char *filename) {
    FILE *file = fopen(filename, "w");
    if (file == NULL) {
        perror("Error opening file");
        return;
    }

    fprintf(file, "timestamp,speed_x,speed_y,speed_z,phi_dot,theta_dot,heading\n");
    int index = cb->start;
    for (int i = 0; i < cb->count; i++) {
        ship_state_log_data *dp = &cb->buffer[index];
        fprintf(file, "%f,%f,%f,%f,%f,%f,%f,%f,%f\n", 
                dp->timestamp, 
                dp->speed_x, 
                dp->speed_x_control, 
                dp->speed_y, 
                dp->speed_y_control,
                dp->speed_z, 
                dp->phi_dot_deg, 
                dp->theta_dot_deg,
                dp->heading_deg);
        index = (index + 1) % BUFFER_SIZE;
    }
    fclose(file);
    if (verbose) printf("Data written to CSV file %s\n", filename);

    if(verbose || (cb->is_full && show_once)){
      printf("Buffer full status: %s\n", cb->is_full ? "Full" : "Not Full");
      show_once = false;
    }
}

//Function to log the ship states:
void log_ship_state(ship_state_log_data payload_ship_log){
  //If time is above the period of the log frequency, save the values on the circular buffer:
  if(payload_ship_log.timestamp  - last_log_time >= (1.0/(float) LOG_FREQUENCY)){
    last_log_time = payload_ship_log.timestamp;

    //add the data point to the circular buffer:
    add_data_point_to_log_buffer(&cb, payload_ship_log);

    if(save_on_log) write_csv(&cb, log_file_path);
    if(send_data_on_tcp) send_to_tcp(&cb);
    if(send_data_on_udp) send_to_udp(&cb);
      
  }
}

//Function to log and send the SHIP_INFO_MSG_GROUND message to the drone through the ivyBus:
void ivy_send_ship_info_msg(struct payload_ship_info_msg_ground payload_ship){

  gettimeofday(&current_time, NULL);
  double delta_time = (current_time.tv_sec*1e6 + current_time.tv_usec) - (last_time_tx.tv_sec*1e6 + last_time_tx.tv_usec);
  if(delta_time >= (1e6/max_tx_freq))
  {
    if(verbose) printf("Message SHIP INFO MSG forwarded through ivyBus on ac ID %d: \n",ac_id); 
    if(verbose) printf("Freq tx =  %.2f \n",(1e6/delta_time));  
    IvySendMsg("ground SHIP_INFO_MSG %d  %f %f %f  %f %f %f  %f %f %f  %f %f %f  %f %f %f  %f %f %f",
            ac_id,
            
            payload_ship.phi,
            payload_ship.theta,
            payload_ship.psi,

            payload_ship.phi_dot,
            payload_ship.theta_dot,
            payload_ship.psi_dot,

            payload_ship.x,
            payload_ship.y,
            payload_ship.z,

            payload_ship.lat,
            payload_ship.lon,
            payload_ship.alt,

            payload_ship.x_dot,
            payload_ship.y_dot,
            payload_ship.z_dot,

            payload_ship.x_ddot,
            payload_ship.y_ddot,
            payload_ship.z_ddot);

            gettimeofday(&last_time_tx, NULL);
  }
}

//Callback function to process the SHIP_INFO_MSG_GROUND message received from the ship box:
static void on_ShipInfoMsgGround(IvyClientPtr app, void *user_data, int argc, char *argv[])
{
  gettimeofday(&current_time, NULL);
  double delta_time = (current_time.tv_sec*1e6 + current_time.tv_usec) - (last_time_rx.tv_sec*1e6 + last_time_rx.tv_usec); 
  if(verbose) printf("Frequency of incoming SHIP_INFO_MSG : %.2f \n",(1e6/delta_time));
  gettimeofday(&last_time_rx, NULL);   
  
  if (argc != 20)
  {
    fprintf(stderr,"ERROR: invalid message length SHIP_INFO_MSG_GROUND\n");
  }
  else{
    /* MESSAGE INFO FROM am_messages_new.xml
    <message name="SHIP_INFO_MSG_GROUND" id="192" >
      <field name="phi" type="float" unit="deg">Roll</field>
      <field name="theta" type="float" unit="deg">Pitch</field>
      <field name="psi" type="float" unit="deg">Yaw</field>
      <field name="heading" type="float" unit="deg*1e5">Heading</field>
      <field name="course" type="float" unit="rad*1e5">Heading</field>
      <field name="phi_dot" type="float" unit="deg/s">roll rate</field>
      <field name="theta_dot" type="float" unit="deg/s">pitch rate</field>
      <field name="psi_dot" type="float" unit="deg/s">yaw rate</field>      
      <field name="x" type="float" unit="m">position_x</field>
      <field name="y" type="float" unit="m">position_y</field>
      <field name="z" type="float" unit="m">position_z</field>
      <field name="lat_ship" type="float" unit="m">lat_ship</field>
      <field name="long_ship" type="float" unit="m">long_ship</field>
      <field name="alt_ship" type="float" unit="m">alt_ship</field>      
      <field name="x_dot" type="float" unit="m/s">speed_x</field>
      <field name="y_dot" type="float" unit="m/s">speed_y</field>
      <field name="z_dot" type="float" unit="m/s">speed_z</field>
      <field name="x_ddot" type="float" unit="m/s^2">acc_x</field>      
      <field name="y_ddot" type="float" unit="m/s^2">acc_y</field>      
      <field name="z_ddot" type="float" unit="m/s^2">acc_z</field>                        
    </message>
    */
    struct payload_ship_info_msg_ground payload_ship; 
    payload_ship.phi = atof(argv[0]);
    payload_ship.theta = atof(argv[1]);
    payload_ship.psi = atof(argv[2]);
    payload_ship.heading = atof(argv[3]);
    payload_ship.course = atof(argv[4]);
    payload_ship.phi_dot = atof(argv[5]);
    payload_ship.theta_dot = atof(argv[6]);
    payload_ship.psi_dot = atof(argv[7]);
    payload_ship.x = atof(argv[8]);
    payload_ship.y = atof(argv[9]);
    payload_ship.z = atof(argv[10]);
    payload_ship.lat = atof(argv[11]);
    payload_ship.lon = atof(argv[12]);
    payload_ship.alt = atof(argv[13]);
    payload_ship.x_dot = atof(argv[14]);
    payload_ship.y_dot = atof(argv[15]);
    payload_ship.z_dot = atof(argv[16]);
    payload_ship.x_ddot = atof(argv[17]);
    payload_ship.y_ddot = atof(argv[18]);
    payload_ship.z_ddot = atof(argv[19]);

    ivy_send_ship_info_msg(payload_ship);

    if(verbose){
      printf("Valid SHIP_INFO_MSG_GROUND message received from Ship box \n");
      printf("Ship roll angle [deg] : %f \n",payload_ship.phi);
      printf("Ship theta angle [deg] : %f \n",payload_ship.theta);
      printf("Ship psi angle [deg] : %f \n",payload_ship.psi);
      printf("Ship heading angle [deg] : %f \n",payload_ship.heading*180/M_PI);
      printf("Ship course angle [deg] : %f \n",payload_ship.course*180/M_PI);
      printf("Ship roll rate [deg/s] : %f \n",payload_ship.phi_dot);
      printf("Ship pitch rate [deg/s] : %f \n",payload_ship.theta_dot);
      printf("Ship yaw rate [deg/s] : %f \n",payload_ship.psi_dot);  
      printf("Ship pos x [m] : %f \n",payload_ship.x);  
      printf("Ship pos y [m] : %f \n",payload_ship.y);  
      printf("Ship pos z [m] : %f \n",payload_ship.z);  
      printf("Ship lat [deg] : %f \n",payload_ship.lat);  
      printf("Ship pos lon [deg] : %f \n",payload_ship.lon);  
      printf("Ship pos alt [m] : %f \n",payload_ship.alt);              
      printf("Ship speed x [m/s] : %f \n",payload_ship.x_dot);  
      printf("Ship speed y [m/s] : %f \n",payload_ship.y_dot);  
      printf("Ship speed z [m/s] : %f \n",payload_ship.z_dot);  
      printf("Ship acc x [m/s^2] : %f \n",payload_ship.x_ddot);  
      printf("Ship acc y [m/s^2] : %f \n",payload_ship.y_ddot);  
      printf("Ship acc z [m/s^2] : %f \n",payload_ship.z_ddot);        
    }
    
    //If we want to save the values on the csv file or send them to the TCP server, proceed:
    if(save_on_log || send_data_on_tcp || send_data_on_udp){
      clock_gettime(CLOCK_BOOTTIME, &current_timespec);
      double current_clock_time = current_timespec.tv_sec + current_timespec.tv_nsec*1e-9; 
      //Create the structure to save the values on the log file:
      ship_state_log_data paylod_ship_log = {
        .timestamp = current_clock_time,
        .speed_x = payload_ship.x_dot,
        .speed_x_control = payload_ship.x_dot * cosf(payload_ship.heading) + payload_ship.y_dot * sinf(payload_ship.heading),
        .speed_y = payload_ship.y_dot,
        .speed_y_control = -payload_ship.x_dot * sinf(payload_ship.heading) + payload_ship.y_dot * cosf(payload_ship.heading),
        .speed_z = payload_ship.z_dot,
        .phi_dot_deg = payload_ship.phi_dot*180/M_PI,
        .theta_dot_deg = payload_ship.theta_dot*180/M_PI,
        .heading_deg = payload_ship.heading*180/M_PI
      };
      //Call the log function to save the values on the file:
      log_ship_state(paylod_ship_log);
    }
  
  }
}

//Function to generate dummy values for the ship states:
void generate_dummy_values(){
  struct payload_ship_info_msg_ground payload_ship; 
  while(1){
    clock_gettime(CLOCK_BOOTTIME, &current_timespec);
    double current_clock_time = current_timespec.tv_sec + current_timespec.tv_nsec*1e-9; 
    float omega_speed_x = 0.1;
    float omega_speed_y = 0.2;
    float omega_speed_z = 0.3;
    float omega_phi_dot = 0.4;
    float omega_theta_dot = 0.5;
    float omega_heading = 0.6;

    //Create the structure to save the values on the log file:
    ship_state_log_data paylod_ship_log = {
      .timestamp = current_clock_time,
      .speed_x = 1.0*sinf(2*M_PI*omega_speed_x*current_clock_time),
      .speed_y = 2.0*sinf(2*M_PI*omega_speed_y*current_clock_time),
      .speed_z = 3.0*sinf(2*M_PI*omega_speed_z*current_clock_time),
      .phi_dot_deg = 4.0*sinf(2*M_PI*omega_phi_dot*current_clock_time),
      .theta_dot_deg = 5.0*sinf(2*M_PI*omega_theta_dot*current_clock_time),
      .heading_deg = 180.0*sinf(2*M_PI*omega_heading*current_clock_time),
      .speed_x_control = 1.0*sinf(2*M_PI*omega_speed_x*current_clock_time) * cosf(180.0*sinf(2*M_PI*omega_heading*current_clock_time)*M_PI/180) + payload_ship.y_dot * sinf(180.0*sinf(2*M_PI*omega_heading*current_clock_time)*M_PI/180),
      .speed_y_control = -1.0*sinf(2*M_PI*omega_speed_x*current_clock_time) * sinf(180.0*sinf(2*M_PI*omega_heading*current_clock_time)*M_PI/180) + payload_ship.y_dot * cosf(180.0*sinf(2*M_PI*omega_heading*current_clock_time)*M_PI/180)
    };
    //Call the log function to save the values on the file:
    log_ship_state(paylod_ship_log);
    usleep(10);
  }
}

int main(int argc, char** argv) {
  //Init timers: 
  gettimeofday(&last_time_tx, NULL);
  gettimeofday(&last_time_rx, NULL);

  /* Defaults */
  #ifdef __APPLE__
    char* ivy_bus = "224.255.255.255";
  #else
    char *ivy_bus = "127.255.255.255";
  #endif

  GMainLoop *ml =  g_main_loop_new(NULL, FALSE);

  /* Arguments options and usage information */
  static struct option long_options[] = {
    {"ac_id", required_argument, NULL, 'i'},
    {"ship_box_id", required_argument, NULL, 'e'},
    {"max_tx_freq", required_argument, NULL, 'f'},
    {"help", no_argument, NULL, 'h'},
    {"verbose", no_argument, NULL, 'v'},
    {0, 0, 0, 0}
  };
  static const char* usage =
    "Usage: %s [options]\n"
    " Options :\n"
    "   -i --ac_id [aircraft_id]               Provides Aircraft id\n"
    "   -s --ship_box_id [ship_box_id]         Provides Ship_box_id\n"
    "   -f --max_frequency_out [Hz]            Sets max_frequency_out\n"
    "   -h --help                              Display this help\n"
    "   -v --verbose                           Print verbose information\n";

  int c;
  int option_index = 0;
  while((c = getopt_long(argc, argv, "i:s:f:h:v", long_options, &option_index)) != -1) {
    switch (c) {
      case 'i':
        ac_id = atoi(optarg);
        break;

      case 's':
        ship_box_id = atoi(optarg);
        break;

      case 'f':
        max_tx_freq = atoi(optarg);
        break;

      case 'v':
        verbose = true;
        break;

      case 'h':
        fprintf(stderr, usage, argv[0]);
        return 0;

      default: /* ’?’ */
        printf("?? getopt returned character code %c ??\r\n", c);
        fprintf(stderr, usage, argv[0]);
        return 1;
    }
  }

  IvyInit ("SHIPINFO2Ivy", "SHIPINFO2Ivy READY", NULL, NULL, NULL, NULL);

  IvyBindMsg(on_ShipInfoMsgGround, NULL, "%d SHIP_INFO_MSG_GROUND (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*)", ship_box_id);

  IvyStart(ivy_bus);

  #ifdef GENERATE_DUMMY_VALUES
    //Open a new thread and submit dummy values on the circular buffer: 
    pthread_t thread_id;
    pthread_create(&thread_id, NULL, (void *)generate_dummy_values, NULL);
  #endif
  
  g_main_loop_run(ml);

  return 0;
}