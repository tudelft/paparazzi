/*
 * Paparazzi UBLox to Ivy
 *
 * Copyright (C) 2021 Freek van Tienen <freek.v.tienen@gmail.com>
 *
 * This file is part of paparazzi.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

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

struct timeval current_time, last_time_rx, last_time_tx;
struct timespec current_timespec;
double last_log_time = 0.0;


static bool verbose = false;
static uint8_t ac_id = 0;
static uint8_t ship_box_id = 0;

static bool save_on_log = true;
//Provide path to save the log file: 
static char* log_file_path = "/media/sf_Shared_folder_virtual_machine/Exchange_ship_log/ship_box_log.csv";
#define LOG_FREQUENCY 4 //Hz
#define MAX_LOG_TIME 200 //s

float max_tx_freq = 1500.0; 

//Path to the ship coefficients file:
static char* ship_coeffs_path = "/media/sf_Shared_folder_virtual_machine/Exchange_ship_log/ship_coeffs.csv";

#define BUFFER_SIZE (LOG_FREQUENCY*MAX_LOG_TIME)
bool show_once = true;

//Structure definition of the ship states to be logged for the lstm prediction: 
typedef struct {
    double timestamp;
    double speed_x; 
    double speed_y;
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
        fprintf(file, "%f,%f,%f,%f,%f,%f,%f\n", 
                dp->timestamp, 
                dp->speed_x, 
                dp->speed_y, 
                dp->speed_z, 
                dp->phi_dot_deg, 
                dp->theta_dot_deg,
                dp->heading_deg);
        index = (index + 1) % BUFFER_SIZE;
    }
    fclose(file);
    if (verbose) printf("Data written to file %s\n", filename);

    if(verbose || (cb->is_full && show_once)){
      printf("Buffer full status: %s\n", cb->is_full ? "Full" : "Not Full");
      show_once = false;
    }
}

void log_ship_state(ship_state_log_data payload_ship_log){
  //If time is above the period of the log frequency, save the values on the circular buffer:
  if(payload_ship_log.timestamp  - last_log_time >= (1/LOG_FREQUENCY)){
    last_log_time = payload_ship_log.timestamp;
    //add the data point to the circular buffer:
    add_data_point_to_log_buffer(&cb, payload_ship_log);
    //Write the values on the log file:
    write_csv(&cb, log_file_path);
  }
}

void ivy_send_ship_info_msg(struct payload_ship_info_msg_ground payload_ship){

  gettimeofday(&current_time, NULL);
  float delta_time = (current_time.tv_sec*1e6 + current_time.tv_usec) - (last_time_tx.tv_sec*1e6 + last_time_tx.tv_usec);
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

static void on_ShipInfoMsgGround(IvyClientPtr app, void *user_data, int argc, char *argv[])
{
  gettimeofday(&current_time, NULL);
  float delta_time = (current_time.tv_sec*1e6 + current_time.tv_usec) - (last_time_rx.tv_sec*1e6 + last_time_rx.tv_usec); 
  if(verbose) printf("Frequency of incoming SHIP_INFO_MSG : %.2f \n",(1e6/delta_time));
  gettimeofday(&last_time_rx, NULL);   
  
  if (argc != 20)
  {
    fprintf(stderr,"ERROR: invalid message length SHIP_INFO_MSG_GROUND\n");
  }
  else{
    /*
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
    
    //If we want to save the values on the csv file, proceed saving the values:
    if(save_on_log){
      clock_gettime(CLOCK_BOOTTIME, &current_timespec);
      double current_clock_time = current_timespec.tv_sec + current_timespec.tv_nsec*1e-9; 
      //Create the structure to save the values on the log file:


      ship_state_log_data paylod_ship_log = {
        .timestamp = current_clock_time,
        .speed_x = payload_ship.x_dot,
        .speed_y = payload_ship.y_dot,
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

  g_main_loop_run(ml);

  return 0;
}