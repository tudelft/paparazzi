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
#include "Pixhawk_4_read_msg.h"
#include <string.h>

struct timeval current_time, last_time;

double delta_time[MESSAGE_ON_TX_FREQUENCY_CALCULATION];
float avg_msg_frequency_tx; 
int delta_time_count; 

/* PPRZ message parser states */
enum normal_parser_states {
  SearchingPPRZ_STX,
  ParsingLength,
  ParsingSenderId,  //Sidenote: This is called the Source in case of v2
  ParsingDestination,
  ParsingClassIdAndComponentId,
  ParsingMsgId,
  ParsingMsgPayload,
  CheckingCRCA,
  CheckingCRCB
};

struct payload_ship_info_msg {
  float phi; 
  float theta; 
  float psi; 
  float phi_dot; 
  float theta_dot; 
  float psi_dot; 
  float x; 
  float y; 
  float z; 
  float x_dot; 
  float y_dot; 
  float z_dot; 
  float x_ddot; 
  float y_ddot; 
  float z_ddot; 
};

struct normal_parser_t {
  enum normal_parser_states state;
  unsigned char length;
  int counter;
  unsigned char sender_id; //Note that Source is the official PPRZLink v2 name
  unsigned char destination;
  unsigned char class_id; //HiNibble 4 bits
  unsigned char component_id; //LowNibble 4 bits
  unsigned char msg_id;
  unsigned char payload[256];
  unsigned char crc_a;
  unsigned char crc_b;
};

int verbose; 
int ac_id; 

struct normal_parser_t parser;

struct payload_ship_info_msg paylod_ship; 

char packetBuffer[256]; //buffer to hold incoming packet
char outBuffer[256];    //buffer to hold outgoing data
uint8_t out_idx = 0;
uint8_t serial_connect_info = 1; //If 1 then spit out serial print wifi connection info for debugging purposes

int serial_port_pixhawk_4;

pthread_t pixhawk_4_msg_generator;

/* 
   PPRZLINK v2
   PPRZ-message: ABCxxxxxxxDE
    A PPRZ_STX (0x99)
    B LENGTH (A->E)
    C PPRZ_DATA
      0 SOURCE (~sender_ID)
      1 DESTINATION (can be a broadcast ID)
      2 CLASS/COMPONENT
        bits 0-3: 16 class ID available
        bits 4-7: 16 component ID available
      3 MSG_ID
      4 MSG_PAYLOAD
      . DATA (messages.xml)
    D PPRZ_CHECKSUM_A (sum[B->C])
    E PPRZ_CHECKSUM_B (sum[ck_a])

    Returns 0 if not ready, return 1 if complete message was detected
*/
uint8_t parse_single_byte(unsigned char in_byte)
{

  switch (parser.state) {

    /* Same for PPRZLINK v1 and v2 */
    case SearchingPPRZ_STX:
      out_idx = 0;
      if (in_byte == PPRZ_STX) {
        //printf("Got PPRZ_STX\n");
        parser.crc_a = 0;
        parser.crc_b = 0;
        parser.counter = 1;
        parser.state = ParsingLength;
      }
      break;

    /* Same for PPRZLINK v1 and v2 */
    case ParsingLength:
      parser.length = in_byte;
      parser.crc_a += in_byte;
      parser.crc_b += parser.crc_a;
      parser.counter++;
      parser.state = ParsingSenderId;
      break;


    /* Same for PPRZLINK v1 and v2, however naming is of Source like old SenderId */
    case ParsingSenderId:
      parser.sender_id = in_byte;
      parser.crc_a += in_byte;
      parser.crc_b += parser.crc_a;
      parser.counter++;
      parser.state = ParsingDestination;
      break;


    case ParsingDestination:
      parser.destination = in_byte;
      parser.crc_a += in_byte;
      parser.crc_b += parser.crc_a;
      parser.counter++;
      parser.state = ParsingClassIdAndComponentId;
      break;
    
    case ParsingClassIdAndComponentId:
      parser.class_id = ((unsigned char)in_byte & 0xf0) >> 4; //HiNibble 1st4 bits
      parser.component_id = (unsigned char)in_byte & 0x0f; //LoNibble last 1st4 bits
      parser.crc_a += in_byte;
      parser.crc_b += parser.crc_a;
      parser.counter++;
      parser.state = ParsingMsgId;
      break;

    case ParsingMsgId:
      parser.msg_id = in_byte;
      parser.crc_a += in_byte;
      parser.crc_b += parser.crc_a;
      parser.counter++;
      if (parser.length == 8)          
      { 
        parser.state = CheckingCRCA;
      } else {
        parser.state = ParsingMsgPayload;
      }
      break;

    case ParsingMsgPayload:
      parser.payload[parser.counter-4] = in_byte;
      parser.crc_a += in_byte;
      parser.crc_b += parser.crc_a;
      parser.counter++;
      if (parser.counter == parser.length - 2) {
        parser.state = CheckingCRCA;
      }
      break;

    case CheckingCRCA:
      //printf("CRCA: %d vs %d\n", in_byte, parser.crc_a);
      if (in_byte == parser.crc_a) {
        parser.state = CheckingCRCB;
      }
      else {
        parser.state = SearchingPPRZ_STX;
      }
      break;

    case CheckingCRCB:
      //printf("CRCB: %d vs %d\n", in_byte, parser.crc_b);
      if (in_byte == parser.crc_b) {
        
        /* Check what to do next if the command was received */
        outBuffer[out_idx++] = in_byte; // final byte
        parser.state = SearchingPPRZ_STX;
        return 1;
      }
      parser.state = SearchingPPRZ_STX;
      break;

    default:
      /* Should never get here */
      break;
  }
  
  outBuffer[out_idx++] = in_byte;
  return 0;
}



void Pixhawk_reading_init(){
  //Init serial port for the communication
  if ((serial_port_pixhawk_4 = serialOpen ("/dev/ttyS0", BAUDRATE_PIXHAWK_4)) < 0){
    fprintf (stderr, "Unable to open serial device /dev/ttyS0 to Pixhawk 4. %s\n", strerror (errno)) ;
  }

  /* Start the Pixhawk 4 reading thread */
  if(verbose) printf("Start reading from Pixhawk 4 serial device /dev/ttyS0");
  pthread_create(&pixhawk_4_msg_generator, NULL, pixhawk_fwd_msg, NULL);

}

void* pixhawk_fwd_msg(){ 
  while(1){
    Pixhawk_read_fwd_ship_info_msg();
  }
}

void Pixhawk_read_fwd_ship_info_msg(void){
    if(serialDataAvail(serial_port_pixhawk_4) > 0) { 
      unsigned char inbyte = serialGetchar(serial_port_pixhawk_4);
      if (parse_single_byte(inbyte)) { 
        // if complete message detected save the message on the paylod_ship struct
        if(parser.msg_id == SHIP_INFO_MSG_ID){

          memcpy(&paylod_ship,&parser.payload[2],sizeof(struct payload_ship_info_msg));

          gettimeofday(&current_time, NULL);
          if ((current_time.tv_sec*1e6 + current_time.tv_usec) - (last_time.tv_sec*1e6 + last_time.tv_usec) >= (1.0/MSG_OUT_TX_FREQUENCY)*1e6){
            delta_time[delta_time_count] = (current_time.tv_sec*1e6 + current_time.tv_usec) - (last_time.tv_sec*1e6 + last_time.tv_usec);
            gettimeofday(&last_time, NULL);
            delta_time_count++; 
            if(delta_time_count >= MESSAGE_ON_TX_FREQUENCY_CALCULATION){ 
              delta_time_count = 0; 
              avg_msg_frequency_tx = 0; 
              for (int j=0; j<MESSAGE_ON_TX_FREQUENCY_CALCULATION; j++){
                avg_msg_frequency_tx +=  delta_time[j];
              }
              avg_msg_frequency_tx = MESSAGE_ON_TX_FREQUENCY_CALCULATION/(avg_msg_frequency_tx*1e-6);
              if(verbose){
                printf("Valid SHIP_INFO_MSG_ID message received from Pixhawk 4 with ID %d: \n",parser.sender_id);
                printf("Ship roll angle [deg] : %f \n",paylod_ship.phi);
                printf("Ship theta angle [deg] : %f \n",paylod_ship.theta);
                printf("Ship psi angle [deg] : %f \n",paylod_ship.psi);
                printf("Ship roll rate [deg/s] : %f \n",paylod_ship.phi_dot);
                printf("Ship pitch rate [deg/s] : %f \n",paylod_ship.theta_dot);
                printf("Ship yaw rate [deg/s] : %f \n",paylod_ship.psi_dot);  
                printf("Ship pos x [m] : %f \n",paylod_ship.x);  
                printf("Ship pos y [m] : %f \n",paylod_ship.y);  
                printf("Ship pos z [m] : %f \n",paylod_ship.z);  
                printf("Ship speed x [m/s] : %f \n",paylod_ship.x_dot);  
                printf("Ship speed y [m/s] : %f \n",paylod_ship.y_dot);  
                printf("Ship speed z [m/s] : %f \n",paylod_ship.z_dot);  
                printf("Ship acc x [m/s^2] : %f \n",paylod_ship.x_ddot);  
                printf("Ship acc y [m/s^2] : %f \n",paylod_ship.y_ddot);  
                printf("Ship acc z [m/s^2] : %f \n",paylod_ship.z_ddot);     
                printf("Average frequency ship message output : %.1f \n",avg_msg_frequency_tx);             
              }
            }
            ivy_send_ship_info_msg();
          }
        }
      }
    }
    else{
      usleep(20);
    }
}

void ivy_send_ship_info_msg(void){
  // if(verbose) printf("Sent received Ship message on ivy bus\n");
  IvySendMsg("ground SHIP_INFO_MSG %d %f %f %f  %f %f %f  %f %f %f  %f %f %f  %f %f %f",
          // SHIP_INFO_MSG_ID,
          ac_id,
          
          paylod_ship.phi,
          paylod_ship.theta,
          paylod_ship.psi,

          paylod_ship.phi_dot,
          paylod_ship.theta_dot,
          paylod_ship.psi_dot,

          paylod_ship.x,
          paylod_ship.y,
          paylod_ship.z,

          paylod_ship.x_dot,
          paylod_ship.y_dot,
          paylod_ship.z_dot,

          paylod_ship.x_ddot,
          paylod_ship.y_ddot,
          paylod_ship.z_ddot);
}

int main(int argc, char** argv) {

  /* Defaults */
  #ifdef __APPLE__
    char* ivy_bus = "224.255.255.255";
  #else
    char *ivy_bus = "127.255.255.255";
  #endif

  GMainLoop *ml =  g_main_loop_new(NULL, FALSE);

  IvyInit ("SHIPINFO2Ivy", "SHIPINFO2Ivy READY", NULL, NULL, NULL, NULL);
  IvyStart(ivy_bus);

  /* Arguments options and usage information */
  static struct option long_options[] = {
    {"ac_id", required_argument, NULL, 'i'},
    {"help", no_argument, NULL, 'h'},
    {"verbose", no_argument, NULL, 'v'},
    {0, 0, 0, 0}
  };

  static const char* usage =
    "Usage: %s [options]\n"
    " Options :\n"
    "   -i --ac_id [aircraft_id]               Aircraft id\n"
    "   -h --help                              Display this help\n"
    "   -v --verbose                           Print verbose information\n";

  int c;
  int option_index = 0;

  while((c = getopt_long(argc, argv, "i:h:v", long_options, &option_index)) != -1) {
    switch (c) {
      case 'i':
        ac_id = atoi(optarg);
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

  Pixhawk_reading_init();
  g_main_loop_run(ml);

}
