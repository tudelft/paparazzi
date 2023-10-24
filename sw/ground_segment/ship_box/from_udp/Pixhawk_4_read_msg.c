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


#define UDP_BUFFER_SIZE   2024

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

/* Endpoints */
struct endpoint_udp_t {
  char server_addr[128];
  uint16_t server_port;
  char client_addr[128];
  uint16_t client_port;

  int fd;
  pthread_t thread;
};

struct endpoint_uart_t {
  char devname[512];
  int baudrate;

  int fd;
  pthread_t thread;
};

enum endpoint_type_t {
  ENDPOINT_TYPE_NONE,
  ENDPOINT_TYPE_UDP,
  ENDPOINT_TYPE_UART
};

struct endpoint_t {
  enum endpoint_type_t type;
  union {
    struct endpoint_udp_t udp;
    struct endpoint_uart_t uart;
  } ep;
};

struct payload_ship_info_msg_ground {
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

static bool verbose = false;
static struct endpoint_t ship_box_ep;
static uint8_t ac_id = 0;
struct normal_parser_t parser;

char outBuffer[256];    //buffer to hold outgoing data
uint8_t out_idx = 0;

struct payload_ship_info_msg_ground paylod_ship; 

void packet_handler(void *ep, uint8_t *data, uint16_t len);

void ivy_send_ship_info_msg(void);

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

/**
 * Create an UDP endpoint thread
 */
void *udp_endpoint(void *arg) {
  struct endpoint_udp_t *ep = (struct endpoint_udp_t *)arg;

  /* Create the socket */
	ep->fd = socket(AF_INET, SOCK_DGRAM, 0);
	if(ep->fd < 0) {
		fprintf(stderr, "Could not create socket for %s:%d\r\n", ep->server_addr, ep->server_port);
    return NULL;
  }

  /* Enable broadcasting */
  int one = 1;
  setsockopt(ep->fd, SOL_SOCKET, SO_BROADCAST, &one, sizeof(one));

  /* Create the input address */
	struct sockaddr_in server;
	server.sin_family = AF_INET;
	server.sin_addr.s_addr = inet_addr(ep->server_addr);
	server.sin_port = htons(ep->server_port);

  /* Bind the socket with the server address  */
  if(bind(ep->fd, (struct sockaddr *)&server, sizeof(server)) < 0) {
    fprintf(stderr, "Could not bind to address %s:%d\r\n", ep->server_addr, ep->server_port);
    return NULL;
  }

  /* Wait for messages */
  while(true) {
    struct sockaddr_in client;
    uint8_t buffer[UDP_BUFFER_SIZE];
    socklen_t len = sizeof(client);
    int n = recvfrom(ep->fd, buffer, UDP_BUFFER_SIZE, MSG_WAITALL, (struct sockaddr *)&client, &len);

    // Ignore errors
    if(n < 0)
      continue;

    if(verbose) printf("Got packet at endpoint [%s:%d] with length %d\r\n", ep->server_addr, ep->server_port, n);

    // Send the message to the handler
    packet_handler(ep, buffer, n);
  }
}

/**
 * Send a message out of an UDP endpoint
 */
void udp_endpoint_send(struct endpoint_udp_t *ep, uint8_t *buffer, uint16_t len) {
  // Check if file descriptor is valid
  if(ep->fd < 0)
    return;

  struct sockaddr_in client;
	client.sin_family = AF_INET;
	client.sin_addr.s_addr = inet_addr(ep->client_addr);
	client.sin_port = htons(ep->client_port);

  if(verbose) printf("Send packet at endpoint [%s:%d] with length %d\r\n", ep->client_addr, ep->client_port, len);
  sendto(ep->fd, buffer, len, MSG_DONTWAIT, (struct sockaddr *)&client, sizeof(client));
}

/**
 * Create an UDP endpoint
 */
void udp_create(char *server_addr, uint16_t server_port, char *client_addr, uint16_t client_port) {
  /* Create the endpoint */
  ship_box_ep.type = ENDPOINT_TYPE_UDP;
  struct endpoint_udp_t *ep = &ship_box_ep.ep.udp;
  strncpy(ep->server_addr, server_addr, 127);
  ep->server_port = server_port;
  strncpy(ep->client_addr, client_addr, 127);
  ep->client_port = client_port;

  /* Start the endpoint thread */
  if(verbose) printf("Created UDP endpoint with server [%s:%d] and client [%s:%d]\r\n", ep->server_addr, ep->server_port, ep->client_addr, ep->client_port);
  pthread_create(&ep->thread, NULL, udp_endpoint, ep);
}

/**
 * Get the baudrate based ont he argument
 */
int get_baud(unsigned int baud_rate)
{
  int baud = 0;
  switch (baud_rate)
  {
      #ifdef B921600
    case 921600:
      baud = B921600;
      break;
      #endif
      #ifdef B460800
    case 460800:
      baud = B460800;
      break;
      #endif
    case 230400:
      baud = B230400;
      break;
    case 115200:
      baud = B115200;
      break;
    case 57600:
      baud = B57600;
      break;
    case 38400:
      baud = B38400;
      break;
    case 19200:
      baud  = B19200;
      break;
    case 9600:
      baud  = B9600;
      break;
    case 4800:
      baud  = B4800;
      break;
    case 2400:
      baud  = B2400;
      break;
    case 1800:
      baud  = B1800;
      break;
    case 1200:
      baud  = B1200;
      break;
    case 600:
      baud  = B600;
      break;
    case 300:
      baud  = B300;
      break;
    case 200:
      baud  = B200;
      break;
    case 150:
      baud  = B150;
      break;
    case 134:
      baud  = B134;
      break;
    case 110:
      baud  = B110;
      break;
    case 75:
      baud  = B75;
      break;
    case 50:
      baud  = B50;
      break;
    default:
      baud = -1;
  }  //end of switch baud_rate
  return baud;
}

/**
 * Create an UART endpoint thread
 */
void *uart_endpoint(void *arg) {
  struct endpoint_uart_t *ep = (struct endpoint_uart_t *)arg;

  /* Open de uart device in blocking Read/Write mode */
  ep->fd = open(ep->devname, O_RDWR | O_NOCTTY);
  if(ep->fd < 0) {
    fprintf(stderr, "Could not open uart for %s:%d\r\n", ep->devname, ep->baudrate);
    return NULL;
  }

  /* Conver the baudrate */
  int baud = get_baud(ep->baudrate);
  if(baud < 0) {
    fprintf(stderr, "Could set baudrate for %s:%d\r\n", ep->devname, ep->baudrate);
    return NULL;
  }

  /* Configure the UART */
  struct termios options;
	tcgetattr(ep->fd, &options);
	options.c_cflag = baud | CS8 | CLOCAL | CREAD;
	options.c_iflag = IGNPAR;
	options.c_oflag = 0;
	options.c_lflag = 0;
	tcflush(ep->fd, TCIFLUSH);
	tcsetattr(ep->fd, TCSANOW, &options);

  /* Try to read bytes */
  while(true) {
    uint8_t buffer[2048];
    int len = read(ep->fd, buffer, 2048);

    // Read got an error
    if(len < 0) {
      if(verbose) printf("Got read error (%d) at endpoint [%s:%d]\r\n", len, ep->devname, ep->baudrate);
      usleep(20);
    }
    // No bytes to read
    else if(len == 0) {
      usleep(20);
    }
    // We succsfully received some bytes
    else if(len > 0) {
      if(verbose) printf("Got packet at endpoint [%s:%d] with length %d\r\n", ep->devname, ep->baudrate, len);

      // Send the message to the handler
      packet_handler(ep, buffer, len);
    }
  }
}

/**
 * Send a message out of an UART endpoint
 */
void uart_endpoint_send(struct endpoint_uart_t *ep, uint8_t *buffer, uint16_t len) {
  // Check if file descriptor is valid
  if(ep->fd < 0)
    return;

  if(verbose) printf("Send packet at endpoint [%s:%d] with length %d\r\n", ep->devname, ep->baudrate, len);
  if(write(ep->fd, buffer, len) < 0)
    fprintf(stderr, "Send packet at endpoint [%s:%d] with length %d errored\r\n", ep->devname, ep->baudrate, len);
}

/**
 * Create an UART endpoint
 */
void uart_create(char *devname, uint32_t baudrate) {
  /* Create the endpoint */
  ship_box_ep.type = ENDPOINT_TYPE_UART;
  struct endpoint_uart_t *ep = &ship_box_ep.ep.uart;
  strncpy(ep->devname, devname, 511);
  ep->baudrate = baudrate;

  /* Start the endpoint thread */
  if(verbose) printf("Created UART endpoint [%s:%d]\r\n", ep->devname, ep->baudrate);
  pthread_create(&ep->thread, NULL, uart_endpoint, ep);
}

/**
 * Handle incoming packets from the SHIP BOX (parsing SHIP_BOX_MSG)
 */
void packet_handler(void *ep, uint8_t *data, uint16_t len) {

  for (int i=0; i<len; i++){
    if (parse_single_byte(data[i])) { 
      // if complete message detected save the message on the paylod_ship struct
      if(parser.msg_id == SHIP_INFO_MSG_GROUND_ID){

        memcpy(&paylod_ship,&parser.payload[2],sizeof(struct payload_ship_info_msg_ground));

        gettimeofday(&current_time, NULL);
        if ((current_time.tv_sec*1e6 + current_time.tv_usec) - (last_time.tv_sec*1e6 + last_time.tv_usec) >= (1.0/MSG_OUT_TX_FREQUENCY)*1e6){
          delta_time[delta_time_count] = (current_time.tv_sec*1e6 + current_time.tv_usec) - (last_time.tv_sec*1e6 + last_time.tv_usec);
          gettimeofday(&last_time, NULL);
          delta_time_count++; 
          if(delta_time_count > MESSAGE_ON_TX_FREQUENCY_CALCULATION){ 
            delta_time_count = 0; 
            avg_msg_frequency_tx = 0; 
            for (int j=0; j<MESSAGE_ON_TX_FREQUENCY_CALCULATION; j++){
              avg_msg_frequency_tx +=  delta_time[j];
            }
            avg_msg_frequency_tx = MESSAGE_ON_TX_FREQUENCY_CALCULATION/(avg_msg_frequency_tx*1e-6);
          }
          ivy_send_ship_info_msg();
          if(verbose){
            printf("Valid SHIP_INFO_MSG_GROUND message received from Ship box with ID %d: \n",parser.sender_id);
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
      }
    }
  }

}

void ivy_send_ship_info_msg(void){
  // if(verbose) printf("Sent received Ship message on ivy bus\n");
  IvySendMsg("ground SHIP_INFO_MSG %d %f %f %f  %f %f %f  %f %f %f  %f %f %f  %f %f %f",
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
    {"endpoint", required_argument, NULL, 'e'},
    {"help", no_argument, NULL, 'h'},
    {"verbose", no_argument, NULL, 'v'},
    {0, 0, 0, 0}
  };
  static const char* usage =
    "Usage: %s [options]\n"
    " Options :\n"
    "   -i --ac_id [aircraft_id]               Aircraft id\n"
    "   -e --endpoint [endpoint_str]           Endpoint address of the GPS\n"
    "   -h --help                              Display this help\n"
    "   -v --verbose                           Print verbose information\n";

  int c;
  int option_index = 0;
  while((c = getopt_long(argc, argv, "i:e:h:v", long_options, &option_index)) != -1) {
    switch (c) {
      case 'i':
        ac_id = atoi(optarg);
        break;

      case 'e':
        // Parse the endpoint argument UDP
        if(!strncmp(optarg, "udp", 3)) {
          char serv_addr[125], cli_addr[125];
          uint16_t serv_port, cli_port;
          if(sscanf(optarg, "udp://%[^:]:%hu:%[^:]:%hu", serv_addr, &serv_port, cli_addr, &cli_port) != 4) {
            fprintf(stderr, "UDP endpoint %s has incorrect arguments\r\n", optarg);
            return 2;
          }
          udp_create(serv_addr, serv_port, cli_addr, cli_port);
        }
        // Parse the endpoint argument UART
        else if(!strncmp(optarg, "uart", 4)) {
          char devname[256];
          uint32_t baudrate;
          if(sscanf(optarg, "uart://%[^:]:%u", devname, &baudrate) != 2) {
            fprintf(stderr, "UART endpoint %s has incorrect arguments\r\n", optarg);
            return 2;
          }
          uart_create(devname, baudrate);
        }
        else {
          fprintf(stderr, "Endpoint %s has incorrect type only uart and udp are supported\r\n", optarg);
          return 2;
        }
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

  g_main_loop_run(ml);

  /*while(true) {
    usleep(50000);
  }*/

  return 0;
}