#ifndef RASPBERRYPI_H
#define RASPBERRYPI_H

#define START_BYTE 0x9B  //1st start block identifier byte


#include "std.h"
#include <stdbool.h>
#include <stdlib.h>
#include "generated/airframe.h"
#include "pprzlink/pprz_transport.h"

struct __attribute__((__packed__)) raspberrypi_data_in {
    //Motor command
	int16_t pos_x_int;
	int16_t pos_y_int;
  int16_t pos_z_int;
  //Rolling_msg
  int16_t rolling_msg_in;
  int16_t rolling_msg_in_id;
  int16_t yaprak;
  uint8_t checksum_in;
};

struct __attribute__((__packed__)) raspberrypi_data_out {
    //Actuator state - unfiltered
    int16_t iteration;
    int16_t check_data;
    uint8_t checksum_out;
};

extern void raspberrypi_init(void);
// extern void raspberrypi_periodic(void);
extern void raspberrypi_event(void);

#endif