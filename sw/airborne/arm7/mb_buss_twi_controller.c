#include "mb_buss_twi_controller.h"

#include <string.h>

#include "i2c.h"

uint8_t mb_buss_twi_command;

uint8_t mb_buss_twi_nb_overun;
uint8_t mb_buss_twi_i2c_done;


#define MB_BUSS_TWI_CONTROLLER_MAX_CMD 255
/*
  Slave address 
  front = 0x52
  back  = 0x54
  right = 0x56
  left  = 0x58
*/
#define MB_BUSS_TWI_CONTROLLER_ADDR 0x52

void mb_buss_twi_controller_init(void) {
  mb_buss_twi_nb_overun = 0;
  mb_buss_twi_i2c_done = TRUE;
}

void mb_buss_twi_controller_set( float throttle ) {
  if (mb_buss_twi_i2c_done) {
    mb_buss_twi_command = throttle * MB_BUSS_TWI_CONTROLLER_MAX_CMD;
    
    const uint8_t msg[] = { MB_BUSS_TWI_CONTROLLER_ADDR, mb_buss_twi_command};
    memcpy((void*)i2c_buf, msg, sizeof(msg));
    i2c_transmit(MB_BUSS_TWI_CONTROLLER_ADDR, sizeof(msg), &mb_buss_twi_i2c_done);
  }
  else
    mb_buss_twi_nb_overun++;
}
