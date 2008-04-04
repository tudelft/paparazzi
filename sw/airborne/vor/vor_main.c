#include "std.h"
#include "init_hw.h"
#include "sys_time.h"
#include "led.h"
#include "interrupt_hw.h"
#include "uart.h"

#include "messages.h"
#include "downlink.h"

//#include "vor_demod.h"

#include "vor_convertions.h"
#include "vor_integer_filters.h"

static inline void main_init( void );
static inline void main_periodic_task( void );

static uint32_t t0, t1;

int main( void ) {
  main_init();
  while(1) {
    if (vor_adc_sample_available) {
      LED_OFF(1);

      vor_adc_sample_available = FALSE;
      int32_t y0_var = filter_bp_var(vor_adc_sample);
      y0_var = y0_var >> 16;
      int32_t y0_ref =  filter_bp_ref(vor_adc_sample);
      y0_ref = y0_ref >> 16;
      int32_t y0_decim =  filter_bp_ref(vor_adc_sample);
      y0_decim = y0_decim << 16;

      VorDacSet(vor_adc_sample);
      LED_ON(1);
    }

  }
  return 0;
}

static inline void main_init( void ) {
  hw_init();
  sys_time_init();
  led_init();
  //  uart0_init_tx();
  //  vor_demod_init();
  VorDacInit();
  vor_adc_init();
  int_enable();
}




