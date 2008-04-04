/* Definitions and declarations required to compile autopilot code on a
   i386 architecture. Bindings for OCaml. */

#include <stdio.h>
#include <assert.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <time.h>
#include <string.h>
#include "std.h"
#include "inter_mcu.h"
#include "autopilot.h"
#include "estimator.h"
#include "gps.h"
#include "traffic_info.h"
#include "flight_plan.h"
#include "settings.h"
#include "nav.h"
#include "fw_h_ctl.h"
#include "fw_v_ctl.h"
#include "infrared.h"
#include "cam.h"
#include "commands.h"
#include "main_ap.h"
#include "ap_downlink.h"
#include "sim_uart.h"
#include "latlong.h"
#include "datalink.h"

#include <caml/mlvalues.h>
#include <caml/memory.h>


/* Dummy definitions to replace the ones from the files not compiled in the
   simulator */
uint8_t ir_estim_mode;
uint8_t vertical_mode;
uint8_t inflight_calib_mode;
bool_t rc_event_1, rc_event_2;
bool_t launch;
uint8_t gps_nb_ovrn, modem_nb_ovrn, link_fbw_fbw_nb_err, link_fbw_nb_err;
float alt_roll_pgain;
float roll_rate_pgain;
bool_t gpio1_status;
uint16_t datalink_time = 0;



uint8_t ac_id;

value sim_periodic_task(value unit) {
  periodic_task_ap();
  periodic_task_fbw();
  event_task_ap();
  event_task_fbw();
  return unit;
}


float ftimeofday(void) {
  struct timeval t;
  struct timezone z;
  gettimeofday(&t, &z);
  return (t.tv_sec + t.tv_usec/1e6);
}

value sim_init(value unit) {
  init_fbw();
  init_ap();
#ifdef SIM_UART
  /* open named pipe */
  char link_pipe_name[128];
#ifdef SIM_XBEE
  sprintf(link_pipe_name, "/tmp/pprz_xbee");
#else
  sprintf(link_pipe_name, "/tmp/pprz_link_%d", AC_ID);
#endif
  struct stat st;
  if (stat(link_pipe_name, &st)) {
    if (mkfifo(link_pipe_name, 0644) == -1) {
      perror("make pipe");
      exit (10);
    }
  }	
  if ( !(pipe_stream = fopen(link_pipe_name, "w")) ) {
    perror("open pipe");
    exit (10);
  }
#endif

  return unit;
}

value update_bat(value bat) {
  fbw_vsupply_decivolt = Int_val(bat);  
  return Val_unit;
}


value get_commands(value val_commands) {
  int i;

  for(i=0; i < COMMANDS_NB; i++)
    Store_field(val_commands, i, Val_int(commands[i]));

  return Val_int(commands[COMMAND_THROTTLE]);
}

value set_ac_info_native(value ac_id __attribute__ ((unused)), value ux __attribute__ ((unused)), value uy __attribute__ ((unused)), value course __attribute__ ((unused)), value alt __attribute__ ((unused)), value gspeed __attribute__ ((unused)), value itow __attribute__ ((unused))) {
#ifdef TRAFFIC_INFO
  SetAcInfo(Int_val(ac_id), Double_val(ux), Double_val(uy), 
	    Double_val(course), Double_val(alt), Double_val(gspeed), Int32_val(itow));
#endif
  return Val_unit;
}

value set_ac_info(value * argv, int argn) {
  assert (argn == 7);
  return set_ac_info_native(argv[0], argv[1], argv[2], argv[3],argv[4], argv[5], argv[6]);
}

value set_datalink_message(value s) {
  int n = string_length(s);
  char *ss = String_val(s);
  assert(n <= MSG_SIZE);

  int i;
  for(i = 0; i < n; i++) 
    dl_buffer[i] = ss[i];

  dl_parse_msg();
  return Val_unit;
}
