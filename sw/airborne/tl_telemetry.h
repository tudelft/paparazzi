#ifndef TL_TELEMETRY_H

#include "std.h"
#include "messages.h"
#include "periodic.h"
#include "uart.h"
#include "downlink.h"
#include "radio_control.h"
#include "commands.h"
#include "actuators.h"
#include "tl_bat.h"
#include "tl_estimator.h"



#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE

#define PERIODIC_SEND_RC() DOWNLINK_SEND_RC(PPM_NB_PULSES, rc_values)
#define PERIODIC_SEND_PPM() DOWNLINK_SEND_PPM(PPM_NB_PULSES, ppm_pulses)
#define PERIODIC_SEND_COMMANDS() DOWNLINK_SEND_COMMANDS(COMMANDS_NB, commands)
#define PERIODIC_SEND_ACTUATORS() DOWNLINK_SEND_ACTUATORS(SERVOS_NB, actuators)
#define PERIODIC_SEND_BAT() { int16_t dummy16=42; uint8_t dummy8=42; DOWNLINK_SEND_BAT(&commands[COMMAND_THROTTLE], &tl_bat_decivolt, &estimator_flight_time, &dummy8, &dummy16, &dummy16, &dummy16); }
#define PERIODIC_SEND_ESTIMATOR() DOWNLINK_SEND_ESTIMATOR(&estimator_z, &estimator_climb)



extern uint8_t telemetry_mode_Ap;

static inline void tl_telemetry_periodic_task(void) {
  PeriodicSendAp()
}

#define TL_TELEMETRY_H
#endif //TL_TELEMETRY_H
