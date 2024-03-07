#include "modules/orange_avoider/orange_avoider.h"
#include "firmwares/rotorcraft/navigation.h"
#include "generated/airframe.h"
#include "state.h"
#include "modules/core/abi.h"
#include <time.h>
#include <stdio.h>

#define NAV_C // needed to get the nav functions like Inside...
#include "generated/flight_plan.h"

#define PRINT(string,...) fprintf(stderr, "[test_receiver->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)

// global vars

int32_t ground_count_left = 0;
int32_t ground_count_center = 0;
int32_t ground_count_right = 0;   


// for the communication with the GROUND_FILTER
#ifndef GROUND_FILTER_DETECTION_ID
#define GROUND_FILTER_DETECTION_ID ABI_BROADCAST
#endif
static abi_event ground_detection_ev;
static void ground_detection_cb(uint8_t __attribute__((unused)) sender_id,
                               int16_t __attribute__((unused)) received_count_left,
                               int16_t __attribute__((unused)) received_count_center,
                               int16_t __attribute__((unused)) received_count_right,
                               int16_t __attribute__((unused)) extra)
{
  
  ground_count_left = received_count_left;
  ground_count_center = received_count_center;
  ground_count_right = received_count_right;

  PRINT("Ground Detection Callback");
}

extern void test_receiver_init(void) {
  PRINT("test receiver init");

  // bind our colorfilter callbacks to receive the color filter outputs
  AbiBindMsgGROUND_FILTER_DETECTION(TEST_RECEIVER_GROUND_FILTER_DETECTION_ID, &ground_detection_ev, ground_detection_cb);

}
extern void test_receiver_periodic(void) {
  PRINT("test receiver periodic");


  PRINT("[countL:%d, countC:%d, countR:%d]", ground_count_left, ground_count_center, ground_count_right);
}