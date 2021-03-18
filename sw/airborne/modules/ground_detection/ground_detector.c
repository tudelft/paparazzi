//
// Created by anish on 14-03-21.
//

//TODO: Fix the shitty ass naming of all these files

#include "ground_detector.h"
#include "modules/computer_vision/ground_detection_testing.h"
#include "firmwares/rotorcraft/navigation.h"
#include "generated/airframe.h"
#include "state.h"
#include "subsystems/abi.h"
#include <time.h>
#include <stdio.h>

#define NAV_C // needed to get the nav functions like Inside...
#include "generated/flight_plan.h"

#define GROUND_DETECTOR_VERBOSE TRUE

#define PRINT(string,...) fprintf(stderr, "[ground_detector->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if GROUND_DETECTOR_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

volatile int go_no_go;

void get_signal_init(void){
    srand(time(NULL));

}

void get_signal_periodic(void){
    //TODO: Add (periodic) navigation logic
    VERBOSE_PRINT("%d \n", go_no_go); /*This prints the go_no_go variable in paparazzi. This variable is 1 if the drone
                                        can fly straight ahead and 0 otherwise*/
}
