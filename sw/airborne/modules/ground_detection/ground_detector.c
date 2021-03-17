//
// Created by anish on 14-03-21.
//

#include "ground_detector.h"
#include "modules/computer_vision/cv_ground_detection.h"
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

extern void cv_ground_detection_init(void);

void get_signal_init(void){
    srand(time(NULL));
}

void get_signal_periodic(void){
    cv_ground_detection_init();
}
