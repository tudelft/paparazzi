 
 #include "group_6_green.h"
 #include "modules/core/abi.h"
 #include "firmwares/rotorcraft/navigation.h"
 #include "state.h"
 #include "autopilot_static.h"
 #include <stdio.h>
 
 #define GREEN_DETECTOR

 // needed to receive output from a separate module running on a parallel process
 
 void green_detector_init(void) {
    fprintf(stderr, "Green detector initialized\n");
  }
 
 void green_detector_periodic(void) {
  
   // only evaluate our state machine if we are flying
   if (!autopilot_in_flight()) {
     return;
   }

 }
 