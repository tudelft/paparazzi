/*
 * This file is part of the Paparazzi UAV project, modified for the course AE4317 Autonomous Flight of 
 * Micro Air Vehicles at the TU Delft. It's tailored to enhance UAV autonomy through advanced obstacle 
 * avoidance techniques, utilizing color-based detection strategies within a guided flight mode.
 *
 * This module extends the functionality of traditional avoidance systems by incorporating dynamic 
 * color detection capabilities. It leverages the cv_detect_color_object module to identify and 
 * react to specific color thresholds (e.g., orange for obstacles, green for the floor) within the 
 * operational environment. This allows the UAV to navigate more effectively in structured 
 * environments like the cyberzoo, avoiding obstacles and the perimeter by detecting changes in 
 * color patterns on the ground.
 *
 * The configuration is highly adaptable, running multiple filters to detect various colors 
 * simultaneously, making it suitable for a wide range of applications beyond the basic obstacle 
 * avoidance scenario. This approach not only improves the UAV's spatial awareness but also 
 * enhances its ability to interact with its surroundings in a meaningful way.
 */

#ifndef GROUP_11_H
#define GROUP_11_H

// Settings
extern float oag_color_count_frac;       // Obstacle detection threshold as a fraction of total image area
extern float oag_floor_count_frac;       // Floor detection threshold as a fraction of total image area
extern float oag_max_speed;              // Maximum flight speed in meters per second (m/s)
extern float oag_heading_rate;           // Heading change rate in radians per second (rad/s)
extern float oag_central_floor_frac;     // Threshold for detecting the central floor area as a fraction of the defined area
extern float oag_plant_frac;             // Plant detection threshold as a fraction of the area designated for plant detection

extern void group_11_init(void);         // Initializes the module
extern void group_11_periodic(void);     // Periodic update function for the module

#endif /* GROUP_11_H */