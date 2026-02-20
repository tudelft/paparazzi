/**
 * @file modules/orange_avoider/orange_avoider_guided.h
 * @author Kirk Scheper
 *
 * @brief Public interface for the Orange Avoider Guided module.
 *
 * This header file declares the functions and variables that other `.c` files can use for the Orange Avoider Guided 
 * obstacle avoidance module.
 *
 * Copyright (C) Kirk Scheper <kirkscheper@gmail.com>
 * This module is part of Paparazzi UAV.
 */

 #ifndef ORANGE_AVOIDER_GUIDED_H
 #define ORANGE_AVOIDER_GUIDED_H
 
 // Public variables
 extern float oag_color_count_frac;
 extern float oag_floor_count_frac;
 extern float oag_max_speed;
 extern float oag_heading_rate;
 
 // Public functions
 extern void orange_avoider_guided_init(void);
 extern void orange_avoider_guided_periodic(void);
 extern void orange_avoider_guided_retreat(void);
 
 #endif 

