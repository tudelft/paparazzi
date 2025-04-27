/**
 * @file modules/orange_avoider/orange_avoider.h
 * @author Roland Meertens
 *
 * @brief Public interface for the Orange Avoider module.
 *
 * This header file declares the functions and variables that other `.c` files can use for the Orange Avoider 
 * obstacle avoidance module.
 *
 * Copyright (C) Roland Meertens
 * This module is part of Paparazzi UAV.
 */

 #ifndef ORANGE_AVOIDER_H
 #define ORANGE_AVOIDER_H
 
 // Public variables
 extern float oa_color_count_frac;
 
 // Public functions
 extern void orange_avoider_init(void);
 extern void orange_avoider_periodic(void);
 
 #endif