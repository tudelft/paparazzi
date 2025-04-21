/**
 * @file modules/orange_avoider/orange_avoider.h
 * @author Roland Meertens
 * @author Kevin Malkow (modifications)
 *
 * @brief  Header file for the orange obstacle avoider module.
 *
 * Defines the interface (public variables and functions) for the orange
 * obstacle avoider module, which relies on color detection to navigate.
 *
 * Copyright (C) Roland Meertens, Kevin Malkow (modifications)
 * This module is part of Paparazzi UAV.
 */

 #ifndef ORANGE_AVOIDER_H
 #define ORANGE_AVOIDER_H
  
 // Configurable variables
 /**
  * @brief Fraction of image pixels required to be orange to trigger obstacle detection.
  */
 extern float oa_color_count_frac;
 
 // Functions
 /**
  * @brief Initializes the orange avoider module.
  */
 extern void orange_avoider_init(void);
 
 /**
  * @brief Periodic function executing the orange obstacle avoidance logic.
  */
 extern void orange_avoider_periodic(void);
 
 #endif

