/*
 * Copyright (C) Roland Meertens
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/green_attractor/green_attractor.h"
 * @author Group2
 * Use the colours detected to avoid obstacles in the cyberzoo
 */

#ifndef GREEN_ATTRACTOR_H
#define GREEN_ATTRACTOR_H

#include <stdint.h>

// settings
extern float ga_color_count_frac;
extern float meander_frac;
extern uint8_t meander_increment;
extern float d_covered;
extern float FPS_green_attractor;
extern int32_t color_count;
extern uint8_t confidence_increment;
extern uint8_t confidence_decrement;
extern uint8_t current_state;
extern float strong_turn_threshold;
extern uint8_t SFSH_increment;

// functions
extern void green_attractor_init(void);
extern void green_attractor_periodic(void);
void log_distance_covered_periodic(void);

#endif

