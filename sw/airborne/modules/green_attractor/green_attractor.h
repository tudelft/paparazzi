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

// #ifndef ORANGE_AVOIDER_H
// #define ORANGE_AVOIDER_H
#ifndef GREEN_ATTRACTOR_H
#define GREEN_ATTRACTOR_H

// settings
extern float oa_color_count_frac;
extern float meander_frac;
extern int meander_increment;
extern float d_covered;
extern float FPS_green_attractor;
extern int color_count;

// functions
extern void green_attractor_init(void);
extern void green_attractor_periodic(void);
void log_distance_covered_periodic(void);

#endif

