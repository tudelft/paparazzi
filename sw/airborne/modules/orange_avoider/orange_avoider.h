/*
 * Copyright (C) Roland Meertens
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/orange_avoider/orange_avoider.h"
 * @author Roland Meertens
 * Example on how to use the colours detected to avoid orange pole in the cyberzoo
 */

#ifndef ORANGE_AVOIDER_H
#define ORANGE_AVOIDER_H

// settings
extern int green_perc_threshold;
extern int green_sides_threshold;

// functions
extern void green_detector_init(void);
extern void green_detector_periodic(void);

#endif

