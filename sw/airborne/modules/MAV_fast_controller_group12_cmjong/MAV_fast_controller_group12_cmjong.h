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

#ifndef MAV_FAST_CONTROLLER_GROUP12_CMJONG_H
#define MAV_FAST_CONTROLLER_GROUP12_CMJONG_H

// settings
extern float oa_color_count_frac;

// functions
extern void MAV_fast_controller_group12_cmjong_init(void);
extern void MAV_fast_controller_group12_cmjong_periodic(void);

#endif

