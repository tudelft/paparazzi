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

// Module settings (found inside conf/modules/orange_avoider.xml)
// Those are sliders inside the Paparazzi Ground Control Station (GCS)
extern float oa_color_count_frac;
extern float heading_increment_setting;
extern float speed_multiplier;

// functions
extern void orange_avoider_init(void);
extern void orange_avoider_periodic(void);

#endif

