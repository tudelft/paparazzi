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

#ifndef GROUP11_OPTICFLOW_H
#define GROUP11_OPTICFLOW_H

// settings
extern float oa_color_count_frac;

// functions
extern void group11_opticflow_init(void);
extern void group11_opticflow_periodic(void);

#endif
