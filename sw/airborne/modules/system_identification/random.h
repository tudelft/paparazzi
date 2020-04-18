#ifndef RANDOM_H
#define RANDOM_H

#include <std.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include "generated/airframe.h"
#include "mcu_periph/sys_time.h"

// Random number from uniform[0,1] distribution
double rand_uniform(void);

// Random number from gaussian(0, 1) distribution
double rand_gaussian(void);

#endif // RANDOM_H
