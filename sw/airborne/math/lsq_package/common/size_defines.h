

#ifndef SIZE_DEFINES_H
#define SIZE_DEFINES_H

#ifndef TESTING_LSQ_PACKAGE
  #include "generated/airframe.h"
#endif

//#define CA_N_U  20 // todo: runtimes errors if this is exceeded
//#define CA_N_V  6
#define CA_N_C  (CA_N_U+CA_N_V)

#ifndef TESTING_LSQ_PACKAGE
	#define SINGLE
#else
	#define SINGLE
	//#define DOUBLE
#endif

#ifdef DOUBLE
typedef double num_t;
#define TOL 1e-7
#else
typedef float num_t;
#define TOL 1e-4
#endif

#endif