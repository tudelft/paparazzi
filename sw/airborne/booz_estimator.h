#ifndef BOOZ_ESTIMATOR_H
#define BOOZ_ESTIMATOR_H

#include "6dof.h"

/* unfiltered rates available when filter crashed or uninitialed */
extern float booz_estimator_uf_p;
extern float booz_estimator_uf_q;
extern float booz_estimator_uf_r;

extern float booz_estimator_p;
extern float booz_estimator_q;
extern float booz_estimator_r;

extern float booz_estimator_phi;
extern float booz_estimator_theta;
extern float booz_estimator_psi;

#ifndef DISABLE_NAV

extern float booz_estimator_dcm[AXIS_NB][AXIS_NB];

/* position in earth frame : not yet available - sim only */
extern float booz_estimator_x;
extern float booz_estimator_y;
extern float booz_estimator_z;

/* speed in body frame : not yet available - sim only */
extern float booz_estimator_u;
extern float booz_estimator_v;
extern float booz_estimator_w;
#endif /* DISABLE_NAV */


extern void booz_estimator_init( void );
extern void booz_estimator_read_inter_mcu_state( void );


#ifndef DISABLE_NAV
extern void booz_estimator_compute_dcm( void );
extern void booz_estimator_set_speed_and_pos(float _u, float _v, float _w, float _x, float _y, float _z);
extern void booz_estimator_set_psi( float _psi);
#endif

#endif /* BOOZ_ESTIMATOR_H */
