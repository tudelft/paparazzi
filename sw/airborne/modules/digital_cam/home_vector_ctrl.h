#ifndef DIGITAL_CAM_HOME_VECTOR_H
#define DIGITAL_CAM_HOME_VECTOR_H

#include "std.h"

extern void home_vector_init(void);
extern void home_vector_periodic(void);

/** Latest body-frame prediction received from MORA. */
extern bool home_vector_result_valid;
/** True from the reply to the latest shot until home_vector_periodic() consumes it into
 *  a waypoint nudge. */
extern bool home_vector_result_fresh;
/** Body-frame direction-to-home, forward component (unit vector, dimensionless). */
extern float home_vector_dx_body;
/** Body-frame direction-to-home, right component (unit vector, dimensionless). */
extern float home_vector_dy_body;
/** Predicted distance to home, metres (only as informative as the deployed checkpoint's
 *  own distance-normalization; not currently used to size the waypoint nudge). */
extern float home_vector_dist_m;

/** Shoot period while active, seconds; GCS adjustable. */
extern float home_vector_period_s;
/** Distance the HOME waypoint is placed ahead of the aircraft along the predicted
 *  world-frame heading on every fresh result; GCS adjustable. */
extern float home_vector_leg_distance_m;

/** Waypoint nudged by every fresh result once home_vector_start() has been called. */
extern uint8_t home_vector_wp_id;

/** Clear the last result and start periodic --aicam-home shoots, nudging wp_id. */
extern uint8_t home_vector_start(uint8_t wp_id);
/** Stop periodic shoots. The HOME waypoint is left at its last nudged position. */
extern uint8_t home_vector_stop(void);
/** Forget the last result without stopping sampling. */
extern uint8_t home_vector_result_clear(void);

#endif // DIGITAL_CAM_HOME_VECTOR_H
