#ifndef DIVERGENCE_STOP_H
#define DIVERGENCE_STOP_H

// settings
extern float oag_color_count_frac;  // obstacle detection threshold as a fraction of total of image
extern float oag_floor_count_frac;  // floor detection threshold as a fraction of total of image
extern float oag_max_speed;         // max flight speed [m/s]
extern float oag_heading_rate;      // heading rate setpoint [rad/s]

extern void divergence_stop_init(void);
extern void divergence_stop_periodic(void);

#endif
