#ifndef OPTICFLOW_CALCULATOR_H
#define OPTICFLOW_CALCULATOR_H

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

#include "std.h"
#include "inter_thread_data.h"
#include "lib/vision/image.h"
#include "lib/v4l/v4l2.h"

struct opticflow_t {
  bool got_first_img;                 ///< If we got a image to work with
  bool just_switched_method;        ///< Boolean to check if methods has been switched (for reinitialization)
  struct image_t img_gray;              ///< Current gray image frame
  struct image_t prev_img_gray;         ///< Previous gray image frame
  uint16_t window_size;               ///< Window size for the blockmatching algorithm (general value for all methods)
  float pyr_scale;
  uint16_t levels;
  uint16_t poly_n;
  float poly_sigma;
  uint16_t flags;
  uint8_t max_iterations;               ///< The maximum amount of iterations the Lucas Kanade algorithm should do
};

extern void opticflow_calc_init(struct opticflow_t *opticflow);
extern bool opticflow_calc_module_call(struct opticflow_t *opticflow, struct image_t *img, float *divg);
extern bool calc_opticfarneback(struct opticflow_t *opticflow, struct image_t *img, float *divg);

#endif /* OPTICFLOW_CALCULATOR_H */