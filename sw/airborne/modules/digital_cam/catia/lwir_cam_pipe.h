#ifndef CATIA_LWIR_CAM_PIPE_H
#define CATIA_LWIR_CAM_PIPE_H

#include <stddef.h>
#include "capture_timing.h"

int lwir_cam_pipe_init(const char *unused);
int lwir_cam_pipe_warmup(void);
void lwir_cam_pipe_set_native_raw(int enabled);
int lwir_cam_pipe_set_calibration(const char *path);
int lwir_cam_pipe_process_mock(char *filename);
int lwir_cam_pipe_geolocate(char *filename);
double lwir_cam_pipe_capture_delay(void);
struct capture_timing lwir_cam_pipe_capture_timing(void);
int lwir_cam_pipe_shoot(char *filename, size_t filename_size, int image_number);
void lwir_cam_pipe_deinit(void);

#endif