#ifndef CATIA_LWIR_CAM_PIPE_H
#define CATIA_LWIR_CAM_PIPE_H

#include <stddef.h>

int lwir_cam_pipe_init(const char *unused);
int lwir_cam_pipe_shoot(char *filename, size_t filename_size, int image_number);
void lwir_cam_pipe_deinit(void);

#endif