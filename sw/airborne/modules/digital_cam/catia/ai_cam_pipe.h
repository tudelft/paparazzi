#ifndef CATIA_AI_CAM_PIPE_H
#define CATIA_AI_CAM_PIPE_H

#include <stddef.h>

int ai_cam_pipe_init(const char *unused);
int ai_cam_pipe_shoot(char *filename, size_t filename_size, int image_number);
void ai_cam_pipe_deinit(void);

#endif