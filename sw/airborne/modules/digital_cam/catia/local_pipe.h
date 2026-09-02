#ifndef CATIA_LOCAL_PIPE_H
#define CATIA_LOCAL_PIPE_H

#include <stddef.h>

int local_pipe_init(const char *mock_image);
int local_pipe_test_init(const char *mock_image);
int local_pipe_shoot(char *filename, size_t filename_size, int image_number);
void local_pipe_deinit(void);

#endif