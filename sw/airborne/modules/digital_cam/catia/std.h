#ifndef STD_H
#define STD_H

#include <errno.h>
#include <poll.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#define TRUE (1==1)
#define FALSE (1==0)

extern int fd;

static inline void camera_link_transmit(uint8_t value)
{
  for (;;) {
    ssize_t result = write(fd, &value, 1);
    if (result == 1) {
      return;
    }
    if (result < 0 && errno == EINTR) {
      continue;
    }
    if (result < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      struct pollfd descriptor = {.fd = fd, .events = POLLOUT, .revents = 0};
      do {
        result = poll(&descriptor, 1, 1000);
      } while (result < 0 && errno == EINTR);
      if (result > 0 && (descriptor.revents & POLLOUT) != 0) {
        continue;
      }
      errno = result == 0 ? ETIMEDOUT : errno;
    }
    fprintf(stderr, "CATIA:\tserial write failed: %s\n", strerror(errno));
    return;
  }
}

#define CameraLinkTransmit(_x) { \
    camera_link_transmit((uint8_t)(_x)); \
  }

#endif
