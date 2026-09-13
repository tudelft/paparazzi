#ifndef STD_H
#define STD_H

/**
 * @file std.h
 * @brief Compatibility transmit primitive for legacy CATIA framing macros.
 * @details Modern concurrent CATIA paths use serial_tx.h. This primitive remains for
 * macros shared with the historical protocol layer and bounds backpressure to one
 * second so a disconnected peer cannot deadlock the process.
 */

#include <errno.h>
#include <poll.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#define TRUE (1==1)
#define FALSE (1==0)

extern int fd;

/** @brief Write one legacy transport byte, retrying only transient interruption/backpressure.
 * @param value Byte to transmit.
 * @details POLLOUT is used after EAGAIN instead of spinning. Errors are reported but
 * not propagated because the historic macro API has no return channel. */
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
