/**
 * @file serial.h
 * @brief Legacy UART descriptor setup and byte-oriented compatibility macros.
 * @details New CATIA code should use serial_tx.h for output because it preserves
 * frame boundaries under concurrent worker activity. The macros remain for platform
 * compatibility with the historic transport interface.
 */


extern int fd;

#include "std.h"

/** @brief Configure the shared non-blocking CATIA UART descriptor.
 * @param port_name UART device path.
 * @return 0 on success or a negative setup error. */
int serial_init(const char *port_name);

static inline int ttyUSB0ChAvailable(void)
{
  return false;
}

#define ttyUSB0Transmit(_char)     \
  {                                  \
    char c = _char;                  \
    int __attribute__((unused)) ret = write(fd,&c,1);        \
  }

#define ttyUSB0Getch() ({char c;int ret=read(fd, &c,1);c;})
