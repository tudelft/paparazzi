/**
 * @file serial.h
 * @brief Legacy UART descriptor setup and byte-oriented compatibility macros.
 * @details New CATIA code should use serial_tx.h for output because it preserves
 * frame boundaries under concurrent worker activity. The macros remain for platform
 * compatibility with the historic transport interface.
 */


extern int fd;

#include "std.h"

/**
 * @brief Configure the shared non-blocking CATIA UART descriptor.
 * @param port_name UART device path.
 * @return 0 on success or a negative setup error.
 * @details The descriptor is set to raw 115200-bit/s 8-bit mode so line discipline cannot
 * translate binary CATIA framing bytes. This module owns descriptor setup only; concurrent
 * transmission must use @c serial_tx.h.
 */
int serial_init(const char *port_name);

/**
 * @brief Preserve the legacy "character available" transport API contract.
 * @return Always false because CATIA receives through the parser's nonblocking descriptor.
 * @details Retained solely for platform compatibility. New code must not use this predicate to
 * gate UART reads; it would permanently suppress them.
 */
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
