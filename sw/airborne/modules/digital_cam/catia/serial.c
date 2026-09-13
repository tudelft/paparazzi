/**
 * @file serial.c
 * @brief Configure CATIA's non-blocking flight-controller UART.
 * @details This module owns only the receive-side descriptor setup. Transmit
 * buffering and concurrent writers are handled by serial_tx.c. Raw 8-bit mode is
 * necessary because CATIA frames are binary and can contain bytes that terminal
 * line discipline would otherwise translate or consume.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <termios.h>
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */

#include <sys/types.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <string.h>
#include <sys/termios.h>
#include <sys/ioctl.h>

/****************************************************************************/
/* Open serial device for requested protocoll */
/****************************************************************************/

/** Shared UART descriptor used by legacy receive/transmit integration code. */
int fd;

int serial_init(const char *port_name)
{
  struct termios orig_termios, cur_termios;

  int br = B115200;

  fd = open(port_name, O_RDWR | O_NOCTTY | O_NONBLOCK);

  if (fd == -1) {
    fprintf(stderr, "CATIA:\tfailed to open serial device %s: %s\n", port_name, strerror(errno));
    return -1;
  }

  if (tcgetattr(fd, &orig_termios)) {
    printf("getting modem serial device attr\n");
    return -2;
  }
  cur_termios = orig_termios;

  /* input modes - turn off input processing */
  cur_termios.c_iflag &= ~(IGNBRK | BRKINT | IGNPAR | PARMRK | INPCK | ISTRIP | INLCR | IGNCR
                           | ICRNL | IXON | IXANY | IXOFF | IMAXBEL);
  /* pas IGNCR sinon il vire les 0x0D */
  cur_termios.c_iflag |= BRKINT;

  /* output_flags - turn off output processing */
  cur_termios.c_oflag  &= ~(OPOST | ONLCR | OCRNL | ONOCR | ONLRET);

  /* control modes */
  cur_termios.c_cflag &= ~(CSIZE | CSTOPB | CREAD | PARENB | PARODD | HUPCL | CLOCAL);
  cur_termios.c_cflag |= CREAD | CS8 | CLOCAL;
  cur_termios.c_cflag &= ~(CRTSCTS);

  /* local modes */
  cur_termios.c_lflag &= ~(ISIG | ICANON | IEXTEN | ECHO | FLUSHO | PENDIN);
  cur_termios.c_lflag |= NOFLSH;

  if (cfsetspeed(&cur_termios, br)) {
    printf("setting modem serial device speed\n");
    return -3;
  }

  if (tcsetattr(fd, TCSADRAIN, &cur_termios)) {
    printf("setting modem serial device attr\n");
    return -4;
  }

  return 0;
}


