#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <sys/time.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>

#define SHIP_INFO_MSG_GROUND_ID 192

#define MESSAGE_ON_TX_FREQUENCY_CALCULATION 20

#define MSG_OUT_TX_FREQUENCY 200

#define PPRZ_STX 0x99

