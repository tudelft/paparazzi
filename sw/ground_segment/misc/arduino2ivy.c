/*
 * Paparazzi UBLox to Ivy
 *
 * Copyright (C) 2021 Freek van Tienen <freek.v.tienen@gmail.com>
 *
 * This file is part of paparazzi.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/*
#include <stdio.h>
#include <unistd.h>
#include <getopt.h>
#include <stdbool.h>
#include <stdint.h>
#include <arpa/inet.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <pthread.h>
#include <string.h>
#include <math.h>
#include <fcntl.h>
#include <termios.h>
#include <glib.h>
#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h> */

#include <stdio.h>
#include <string.h>
#include "arduino2ivy.h"

#define BUF_SIZE 1024*2

char serialPortFilename[] = "/dev/ttyUSB2";



int main(int argc, char** argv) {

  FILE *serPort = fopen(serialPortFilename, "r");

  static char txt[2];
  static size_t indx = 0;

  while(1){
    while (available()) {
    // read input
    char c = read();
    txt[indx++] = c;

    // report
    printf("Received: '");
    printf(c);
    printf("', string: \"");
    writef(txt, indx);
    printf("\", size: ");
    printf(indx);
    if (indx == 2 && memcmp(txt, "OK", 2) == 0) {
      printf("Correct message");
    }

    // reset index
    if (indx >= sizeof txt) {
      indx = 0;
    }
    }
  }

  /*
  char readBuffer[BUF_SIZE];
  int numBytesRead;

  FILE *serPort = fopen(serialPortFilename, "r");

	if (serPort == NULL)
	{
		printf("ERROR");	
		return 0;
	}

	printf(serialPortFilename);
	printf(":\n");
	while(1)
	{
		memset(readBuffer, 0, BUF_SIZE);
		fread(readBuffer, sizeof(char),BUF_SIZE,serPort);
		if(sizeof(readBuffer) != 0)
		{
			printf(readBuffer);
		}
	}

  fclose(serialPortFilename);  
  */

  return 0;
}
