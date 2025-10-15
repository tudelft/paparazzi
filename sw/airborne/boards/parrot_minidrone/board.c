/*
 * Copyright (C) 2017 The Paparazzi Team
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file boards/parrot_minidrone/board.c
 *
 * Parrot Minidrone board initialization functions.
 *
 */
#include "boards/parrot_minidrone.h"
#include "mcu.h"
#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>
#include <linux/input.h>
#include "modules/energy/electrical.h"

#include <sys/resource.h>// for setrlimit, not use ATM
#include <errno.h> //Remove after it is not needed anymore debugging

#define MAXPATHLEN 200   /* make this larger if you need to. */

// not used atm but thingy below #include <linux/videodev2.h>
#include "modules/computer_vision/lib/v4l/v4l2.h"
#include "peripherals/video_device.h"

struct video_config_t front_camera = {};

struct video_config_t bottom_camera = {
  .output_size = {
    .w = 160,
    .h = 120
  },
  .sensor_size = {
    .w = 160,
    .h = 120
  },
  .crop = {
    .x = 0,
    .y = 0,
    .w = 160,
    .h = 120
  },
  .dev_name = "/dev/video0", //TODO start useing the symlink? /dev/vertical_camera
  .subdev_name = NULL,
  .format = V4L2_PIX_FMT_YUYV, //AFAIK Sadly no UYUV support?
  //.format = V4L2_PIX_FMT_UYVY,
  //.subdev_format = V4L2_MBUS_FMT_UYVY8_2X8,
  .buf_cnt = 60,
  .filters = 0,
  .cv_listener = NULL,
  .fps = MT9V117_TARGET_FPS,
    .camera_intrinsics = {
    .focal_x = MT9V117_FOCAL_X,
    .focal_y = MT9V117_FOCAL_Y,
    .center_x = MT9V117_CENTER_X,
    .center_y = MT9V117_CENTER_Y,
    .Dhane_k = MT9V117_DHANE_K
  }
};

/**
 * Battery reading thread
 */
static void *bat_read(void *data __attribute__((unused)))
{
  FILE *fp;
  char path[16];

  while (TRUE) {
    /* Open the command for reading. */
    fp = popen("cat /sys/devices/platform/p6-spi.2/spi2.0/vbat", "r");
    if (fp == NULL) {
      printf("Failed to read battery\n");
    } else {
      /* Read the output a line at a time - output it. */
      while (fgets(path, sizeof(path) - 1, fp) != NULL) {
        int raw_bat = atoi(path);
        // convert to decivolt
        // from /bin/mcu_vbat.sh: MILLIVOLTS_VALUE=$(( ($RAW_VALUE * 4250) / 1023 ))
        electrical.vsupply = (float)((raw_bat * 4250) / 1023) / 1000.f;
      }
      /* close */
      pclose(fp);
    }

    // Wait 100ms
    // reading is done at 10Hz like the electrical_periodic from rotorcraft main program
    usleep(100000);
  }

  return NULL;
}

/**
 * Check power button pressed status
 */
static void *button_read(void *data __attribute__((unused)))
{
  struct input_event ev;
  ssize_t n;

  /* Open power button event sysfs file */
  int fd_button = open("/dev/input/pm_mcu_event", O_RDONLY);
  if (fd_button == -1) {
    printf("Unable to open mcu_event to read power button state\n");
    return NULL;
  }

  while (TRUE) {
    /* Check power button (read is blocking) */
    n = read(fd_button, &ev, sizeof(ev));

    //printf("Read n: %d", n);
    //printf("Read type, code, value: %d,%d,%d\n", ev.type, ev.code, ev.value);

    if (n == sizeof(ev) && ev.type == EV_KEY && ev.code == KEY_POWER && ev.value > 0) {
      //printf("Stopping Paparazzi from power button and rebooting\n");
      usleep(1000);
      int ret __attribute__((unused)) = system("reboot.sh");
      exit(0);
    }
  }

  return NULL;
}

void board_init(void)
{
  /*
   *  Stop original processes using pstop/ptart commands
   *  Don't kill as to avoid automatic restart of the processes
   */
  int ret __attribute__((unused));

  ret = system("pstop delosd");
  ret = system("pstop dragon-prog");

  /* If our OS stack size is not set correctly, we cannot start the UDP thread, so we dynamically set it here
  an option would be to set it in an init script, but it would require a drone os change and reboot to take effect */
  char os_commandline[MAXPATHLEN] = "ulimit -s ";
  char response[6] = "";
  FILE *fp;

  fflush(NULL);
  fp = popen(os_commandline, "r");
  if (fp != NULL) {
    fgets(response, sizeof(response) - 1, fp);
    fflush(fp);
    pclose(fp);
  } else {
    fprintf(stderr,"Error getting current OS stack size.\n");
    exit(EXIT_FAILURE);
  } 

  if(atoi(response)>512) { //NOTE: 512, randome but as for now seems to be the stack size that works
    ret = readlink("/proc/self/exe", os_commandline, sizeof(os_commandline));
    if (ret < 0) {
        fprintf(stderr, "Error resolving symlink /proc/self/exe.\n");
        exit(EXIT_FAILURE);
    }
    if (ret >= MAXPATHLEN) {
        fprintf(stderr, "Path too long. Truncated.\n");
        exit(EXIT_FAILURE);
    }

    os_commandline[ret] = '\0';  //remove @ from end of line
    char full_commandline[MAXPATHLEN] = "ulimit -s 512 && ";
    strcat(full_commandline, os_commandline);
    //printf("Full commandline is: %s\n", full_commandline);
    ret = system(full_commandline);
    exit(ret);
  }

  usleep(50000); /* Give 50ms time to end on a busy system */

  /* Start battery reading thread*/
  //TODO: Optionally move this a module like in ARDrone2
  pthread_t bat_thread;
  if (pthread_create(&bat_thread, NULL, bat_read, NULL) != 0) {
    printf("[parrot_minidrone_board] Could not create battery reading thread!\n");
  }
  pthread_setname_np(bat_thread, "pprz_bat_thread");

  /* Start button reading thread */
  pthread_t button_thread;
  if (pthread_create(&button_thread, NULL, button_read, NULL) != 0) {
    printf("[parrot_minidrone_board] Could not create button reading thread!\n");
  }
  pthread_setname_np(button_thread, "pprz_button_thread");

  /* NOTE Barometer senor reading is added via common barometric code*/

  /* NOTE: Ultrasonic ranging sensor reading is handled by optional ranging/sonar module "sonar_parrot_minidrone" */

  /* NOTE: Mainboard default camera reading is handled by optional module "Video thread" */
}

