/*
 * Copyright (C) 2012-2014 The Paparazzi Community
 *               2015 Freek van Tienen <freek.v.tienen@gmail.com>
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 */

/**
 * @file modules/computer_vision/viewvideo.c
 *
 * Get live images from a RTP/UDP stream
 * and save pictures on internal memory
 *
 * Works on Linux platforms
 */

// Own header
#include "modules/computer_vision/viewvideo.h"
#include "modules/computer_vision/cv.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>
#include <math.h>

// Video
#include "lib/vision/image.h"
#include "lib/encoding/jpeg.h"
#include "lib/encoding/rtp.h"
#include "udp_socket.h"

#include BOARD_CONFIG

// Downsize factor for video stream
#ifndef VIEWVIDEO_DOWNSIZE_FACTOR
#define VIEWVIDEO_DOWNSIZE_FACTOR 4
#endif
PRINT_CONFIG_VAR(VIEWVIDEO_DOWNSIZE_FACTOR)

// From 0 to 99 (99=high)
#ifndef VIEWVIDEO_QUALITY_FACTOR
#define VIEWVIDEO_QUALITY_FACTOR 50
#endif
PRINT_CONFIG_VAR(VIEWVIDEO_QUALITY_FACTOR)

// RTP time increment at 90kHz (default: 0 for automatic)
#ifndef VIEWVIDEO_RTP_TIME_INC
#define VIEWVIDEO_RTP_TIME_INC 0
#endif
PRINT_CONFIG_VAR(VIEWVIDEO_RTP_TIME_INC)

// Define stream framerate
#ifndef VIEWVIDEO_FPS
#define VIEWVIDEO_FPS 5
#endif
PRINT_CONFIG_VAR(VIEWVIDEO_FPS)

// Define stream priority
#ifndef VIEWVIDEO_NICE_LEVEL
#define VIEWVIDEO_NICE_LEVEL 5
#endif
PRINT_CONFIG_VAR(VIEWVIDEO_NICE_LEVEL)

// Check if we are using netcat instead of RTP/UDP
#ifndef VIEWVIDEO_USE_NETCAT
#define VIEWVIDEO_USE_NETCAT FALSE
#endif

#if !VIEWVIDEO_USE_NETCAT && !(defined VIEWVIDEO_USE_RTP)
#define VIEWVIDEO_USE_RTP TRUE
#endif

#if VIEWVIDEO_USE_NETCAT
#include <sys/wait.h>
PRINT_CONFIG_MSG("[viewvideo] Using netcat.")
#else
struct UdpSocket video_sock1;
struct UdpSocket video_sock2;
PRINT_CONFIG_MSG("[viewvideo] Using RTP/UDP stream.")
PRINT_CONFIG_VAR(VIEWVIDEO_USE_RTP)
#endif

/* These are defined with configure */
PRINT_CONFIG_VAR(VIEWVIDEO_HOST)
PRINT_CONFIG_VAR(VIEWVIDEO_PORT_OUT)
PRINT_CONFIG_VAR(VIEWVIDEO_PORT2_OUT)

// Initialize the viewvideo structure with the defaults
struct viewvideo_t viewvideo = {
  .is_streaming = FALSE,
  .downsize_factor = VIEWVIDEO_DOWNSIZE_FACTOR,
  .quality_factor = VIEWVIDEO_QUALITY_FACTOR,
#if !VIEWVIDEO_USE_NETCAT
  .use_rtp = VIEWVIDEO_USE_RTP,
#endif
};

/**
 * Handles all the video streaming and saving of the image shots
 * This is a separate thread, so it needs to be thread safe!
 */
static struct image_t *viewvideo_function(struct UdpSocket *socket, struct image_t *img, uint32_t *rtp_frame_nr)
{
  // Resize image if needed
  struct image_t img_small;
  image_create(&img_small,
               img->w / viewvideo.downsize_factor,
               img->h / viewvideo.downsize_factor,
               IMAGE_YUV422);

  // Create the JPEG encoded image
  struct image_t img_jpeg;
  image_create(&img_jpeg, img_small.w/2, img_small.h, IMAGE_JPEG);

  // Blob detection
  struct image_t img_blob, img_contour;
  image_create(&img_blob, img_small.w/2, img_small.h, IMAGE_LABELS);
  image_create(&img_contour, img_small.w, img_small.h, IMAGE_YUV422);
  struct image_label_t labels[512];
  struct image_filter_t filter;
  filter.y_min = 0;
  filter.y_max = 110;
  filter.u_min = 50;
  filter.u_max = 205;
  filter.v_min = 50;
  filter.v_max = 205;

  // Initialize timing
  uint32_t microsleep = (uint32_t)(1000000. / (float)viewvideo.fps);
  struct timeval last_time;
  gettimeofday(&last_time, NULL);

#if VIEWVIDEO_USE_NETCAT
  char nc_cmd[64];
  sprintf(nc_cmd, "nc %s %d 2>/dev/null", STRINGIFY(VIEWVIDEO_HOST), VIEWVIDEO_PORT_OUT);
#endif

  if (viewvideo.is_streaming) {

    // Wait for a new frame (blocking)
    struct image_t img;
    v4l2_image_get(viewvideo.dev, &img);







    // Do a blob detection
    uint16_t labels_cnt = 512;
    image_labeling(&img, &img_blob, &filter, 1, labels, &labels_cnt);

    // Show blobs
    //uint16_t *img_buf = (uint16_t *)img_blob.buf;
    /*for(uint16_t i = 0; i < img_blob.buf_size/2; i++) {
      if(img_buf[i] != 0xFFFF && labels[labels[img_buf[i]].id].pixel_cnt > 20*20 && labels[labels[img_buf[i]].id].pixel_cnt < 200*200) {
        img_buf[i] = labels[img_buf[i]].id;
      } else {
        img_buf[i] = 0xFFFF;
      }
    }*/

    // Draw contour
    memset(img_contour.buf, 0x00, img_contour.buf_size);
    //img_buf = (uint16_t *)img_contour.buf;
    uint16_t cnt = 0;
    for(uint16_t i = 0; i < labels_cnt ; i++) {
      if(labels[i].id == i && labels[i].pixel_cnt > 20*20 && labels[i].pixel_cnt < 200*200) {
        struct image_label_t *label = &labels[i];
        image_contour(&img_blob, labels, label);


        // Make it visible
        /*for(uint16_t c = 0; c < label->contour_cnt; c++) {
          //printf("X: %d Y; %d\n", label->contour[c].x, label->contour[c].y);
          img_buf[label->contour[c].y*img_blob.w + label->contour[c].x] = c;
        }*/
        if(label->contour_cnt > 0 && image_square(label)) {
          cnt++;
          image_draw_line(&img_contour, &label->contour[label->corners[0]], &label->contour[label->corners[1]]);
          image_draw_line(&img_contour, &label->contour[label->corners[1]], &label->contour[label->corners[2]]);
          image_draw_line(&img_contour, &label->contour[label->corners[2]], &label->contour[label->corners[3]]);
          image_draw_line(&img_contour, &label->contour[label->corners[3]], &label->contour[label->corners[0]]);
          uint16_t code = 0;
          float help = image_code(&img_blob, label, &code);
          if(help > 0.8)
            printf("Code(%d): %d %f\n", label->id, code, help);
        }
      }
    }
    printf("Squares: %d\n", cnt);

    /*for(uint16_t i = 0; i < img_blob.buf_size/2; i++) {
      if(img_buf[i] != 0xFFFF) {
        printf("%3d ", img_buf[i]);
      } else {
        printf("  X ");
      }
      if(i % img_blob.w == img_blob.w-1)
        printf("\n");
    }
    printf("\n");*/

    image_to_grayscale(&img_contour, &img);









    // Check if we need to take a shot
    if (viewvideo.take_shot) {
      // Create a high quality image (99% JPEG encoded)
      struct image_t jpeg_hr;
      image_create(&jpeg_hr, img.w, img.h, IMAGE_JPEG);
      jpeg_encode_image(&img, &jpeg_hr, 99, TRUE);

      // Search for a file where we can write to
      char save_name[128];
      for (; viewvideo.shot_number < 99999; viewvideo.shot_number++) {
        sprintf(save_name, "%s/img_%05d.jpg", STRINGIFY(VIEWVIDEO_SHOT_PATH), viewvideo.shot_number);
        // Check if file exists or not
        if (access(save_name, F_OK) == -1) {
          FILE *fp = fopen(save_name, "w");
          if (fp == NULL) {
            printf("[viewvideo-thread] Could not write shot %s.\n", save_name);
          } else {
            // Save it to the file and close it
            fwrite(jpeg_hr.buf, sizeof(uint8_t), jpeg_hr.buf_size, fp);
            fclose(fp);
          }

          // We don't need to seek for a next index anymore
          break;
        }
      }

      // We finished the shot
      image_free(&jpeg_hr);
      viewvideo.take_shot = FALSE;
    }

    // Only resize when needed
    if (viewvideo.downsize_factor != 1) {
      image_yuv422_downsample(img, &img_small, viewvideo.downsize_factor);
      jpeg_encode_image(&img_small, &img_jpeg, VIEWVIDEO_QUALITY_FACTOR, VIEWVIDEO_USE_NETCAT);
    } else {
      jpeg_encode_image(img, &img_jpeg, VIEWVIDEO_QUALITY_FACTOR, VIEWVIDEO_USE_NETCAT);
    }

#if VIEWVIDEO_USE_NETCAT
    // Open process to send using netcat (in a fork because sometimes kills itself???)
    pid_t pid = fork();

    if (pid < 0) {
      printf("[viewvideo] Could not create netcat fork.\n");
    } else if (pid == 0) {
      // We are the child and want to send the image
      FILE *netcat = popen(nc_cmd, "w");
      if (netcat != NULL) {
        fwrite(img_jpeg.buf, sizeof(uint8_t), img_jpeg.buf_size, netcat);
        pclose(netcat); // Ignore output, because it is too much when not connected
      } else {
        printf("[viewvideo] Failed to open netcat process.\n");
      }

      // Exit the program since we don't want to continue after transmitting
      exit(0);
    } else {
      // We want to wait until the child is finished
      wait(NULL);
    }
#else
    if (viewvideo.use_rtp) {
      // Send image with RTP
      rtp_frame_send(
        socket,              // UDP socket
        &img_jpeg,
        0,                        // Format 422
        VIEWVIDEO_QUALITY_FACTOR, // Jpeg-Quality
        0,                        // DRI Header
        (img->ts.tv_sec * 1000000 + img->ts.tv_usec),
        rtp_frame_nr
      );
    }
#endif
  }

  // Free all buffers
  image_free(&img_jpeg);
  image_free(&img_small);
  return NULL; // No new images were created
}

#ifdef VIEWVIDEO_CAMERA
static struct image_t *viewvideo_function1(struct image_t *img)
{
  static uint32_t rtp_frame_nr = 0;
  return viewvideo_function(&video_sock1, img, &rtp_frame_nr);
}
#endif

#ifdef VIEWVIDEO_CAMERA2
static struct image_t *viewvideo_function2(struct image_t *img)
{
  static uint32_t rtp_frame_nr = 0;
  return viewvideo_function(&video_sock2, img, &rtp_frame_nr);
}
#endif

/**
 * Initialize the view video
 */
void viewvideo_init(void)
{
  viewvideo.is_streaming = true;

#if VIEWVIDEO_USE_NETCAT
  // Create an Netcat receiver file for the streaming
  sprintf(save_name, "%s/netcat-recv.sh", STRINGIFY(VIEWVIDEO_SHOT_PATH));
  FILE *fp = fopen(save_name, "w");
  if (fp != NULL) {
    fprintf(fp, "i=0\n");
    fprintf(fp, "while true\n");
    fprintf(fp, "do\n");
    fprintf(fp, "\tn=$(printf \"%%04d\" $i)\n");
    fprintf(fp, "\tnc -l 0.0.0.0 %d > img_${n}.jpg\n", (int)(VIEWVIDEO_PORT_OUT));
    fprintf(fp, "\ti=$((i+1))\n");
    fprintf(fp, "done\n");
    fclose(fp);
  } else {
    printf("[viewvideo] Failed to create netcat receiver file.\n");
  }
#else
  // Open udp socket
#ifdef VIEWVIDEO_CAMERA
  if (udp_socket_create(&video_sock1, STRINGIFY(VIEWVIDEO_HOST), VIEWVIDEO_PORT_OUT, -1, VIEWVIDEO_BROADCAST)) {
    printf("[viewvideo]: failed to open view video socket, HOST=%s, port=%d\n", STRINGIFY(VIEWVIDEO_HOST),
           VIEWVIDEO_PORT_OUT);
  }
#endif

#ifdef VIEWVIDEO_CAMERA2
  if (udp_socket_create(&video_sock2, STRINGIFY(VIEWVIDEO_HOST), VIEWVIDEO_PORT2_OUT, -1, VIEWVIDEO_BROADCAST)) {
    printf("[viewvideo]: failed to open view video socket, HOST=%s, port=%d\n", STRINGIFY(VIEWVIDEO_HOST),
           VIEWVIDEO_PORT2_OUT);
  }
#endif
#endif

#ifdef VIEWVIDEO_CAMERA
  struct video_listener *listener1 = cv_add_to_device_async(&VIEWVIDEO_CAMERA, viewvideo_function1,
                                     VIEWVIDEO_NICE_LEVEL, VIEWVIDEO_FPS);
  fprintf(stderr, "[viewvideo] Added asynchronous video streamer listener for CAMERA1 at %u FPS \n", VIEWVIDEO_FPS);
#endif

#ifdef VIEWVIDEO_CAMERA2
  struct video_listener *listener2 = cv_add_to_device_async(&VIEWVIDEO_CAMERA2, viewvideo_function2,
                                     VIEWVIDEO_NICE_LEVEL, VIEWVIDEO_FPS);
  fprintf(stderr, "[viewvideo] Added asynchronous video streamer listener for CAMERA2 at %u FPS \n", VIEWVIDEO_FPS);
#endif
}
