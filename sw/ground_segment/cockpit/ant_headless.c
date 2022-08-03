 /*
 
 * Written by OpenUAS inspired by Chris Efstathiou code for the Pololu Micro Mestro usb servo controller
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 * The antenna tracker zero azimuth is to the NORTH (NORTH = 0, EAST = 90 WEST = -90, SOUTH = 180/0 degrees).
 * The elevation zero is totally horizontal, 90 is up and 180 is to the back.
 * The servo used must be able to do 180 degrees in order to get full 360 degree coverage from the tracker.

 *
 Compile via:
 gcc -Wall -fPIC -I/usr/include/glib-2.0 -I/usr/lib/x86_64-linux-gnu/glib-2.0/include -o ant_headless ant_headless.c -lglibivy -lglib-2.0 -lpcre -lm
*/

#include <glib.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>

#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>

#include <termios.h>
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */

#define POLOLU_PROTOCOL_START          0xAA
#define POLOLU_BOARD_ID                12 //Device Number set by board config
#define AC_ID                          1
#define SET_SERVO_POSITION_COMMAND     0x04
#define SET_SERVO_SPEED_COMMAND        0x07
#define SET_SERVO_ACCELERATION_COMMAND 0x09
#define SET_SERVO_CENTER_COMMAND       0x22

#define MANUAL 0
#define AUTO 1


//FIXME ADD DGPS message handling so to be more flexible and no AC Home Position is needed
static double gps_pos_x = 0;
static double gps_pos_y = 0;
static double gps_home_pos_x = 0;
static double gps_home_pos_y = 0;
static double gps_alt = 0;
static double home_alt = 0;
static double home_alt_backup = 0;
static double ant_azim = 0;
static double ant_elev = 0;
static int mode;
static int home_found = 0;
static int ant_tracker_pan_mode = 180;
static double theta_servo_pw_span = 0;
static double psi_servo_pw_span = 0;
static double theta_servo_pw_span_default = 1000.;
static double psi_servo_pw_span_default = 1000.;
static double theta_servo_center_pw = 1500;
static double psi_servo_center_pw = 1500;
static char pololu_board_id = 12; //Device number or is it serialnum
static char ac_id = 1;
static char servo_acceleration = 0;//3
static char psi_servo_address = 1;
static char theta_servo_address = 0;

int fd; /* File descriptor for the port */
volatile int serial_error = 0;

double hfov = 180., vfov = 180.;
double hnp = 0., vnp = 0.;
double elevation_trim = 0;
double elev_trim_pw = 0;
double azimuth_trim = 0;
double azim_trim_pw = 0;
unsigned char  speed = 0x00;
char str_count[30] = {0};

void set_servos(void);

//GtkBuilder  *builder;
//GtkWidget   *window;
//GError      *err = NULL;

int   azim_scale;
int   elev_scale;
int   hnp_scale;
//int   entry;
int   elev_trim_scale;
int   azim_trim_scale;
int   azimuth_pw_span_scale;
int   elevation_pw_span_scale;

void set_servos(void){

  double hpos = 0, vpos = 0;
  int hservo = theta_servo_center_pw, vservo = psi_servo_center_pw;
  
  elev_trim_pw = (elevation_trim / vfov) * psi_servo_pw_span;
  azim_trim_pw = (azimuth_trim / vfov) * theta_servo_pw_span;

  if (ant_tracker_pan_mode == 180) {
    // Take the vertical angle relative to the neutral point "vnp"
    vpos = ant_elev - vnp; 
    if(vpos < 0) { vpos = 0; }

    // keep within the field of view "vfov"
    //if (vpos > (vfov / 2)) { vpos = vfov / 2; } else if (-vpos > (vfov / 2)) { vpos = -vfov / 2; }

    // First take the horizontal angle relative to the neutral point "hnp"
    hpos = ant_azim - hnp;

    // Keep the range between (-180,180). this is done so that it consistently swaps sides
    if (hpos < -180) { hpos += 360; } else if (hpos > 180) { hpos -= 360; }

    // Swap sides to obtain 360 degrees of Azimuth coverage.
    if (hpos > 90) { hpos = hpos - 180;  vpos = 180 - vpos; } else if (hpos < -90) { hpos = hpos + 180;  vpos = 180 - vpos; }

    // keep the range within the field of view "hfov"
    if (hpos > (hfov / 2)) { hpos = hfov / 2; } else if (-hpos > (hfov / 2)) { hpos = -hfov / 2; }

    // Convert angles to servo microsecond values suitable for the Pololu micro Maestro servo controller.
    vpos = (psi_servo_center_pw - (psi_servo_pw_span / 2)) + (vpos * (psi_servo_pw_span / vfov));
    hpos = theta_servo_center_pw + (hpos * ((theta_servo_pw_span / 2) / (hfov / 2)));

    //convert the values to integer.
    hservo = hpos+azim_trim_pw;;
    vservo = vpos+elev_trim_pw;

  } else {
    vpos = ant_elev - vnp;
    if(vpos < 0) { vpos = 0; }
    if(vpos > (hfov/2)) { vpos = hfov/2; }
    // First take the horizontal angle relative to the neutral point "hnp"
    hpos = ant_azim - hnp;

    // Keep the range between (-180,180).
    if (hpos < -180) { hpos += 360; } else if (hpos > 180) { hpos -= 360; }

    // Keep the range between 0 to 360.
    //if (hpos < 0) { hpos += 360; } else if (hpos > 360){ hpos -= 360; }

    // keep the range within the field of view "hfov"
    if (hpos > (hfov / 2)) { hpos = hfov / 2; } else if (-hpos > (hfov / 2)) { hpos = -hfov / 2;  }

    // Convert angles to servo microsecond values suitable for the Pololu micro Maestro servo controller.
    vpos = (psi_servo_center_pw - (psi_servo_pw_span / 2)) + (vpos * (psi_servo_pw_span / vfov));
    hpos = theta_servo_center_pw + (hpos * (theta_servo_pw_span / hfov));

    //convert the values to integer.
    hservo = hpos+azim_trim_pw;;
    vservo = vpos+elev_trim_pw;
  }

  hservo *= 4; //The pololu Maestro uses 0.25 microsecond increments so we need to multiply microseconds by 4.
  vservo *= 4; //The pololu Maestro uses 0.25 microsecond increments so we need to multiply microseconds by 4.
  //g_message("home_alt %f gps_alt %f azim %f elev %f", home_alt, gps_alt, ant_azim, ant_elev);

  // Send servo position.
  char buffer1[] = { POLOLU_PROTOCOL_START, pololu_board_id, SET_SERVO_POSITION_COMMAND, psi_servo_address, vservo % 128, vservo / 128,
                     POLOLU_PROTOCOL_START, pololu_board_id, SET_SERVO_POSITION_COMMAND, theta_servo_address, hservo % 128, hservo / 128
                   };

  serial_error = write(fd, buffer1, 12);
  //Divide by 4 so we can have the servo PW with 1 microsecond resolution.
  //g_message("vservo %i hservo %i", (int)(vservo / 4), (int)(hservo / 4));
  //sprintf(str_count, "%d us", (hservo / 4));
  //gtk_label_set_text(GTK_LABEL(azim_servo_pw_label), str_count);
  //sprintf(str_count, "%d us", (vservo / 4));
  //gtk_label_set_text(GTK_LABEL(elev_servo_pw_label), str_count);

  return;
}

void on_GPS_STATUS(IvyClientPtr app, void *user_data, int argc, char *argv[]){
  if (home_found == 0) {
	if (atof(argv[0]) == 3) { /* wait until we have a valid GPS fix */
    	   home_alt = atof(argv[4])/100.; /* get the altitude */
    	   home_found = 1;
  	}
  }
  gps_alt = atof(argv[4])/100.;
  set_servos();
}

void on_GPS_LLA_STATUS(IvyClientPtr app, void *user_data, int argc, char *argv[]){
  // if (home_found == 0) {
	// if (atof(argv[0]) == 3) { /* wait until we have a valid GPS fix */ //TODO correct index
  //   	   home_alt = atof(argv[4])/100.; /* get the altitude */
  //   	   home_found = 1;
  // 	}
  // }
  // gps_alt = atof(argv[4])/100.;
  // set_servos();
}

void on_NAV_STATUS(IvyClientPtr app, void *user_data, int argc, char *argv[]){

  if (mode == AUTO) {
    gps_pos_x = atof(argv[2]);
    gps_pos_y = atof(argv[3]);
    /* calculate azimuth */
    ant_azim = atan2(gps_pos_x, gps_pos_y) * 180. / M_PI;
    if (ant_azim < 0) ant_azim += 360.;
    /* calculate elevation */
    ant_elev = atan2( (gps_alt-home_alt), sqrt(atof(argv[5])) ) * 180. / M_PI;
    if (ant_elev < 0) ant_elev = 0.;

//azim_scale

    //gtk_range_set_value(azim_scale, ant_azim);
    //gtk_range_set_value(elev_scale, ant_elev);
  }
   g_message("home_alt %f gps_alt %f azim %f elev %f gps_pos_x %f", home_alt, gps_alt, ant_azim, ant_elev, gps_pos_x );
    //  ant_azim = gtk_range_get_value(GTK_RANGE(azim_scale));
    //ant_elev = gtk_range_get_value(GTK_RANGE(elev_scale));
    set_servos();
}

int open_port(char *port){

  struct termios options;

  // would probably be good to set the port up as an arg.
  // The Pololu micro maestro registers two ports /dev/ttyACM0 and /dev/ttyACM1, /dev/ttyACM0 is the data port.
  fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd == -1) {
    //perror("open_port: Unable to open /dev/ttyUSB1");
    printf("open_port: Unable to open %s \n", port);
    serial_error = fd;

  } else {
    printf("Success, %s %s \n", port, "opened");
  }
  fcntl(fd, F_SETFL, 0);

  tcgetattr(fd, &options);

  // Set the baud rates to 19200. This can be between 2,000 to 40,000

  cfsetispeed(&options, B19200);
  cfsetospeed(&options, B19200);

  options.c_cflag |= (CLOCAL | CREAD);

  tcsetattr(fd, TCSANOW, &options);

  // Send initialisation to the pololu micro maestro board.
  // if "speed" is nonzero then 1 is the slowest 127 is the fastest. 0 = no speed restriction
  char buffer_0[] = { POLOLU_PROTOCOL_START, pololu_board_id, SET_SERVO_SPEED_COMMAND, psi_servo_address, 0x00, 0x00,
                      POLOLU_PROTOCOL_START, pololu_board_id, SET_SERVO_SPEED_COMMAND, theta_servo_address, 0x00, 0x00
                    };

  serial_error = write(fd, buffer_0, 12);
  // Set servo acceleration to 3 for protecting the servo gears. Fastest = 0, slowest = 255
  char buffer_1[] = { POLOLU_PROTOCOL_START, pololu_board_id, SET_SERVO_ACCELERATION_COMMAND, psi_servo_address, (servo_acceleration % 128),
                      (servo_acceleration / 128),
                      POLOLU_PROTOCOL_START, pololu_board_id, SET_SERVO_ACCELERATION_COMMAND, theta_servo_address, (servo_acceleration % 128),
                      (servo_acceleration / 128)
                    };

  serial_error = write(fd, buffer_1, 12);
  // Set the two servos to their neutral position, Azimuth = 1500us = EAST = 0 degrees & Elevation = 1000 = parallel to ground.
  char buffer_2[] = { POLOLU_PROTOCOL_START, pololu_board_id, SET_SERVO_POSITION_COMMAND, theta_servo_address,
                      (((int)theta_servo_center_pw * 4) % 128), (((int)theta_servo_center_pw * 4) / 128),
                      POLOLU_PROTOCOL_START, pololu_board_id, SET_SERVO_POSITION_COMMAND, psi_servo_address,
                      (((int)psi_servo_center_pw * 4) % 128), (((int)psi_servo_center_pw * 4) / 128)
                    };

  serial_error = write(fd, buffer_2, 12);

  return (fd);
}

int main ( int argc, char** argv) {

  int x = 0, y = 0, z = 0;
  char buffer[20];
  char serial_open = 0;

  printf("Tracker application for the Paparazzi autopilot system \n");
  printf("Can be used to keep an Antenna, Camera, Microphone or whatnot pointed straight at an flying, driving, floating object \n");

  mode = AUTO;

  if (argc > 1) {
    char arg_string1[] = "--help";
    for (x = 1; x < argc; x++) {
      if ((strncmp(argv[x], arg_string1, (sizeof(arg_string1) - 1))) == 0) {
        printf("OPTIONS \n");
        printf("-------------------------------------------------------------------------------- \n");
        printf("'--help' displays this screen \n");
        printf("'--port=xxx..x' opens port xxx..x, example --port=/dev/ttyACM0 (Default) \n");
        printf("'--pan=xxx' sets pan mode to 180 or 360 degrees. Example --pan=180 (Default) \n");
        printf("'--zero_angle=xxx' set the mechanical zero angle. Default is 0 (North)\n");
        printf("'--id=xx' sets the Pololu board id. Example --id=12 (Default)\n");
        printf("'--servo_acc=xxx' sets the servo acceleration. Example --servo_acc=3 (Default)\n");
        printf("'--pan_servo=x' sets the pan (Theta) servo number. Example --pan_servo=0 (Default)\n");
        printf("'--tilt_servo=x' sets the tilt (Psi) servo number.Example --tilt_servo=1 (Default) \n");
        printf("'--pan_epa=xx..x' sets the Azimuth servo's max travel (Default is 1000us) \n");
        printf("'--tilt_epa=xx..x' sets the elevation servo's max travel (Default is 1000us). \n");
        printf("HINT a negative value EPA value reverses the servo direction \n");
        printf("'--pan_servo_center_pw=xx..x' sets the Azimuth servo's center position (Default is 1500us) \n");
        printf("'--tilt_servo_center_pw=xx..x' sets the elevation servo's center position (Default is 1500us) \n");
        printf("'--ac=xxx' (NOT IMPLEMENTED FIXME) Set Aircraft ID to track xxx, example --ac=11, Default is ALL Aircraft, but that is plain weird \n");
        printf("WARNING: The pololu board limit servo travel to 1000-2000 microseconds. \n");
        printf("WARNING: Use the pololu board setup program to change the above limits. \n");
        printf("Example --tilt_epa=1100 sets the PW from 950 to 2050 microseconds \n");
        printf("Example --pan_epa=-1000 sets the PW from 1000 to 2000 microseconds and reverses the servo direction \n");
        printf("An EPA of 1100 sets the servo travel from 1500+(1100/2)=2050us to 1500-(1100/2)=950us. \n");
        printf("Use programmable servos like the Hyperion Atlas. \n");
        printf("You can also use the proportional 360 degree GWS S125-1T as the Theta (Azimuth) \n");
        printf(" \n");
        printf("FOR THE 360 DEGREE PAN MODE: \n");
        printf("Mechanical zero (0 degrees or 1500 ms) is to the NORTH, 90 = EAST, +-180 = SOUTH and -90 = WEST. \n");
        printf("Elevation center is 45 degrees up (1500ms), 0 degrees = horizontal, 90 degrees is vertical (up) \n");
        printf("Of course use this mode if your PAN servo can do a full 360 degrees rotation (GWS S125-1T for example) \n");
        printf(" \n");
        printf("FOR THE 180 DEGREE PAN MODE: \n");
        printf("Mechanical zero (0 degrees or 1500 ms) is to the NORTH, 90 = EAST, -90 = WEST. \n");
        printf("Elevation center is 90 degrees up (1500ms), 0 degrees = horizontal, 180 degrees is horizontal to the opposite side \n");
        printf("When the azimuth is > 90 or < -90 the azimuth and elevation servos swap sides to obtain the full 360 degree coverage. \n");
        printf("Of course your PAN and TILT servos must be true 180 degrees servos like the Hyperion ATLAS servos for example. \n");
        printf(" \n");
        printf("-------------------------------------------------------------------------------- \n");
        printf("Tracker v0.1 for the Paparazzi autopilot 27/July/2022\n");
        printf("-------------------------------------------------------------------------------- \n");
        return 0;
      }
    }
    printf("Type '--help' for help \n");

    for (z = 0; z < sizeof(buffer); z++) { buffer[z] = '\0'; } //Reset the buffer.
    char arg_string2[] = "--port=";
    for (x = 1; x < argc; x++) {
      if ((strncmp(argv[x], arg_string2, (sizeof(arg_string2) - 1))) == 0) {
        y = sizeof(arg_string2) - 1;
        z = 0;
        while (1) {
          buffer[z] = argv[x][y];
          if (buffer[z] != '\0') { y++; z++; } else { break; }
        }
        printf("Trying to open %s \n", buffer);
        open_port(buffer);
      }
    }

    for (z = 0; z < sizeof(buffer); z++) { buffer[z] = '\0'; } //Reset the buffer.
    char arg_string3[] = "--pan=";
    for (x = 1; x < argc; x++) {
      if ((strncmp(argv[x], arg_string3, (sizeof(arg_string3) - 1))) == 0) {
        y = (sizeof(arg_string3) - 1);
        z = 0;
        while (1) {
          buffer[z] = argv[x][y];
          if (buffer[z] != '\0') { y++; z++; } else { break; }
        }
        ant_tracker_pan_mode = atoi(buffer);
        if (ant_tracker_pan_mode == 180 || ant_tracker_pan_mode == 360) {
          printf("PAN mode set to %i %s \n", ant_tracker_pan_mode, "degrees");
          if (ant_tracker_pan_mode == 360) { hfov = 360; vfov = 90; } else { hfov = 180; vfov = 180; }

        } else {
          perror("ERROR: Pan mode can be either 180 or 360 degrees");
          ant_tracker_pan_mode = 180;
          hfov = 180;
          vfov = 180;
          printf("PAN servo set to %i %s \n", ant_tracker_pan_mode, "degrees");
        }
      }
    }

    for (z = 0; z < sizeof(buffer); z++) { buffer[z] = '\0'; } //Reset the buffer.
    char arg_string4[] = "--pan_epa=";
    for (x = 1; x < argc; x++) {
      if ((strncmp(argv[x], arg_string4, (sizeof(arg_string4) - 1))) == 0) {
        y = (sizeof(arg_string4) - 1);
        z = 0;
        while (1) {
          buffer[z] = argv[x][y];
          if (buffer[z] != '\0') { y++; z++; } else { break; }
        }
        theta_servo_pw_span_default = atoi(buffer);
        printf("THETA servo EPA set to  %i \n", atoi(buffer));
        if (abs(theta_servo_pw_span_default) > 1000) {
          printf("REMEMBER TO SET THE MIN/MAX SERVO LIMITS WITH THE POLOLU SETUP PROGRAM \n");
          printf("OTHERWISE THE MAX SERVO MOVEMENT WILL BE RESTRAINED TO 1000 MICROSECONDS \n");
        }
      }
    }

    for (z = 0; z < sizeof(buffer); z++) { buffer[z] = '\0'; } //Reset the buffer.
    char arg_string5[] = "--tilt_epa=";
    for (x = 1; x < argc; x++) {
      if ((strncmp(argv[x], arg_string5, (sizeof(arg_string5) - 1))) == 0) {
        y = (sizeof(arg_string5) - 1);
        z = 0;
        while (1) {
          buffer[z] = argv[x][y];
          if (buffer[z] != '\0') { y++; z++; } else { break; }
        }
        psi_servo_pw_span_default = atoi(buffer);
        printf("PSI servo EPA set to  %i \n", atoi(buffer));
        if (abs(psi_servo_pw_span_default) > 1000) {
          printf("REMEMBER TO SET THE MIN/MAX SERVO LIMITS WITH THE POLOLU SETUP PROGRAM \n");
          printf("OTHERWISE THE MAX SERVO MOVEMENT WILL BE RESTRAINED TO 1000 MICROSECONDS \n");
        }
      }
    }

    for (z = 0; z < sizeof(buffer); z++) { buffer[z] = '\0'; } //Reset the buffer.
    char arg_string8[] = "--id=";
    for (x = 1; x < argc; x++) {
      if ((strncmp(argv[x], arg_string8, (sizeof(arg_string8) - 1))) == 0) {
        y = (sizeof(arg_string8) - 1);
        z = 0;
        while (1) {
          buffer[z] = argv[x][y];
          if (buffer[z] != '\0') { y++; z++; } else { break; }
        }
        pololu_board_id = (char)atoi(buffer);
        printf("Pololu Board id set to  %i \n", atoi(buffer));
      }
    }

    for (z = 0; z < sizeof(buffer); z++) { buffer[z] = '\0'; } //Reset the buffer.
    char arg_string9[] = "--servo_acc=";
    for (x = 1; x < argc; x++) {
      if ((strncmp(argv[x], arg_string9, (sizeof(arg_string9) - 1))) == 0) {
        y = (sizeof(arg_string9) - 1);
        z = 0;
        while (1) {
          buffer[z] = argv[x][y];
          if (buffer[z] != '\0') { y++; z++; } else { break; }
        }
        servo_acceleration = (char)atoi(buffer);
        printf("Servo acceleration set to  %i \n", atoi(buffer));
      }
    }

    for (z = 0; z < sizeof(buffer); z++) { buffer[z] = '\0'; } //Reset the buffer.
    char arg_string10[] = "--pan_servo=";
    for (x = 1; x < argc; x++) {
      if ((strncmp(argv[x], arg_string10, (sizeof(arg_string10) - 1))) == 0) {
        y = (sizeof(arg_string10) - 1);
        z = 0;
        while (1) {
          buffer[z] = argv[x][y];
          if (buffer[z] != '\0') { y++; z++; } else { break; }
        }
        theta_servo_address = (char)atoi(buffer);
        printf("Pan (Theta) servo number set to  %i \n", atoi(buffer));
      }
    }

    for (z = 0; z < sizeof(buffer); z++) { buffer[z] = '\0'; } //Reset the buffer.
    char arg_string11[] = "--tilt_servo=";
    for (x = 1; x < argc; x++) {
      if ((strncmp(argv[x], arg_string11, (sizeof(arg_string11) - 1))) == 0) {
        y = (sizeof(arg_string11) - 1);
        z = 0;
        while (1) {
          buffer[z] = argv[x][y];
          if (buffer[z] != '\0') { y++; z++; } else { break; }
        }
        psi_servo_address = (char)atoi(buffer);
        printf("Tilt (Psi) servo number set to  %i \n", atoi(buffer));
      }
    }

    for (z = 0; z < sizeof(buffer); z++) { buffer[z] = '\0'; } //Reset the buffer.
    char arg_string12[] = "--zero_angle=";
    for (x = 1; x < argc; x++) {
      if ((strncmp(argv[x], arg_string12, (sizeof(arg_string12) - 1))) == 0) {
        y = (sizeof(arg_string12) - 1);
        z = 0;
        while (1) {
          buffer[z] = argv[x][y];
          if (buffer[z] != '\0') { y++; z++; } else { break; }
        }
        hnp = (double)atoi(buffer);
        printf("Zero angle is set to %i %s \n", atoi(buffer), "degrees");
      }
    }

    for (z = 0; z < sizeof(buffer); z++) { buffer[z] = '\0'; } //Reset the buffer.
    char arg_string13[] = "--tilt_servo_center_pw=";
    for (x = 1; x < argc; x++) {
      if ((strncmp(argv[x], arg_string13, (sizeof(arg_string13) - 1))) == 0) {
        y = (sizeof(arg_string13) - 1);
        z = 0;
        while (1) {
          buffer[z] = argv[x][y];
          if (buffer[z] != '\0') { y++; z++; } else { break; }
        }
        psi_servo_center_pw = atoi(buffer);
        printf("PSI servo center pulse width set to  %i \n", atoi(buffer));
      }
    }

    for (z = 0; z < sizeof(buffer); z++) { buffer[z] = '\0'; } //Reset the buffer.
    char arg_string14[] = "--pan_servo_center_pw=";
    for (x = 1; x < argc; x++) {
      if ((strncmp(argv[x], arg_string14, (sizeof(arg_string14) - 1))) == 0) {
        y = (sizeof(arg_string14) - 1);
        z = 0;
        while (1) {
          buffer[z] = argv[x][y];
          if (buffer[z] != '\0') { y++; z++; } else { break; }
        }
        theta_servo_center_pw = atoi(buffer);
        printf("THETA servo center pulse width set to  %i \n", atoi(buffer));
      }
    }

    for (z = 0; z < sizeof(buffer); z++) { buffer[z] = '\0'; } //Reset the buffer.
    char arg_string15[] = "--hfov=";
    for (x = 1; x < argc; x++) {
      if ((strncmp(argv[x], arg_string15, (sizeof(arg_string15) - 1))) == 0) {
        y = (sizeof(arg_string15) - 1);
        z = 0;
        while (1) {
          buffer[z] = argv[x][y];
          if (buffer[z] != '\0') { y++; z++; } else { break; }
        }
        hfov = atoi(buffer);
        printf("Horizontal field of view set to %i %s \n", (int)hfov, "degrees");
      }
    }

    for (z = 0; z < sizeof(buffer); z++) { buffer[z] = '\0'; } //Reset the buffer.
    char arg_string16[] = "--vfov=";
    for (x = 1; x < argc; x++) {
      if ((strncmp(argv[x], arg_string16, (sizeof(arg_string16) - 1))) == 0) {
        y = (sizeof(arg_string16) - 1);
        z = 0;
        while (1) {
          buffer[z] = argv[x][y];
          if (buffer[z] != '\0') { y++; z++; } else { break; }
        }
        vfov = atoi(buffer);
        printf("Vertical field of view set to %i %s \n", (int)vfov, "degrees");
      }
    }
  }

  for (z = 0; z < sizeof(buffer); z++) { buffer[z] = '\0'; } //Reset the buffer.
    char arg_string17[] = "--ac=";
    for (x = 1; x < argc; x++) {
      if ((strncmp(argv[x], arg_string17, (sizeof(arg_string17) - 1))) == 0) {
        y = (sizeof(arg_string17) - 1);
        z = 0;
        while (1) {
          buffer[z] = argv[x][y];
          if (buffer[z] != '\0') { y++; z++; } else { break; }
        }
        ac_id = (char)atoi(buffer);
        printf("Aircraft id set to  %i \n", atoi(buffer));
      }
    }

  psi_servo_pw_span = psi_servo_pw_span_default;
  theta_servo_pw_span = theta_servo_pw_span_default;

  if (serial_open == 0) { printf("Trying to open /dev/ttyACM0 \n");  open_port("/dev/ttyACM0"); }

  GMainLoop *ml =  g_main_loop_new(NULL, FALSE);

  IvyInit("Ant0", "Ant0 READY", NULL, NULL, NULL, NULL);
  IvyBindMsg( on_GPS_STATUS, NULL, "^\\S* GPS (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*)");
  //IvyBindMsg( on_GPS_LLA_STATUS, NULL, "^\\S* GPS_LLA (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*)");
  IvyBindMsg(on_NAV_STATUS, NULL, "^\\S* NAVIGATION (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*)");
  IvyStart("127.255.255.255");

  g_main_loop_run(ml);

  return 0;
}


// static void on_Attitude(IvyClientPtr app, void *user_data, int argc, char *argv[]){
//     guint ac_id = atoi(argv[0]);
//     printf("Aircraft %d", ac_id);
//     float estimator_phi = 57.296f * atof(argv[1]);
//     float estimator_psi = 57.296f * atof(argv[2]);
//     float estimator_theta = 57.296f * atof(argv[3]);
//     printf(" psi is %f\n", estimator_psi);
// }
