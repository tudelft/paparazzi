# Usage:
# make        # compile all binary
# make clean  # remove ALL binaries and objects
CC = gcc

PAPARAZZI_SRC=../../..
UNAME = $(shell uname -s)


CFLAGS += -Wall -fPIC -g
GTK_CFLAGS  = $(shell pkg-config gtk+-2.0 --cflags) -DGTK_DISABLE_DEPRECATED
GTK_LDFLAGS = $(shell pkg-config gtk+-2.0 --libs) $(shell pkg-config --libs ivy-glib) $(shell pcre-config --libs) -fPIC

GLIBIVY_CFLAGS = $(shell pkg-config --cflags ivy-glib)
GLIBIVY_LDFLAGS = $(shell pkg-config --libs ivy-glib) $(shell pcre-config --libs)

# fallback to ivy-glib pkg-config info if there is no ivy-c.pc
IVY_INC = $(shell pkg-config --cflags-only-I ivy-c 2> /dev/null)
IVY_LDFLAGS = $(shell pkg-config --libs ivy-c 2> /dev/null)
ifeq ($(strip $(IVY_INC)),)
IVY_INC = $(shell pkg-config --cflags-only-I ivy-glib)
IVY_LDFLAGS = $(shell pkg-config --libs-only-L ivy-glib) -livy
endif

# GLIB libraries
GLIB_LDFLAGS = $(shell pkg-config glib-2.0 --libs) -lglibivy -lm $(shell pcre-config --libs)

# Paparazzi includes
INCLUDES += $(shell pkg-config glib-2.0 --cflags) -I$(PAPARAZZI_SRC)/sw/airborne/ -I$(PAPARAZZI_SRC)/sw/include/ $(IVY_INC)
INCLUDES += -I$(PAPARAZZI_SRC)/sw/ext/libsbp/c/include/ -I$(PAPARAZZI_SRC)/sw/airborne/modules/gps/librtcm3/ -I$(PAPARAZZI_SRC)/sw/airborne/arch/linux/


make:
	@echo "Compiling..."

	gcc -o Pixhawk_4_read_msg_v3 Pixhawk_4_read_msg_v3_ivy.c $(INCLUDES) $(LIBRARYS) -lpthread $(GLIB_LDFLAGS) $(IVY_LDFLAGS)
clean:
	@echo "Cleaning up..."
	rm -rvf Pixhawk_4_read_msg_v3
