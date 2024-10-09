# Usage:
# make        # compile all binary
# make clean  # remove ALL binaries and objects
CXX = g++

PAPARAZZI_SRC=../../../..
UNAME = $(shell uname -s)

CXXFLAGS += -Wall -fPIC -g -std=c++14 -pthread
GTK_CFLAGS  = $(shell pkg-config gtk+-2.0 --cflags) -DGTK_DISABLE_DEPRECATED
GTK_LDFLAGS = $(shell pkg-config gtk+-2.0 --libs) $(shell pkg-config --libs ivy-glib) $(shell pcre-config --libs) -fPIC

GLIBIVY_CFLAGS = $(shell pkg-config --cflags ivy-glib)
GLIBIVY_LDFLAGS = $(shell pkg-config --libs ivy-glib) $(shell pcre-config --libs)

LDFLAGS = -L ../bin/rpi64/ -lSdsFalcon -Wl,-rpath,../bin/rpi64/

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
INCLUDES += -I../include  # Add the path to SdsFalcon.h
INCLUDES += -I/usr/include/Ivy  # Correct path to ivy.h

SOURCES = custom_mode_am_v2.cpp
OBJECTS = $(SOURCES:.cpp=.o)
TARGET = custom_mode_am

# Default target
all: $(TARGET)

# Build target
$(TARGET): $(OBJECTS)
	$(CXX) -o $@ $^ $(CXXFLAGS) $(INCLUDES) $(GTK_LDFLAGS) $(GLIBIVY_LDFLAGS) $(GLIB_LDFLAGS) $(IVY_LDFLAGS) $(LDFLAGS)

# Pattern rule for object files
%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@ $(LDFLAGS)

# Clean up
clean:
	@echo "Cleaning up..."
	rm -f $(OBJECTS) $(TARGET)
