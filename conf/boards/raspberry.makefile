# Hey Emacs, this is a -*- makefile -*-
#
# raspberry.makefile
#
# http://wiki.paparazziuav.org/wiki/TODO
#

BOARD=raspberry
BOARD_CFG=\"boards/$(BOARD).h\"

ARCH=linux
$(TARGET).ARCHDIR = $(ARCH)
ap.MAKEFILE = raspberry

# -----------------------------------------------------------------------
USER=pi
HOST?=192.168.42.1
SUB_DIR=paparazzi
FTP_DIR=~
TARGET_DIR=$(FTP_DIR)/$(SUB_DIR)
# -----------------------------------------------------------------------

# The datalink default uses UDP
MODEM_HOST         ?= 192.168.42.255

# The GPS sensor is connected internally
GPS_PORT           ?= UART1
GPS_BAUD           ?= B230400

# SBUS port, mapped to internal /dev/uart-sbus
SBUS_PORT ?= UART3
$(TARGET).CFLAGS += -DUSE_ARBITRARY_BAUDRATE

# handle linux signals by hand
$(TARGET).CFLAGS += -DUSE_LINUX_SIGNAL -D_GNU_SOURCE

# board specific init function
# if you need specific code to run (e.g. kill parrot app)
$(TARGET).srcs +=  $(SRC_BOARD)/board.c

# Link static (Done for GLIBC)
$(TARGET).CFLAGS += -DLINUX_LINK_STATIC
$(TARGET).LDFLAGS += -static

# limit main loop to 1kHz so ap doesn't need 100% cpu
#$(TARGET).CFLAGS += -DLIMIT_EVENT_POLLING

# -----------------------------------------------------------------------

# default LED configuration
RADIO_CONTROL_LED           ?= none
BARO_LED                    ?= none
AHRS_ALIGNER_LED            ?= none
GPS_LED                     ?= none
SYS_TIME_LED                ?= none
