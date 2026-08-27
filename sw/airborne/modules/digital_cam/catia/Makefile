
PAPARAZZI_HOME ?= $(abspath ../../../../..)

CATIA_SERIAL_DEVICE ?= /dev/ttySAC0
CATIA_FAKE_IMAGE ?= $(PAPARAZZI_HOME)/conf/simulator/Malolo1/thumbnail.jpg

CFLAGS = -static -Wall -Wextra \
	-DCATIA_SERIAL_DEVICE='"$(CATIA_SERIAL_DEVICE)"' \
	-DCATIA_FAKE_IMAGE='"$(CATIA_FAKE_IMAGE)"'
LINK   = -lpthread
CFILES = serial.c \
	 chdk_pipe.c \
	 socket.c \
	 protocol.c
HFILES = $(CFILES:.c=.h)


all:	catia

catia:	$(CFILES) $(HFILES) catia.c
	gcc $(CFLAGS) $(CFILES) catia.c -o catia $(LINK)

clean:
	rm -rf ./catia
