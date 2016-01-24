
#serial UDP

include $(CFG_SHARED)/udp.makefile

MODEM_DEV         ?= UDP0
MODEM_PORT_OUT    ?= 4242
MODEM_PORT_IN     ?= 4243
MODEM_BROADCAST   ?= TRUE

UDP_MODEM_PORT_LOWER=$(shell echo $(MODEM_DEV) | tr A-Z a-z)
UDP_MODEM_PORT_UPPER=$(shell echo $(MODEM_DEV) | tr a-z A-Z)

MODEM_CFLAGS  = -DUSE_$(UDP_MODEM_PORT_UPPER) -D$(UDP_MODEM_PORT_UPPER)_PORT_OUT=$(MODEM_PORT_OUT) -D$(UDP_MODEM_PORT_UPPER)_PORT_IN=$(MODEM_PORT_IN)
MODEM_CFLAGS += -D$(UDP_MODEM_PORT_UPPER)_BROADCAST=$(MODEM_BROADCAST) -D$(UDP_MODEM_PORT_UPPER)_HOST=$(MODEM_HOST)

TELEM_CFLAGS  = -DDOWNLINK -DPERIODIC_TELEMETRY -DDOWNLINK_DEVICE=$(UDP_MODEM_PORT_LOWER) -DPPRZ_UART=$(UDP_MODEM_PORT_LOWER)
TELEM_CFLAGS += -DDOWNLINK_TRANSPORT=pprz_tp -DDATALINK=PPRZ


$(TARGET).CFLAGS += $(MODEM_CFLAGS) $(TELEM_CFLAGS)
$(TARGET).srcs += subsystems/datalink/downlink.c $(PAPARAZZI_HOME)/var/share/pprzlink/src/pprz_transport.c subsystems/datalink/telemetry.c

