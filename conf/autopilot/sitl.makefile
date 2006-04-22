sim.ARCHDIR = $(ARCHI)
sim.ARCH = sitl
sim.TARGET = autopilot
sim.TARGETDIR = autopilot
sim.CFLAGS += -DSITL -DAP -DFBW -DRADIO_CONTROL -DINTER_MCU -DDOWNLINK -DDOWNLINK_TRANSPORT=IvyTransport -DINFRARED
sim.srcs = radio_control.c downlink.c commands.c gps.c inter_mcu.c link_mcu.c infrared.c pid.c nav.c estimator.c cam.c sys_time.c main_fbw.c main_ap.c $(SRC_ARCH)/ppm_hw.c $(SRC_ARCH)/sim_gps.c $(SRC_ARCH)/sim_ir.c $(SRC_ARCH)/sim_ap.c  $(SRC_ARCH)/ivy_transport.c 
