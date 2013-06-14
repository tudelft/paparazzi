
# Udp telemetry

ap.CFLAGS += -DDOWNLINK -DDOWNLINK_DEVICE=Udp
ap.CFLAGS += -DDOWNLINK_TRANSPORT=PprzTransport -DDATALINK=UDP -DDefaultPeriodic='&telemetry_Main'
ap.srcs += subsystems/datalink/downlink.c subsystems/datalink/udp.c subsystems/datalink/pprz_transport.c subsystems/datalink/telemetry.c
ap.srcs += $(SRC_FIRMWARE)/datalink.c $(SRC_FIRMWARE)/telemetry.c
ap.srcs += fms/fms_network.c
