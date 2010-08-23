# UBlox LEA 5H


ap.CFLAGS += -DGPS -DUBX -DGPS_USE_LATLONG
ap.CFLAGS += -DGPS_LINK=Uart$(GPS_UART_NR)
ap.CFLAGS += -DUSE_UART$(GPS_UART_NR)
ap.CFLAGS += -DUART$(GPS_UART_NR)_BAUD=$(GPS_BAUD)

ap.srcs   += $(SRC_FIXEDWING)/gps_ubx.c $(SRC_FIXEDWING)/gps.c $(SRC_FIXEDWING)/latlong.c
