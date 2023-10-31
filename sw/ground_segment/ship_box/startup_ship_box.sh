#!/bin/bash

cd /home/orangepi/

#run pprzlink to reroute the pixhawk serial on udp under the 4342/4343 port: 
./paparazzi/sw/misc/link/pprzlink_router_cc -e uart:///dev/ttyS0:460800 -e udp://0.0.0.0:4343:192.168.42.255:4342 &

#forward the RTCM message from the GPS 1 to the wireless network under the 4442/4443 port:
./paparazzi/sw/misc/link/pprzlink_router_cc -e uart:///dev/ttyS1:460800 -e udp://0.0.0.0:4443:192.168.42.255:4442 &

#forward the messages from the GPS 2 to the wireless network under the 4542/4543 port:
./paparazzi/sw/misc/link/pprzlink_router_cc -e uart:///dev/ttyS3:460800 -e udp://0.0.0.0:4543:192.168.42.255:4542 &
