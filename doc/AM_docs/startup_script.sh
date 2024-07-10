#!/bin/bash
sleep 2
cd /home/orangepi/paparazzi/sw/ext/sds_software_suite_2024_04_09/examples/
sudo ./custom_mode_am &
sleep 2
cd /home/orangepi/paparazzi/sw/ext/nonlinear_CA/
sudo ./Orangepi_PPZ_run &

#cd /home/orangepi/paparazzi/sw/ext/aruco_detection/
#python3 ArucoMarker_Complete_V4_RealTime.py
