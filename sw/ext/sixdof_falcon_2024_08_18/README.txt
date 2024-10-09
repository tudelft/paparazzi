Copyright (c) 2017-2024 Sixdof Space All rights reserved. 
Proprietary and Confidential

This folder contains the Sixdof Falcon package: Falcon Studio and the SdsFalcon SDK.

To get started, review the "Sixdof Falcon User Guide" from the docs directory.

The contents include:

+---bin						- Shared libraries for supported platforms and Falcon Studio executables for Windows and Linux
|   +---jetson
|   |       libSdsFalcon.so
|   |
|   +---rpi32
|   |       libSdsFalcon.so
|   |
|   +---rpi64
|   |       libSdsFalcon.so
|   |
|   +---ubuntu
|   |       libSdsFalcon.so
|   |       sixdof_falcon_studio
|   |
|   +---windows_gcc
|   |       SdsFalcon.dll
|   |       sixdof_falcon_studio.exe
|   |
|   \---windows_msvc
|           SdsFalcon.dll
|           SdsFalcon.lib
|
+---doc						- Documentation for Falcon Studio and the SdsFalcon SDK
|       SdsFalcon API.pdf
|       Sixdof Falcon User Guide.pdf
|       Python Bindings.pdf
|       Build Examples.pdf
|       Sixdof Software License.pdf
|
+---examples					- C++ build examples
|       CMakeLists.txt
|       field_of_view_report.cpp
|       gyro_calibration.cpp
|       heartbeat.cpp
|       heartbeat_manager.cpp
|       relative_angle_tracking.cpp
|       relative_beacon_tracking.cpp
|       sixdof_tracking.cpp
|
+---include					- Header files
|       SdsFalcon.h
|
\---python_bindings				- Python bindings
        CMakeLists.txt
        pyFalcon.cpp
        falcon_test_bindings.py
        falcon_test_bindings_2.py


Version Information:

| Program                    | Version       | Release Date  |
|----------------------------|---------------|---------------|
| SdsFalcon SDK              | 1.6.0         | 2024-08-18    |
| Sixdof Falcon Studio       | 1.1.0         | 2024-08-18    |
| Sixdof Falcon User Guide   | 4             | 2024-08-18    |
| Python Bindings            | 2             | 2024-08-18    |
| Build Examples             | 4             | 2024-08-18    |



