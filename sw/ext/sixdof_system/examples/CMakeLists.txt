# specify allowed CMake versions
cmake_minimum_required(VERSION 3.4...3.18)

# create project
project(SdsDroneSdkExamples LANGUAGES C CXX)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# print system diagnostic info
message("CMAKE_CXX_COMPILER_ID: ${CMAKE_CXX_COMPILER_ID}")
message("CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")
message("CMAKE_SOURCE_DIR: ${CMAKE_SOURCE_DIR}")
message("CMAKE_SYSTEM: ${CMAKE_SYSTEM}")
message("CMAKE_SYSTEM_NAME: ${CMAKE_SYSTEM_NAME}")
message("CMAKE_SYSTEM_VERSION: ${CMAKE_SYSTEM_VERSION}")
message("CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")
message("CMAKE_HOST_SYSTEM: ${CMAKE_HOST_SYSTEM}")
message("CMAKE_HOST_SYSTEM_NAME: ${CMAKE_HOST_SYSTEM_NAME}")
message("CMAKE_HOST_SYSTEM_PROCESSOR: ${CMAKE_HOST_SYSTEM_PROCESSOR}")
message("CMAKE_HOST_SYSTEM_VERSION: ${CMAKE_HOST_SYSTEM_VERSION}")
message("CMAKE_USERNAME: ${CMAKE_USERNAME}")


# un-comment the desired platform
#set(PLATFORM "windows_gcc")
#set(PLATFORM "windows_msvc")
set(PLATFORM "ubuntu")
#set(PLATFORM "rpi32")
#set(PLATFORM "rpi64")
#set(PLATFORM "jetson")


if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    set(SHARED_LIB_FILE_NAME "SdsDroneSdk.dll")
elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    set(SHARED_LIB_FILE_NAME "libSdsDroneSdk.so")
else()
    message(FATAL_ERROR "Unrecognized CMAKE_SYSTEM_NAME: ${CMAKE_SYSTEM_NAME}")
endif()

message("SELECTED PLATFORM: ${PLATFORM}")

set(SHARED_LIB_DIR "${CMAKE_SOURCE_DIR}/../bin/${PLATFORM}")
set(SHARED_LIB_PATH "${SHARED_LIB_DIR}/${SHARED_LIB_FILE_NAME}")

# move DLL to build folder
file(COPY "${SHARED_LIB_PATH}" DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

set(CMAKE_BUILD_RPATH $ORIGIN)
include_directories("${CMAKE_SOURCE_DIR}/../include")
link_directories("${SHARED_LIB_DIR}")

add_executable(sixdof_tracking sixdof_tracking.cpp)
target_link_libraries(sixdof_tracking PRIVATE SdsDroneSdk)

add_executable(relative_beacon_tracking relative_beacon_tracking.cpp)
target_link_libraries(relative_beacon_tracking PRIVATE SdsDroneSdk)

add_executable(relative_angle_tracking relative_angle_tracking.cpp)
target_link_libraries(relative_angle_tracking PRIVATE SdsDroneSdk)

add_executable(field_of_view_report field_of_view_report.cpp)
target_link_libraries(field_of_view_report PRIVATE SdsDroneSdk)

add_executable(gyro_calibration gyro_calibration.cpp)
target_link_libraries(gyro_calibration PRIVATE SdsDroneSdk)

add_executable(heartbeat heartbeat.cpp)
target_link_libraries(heartbeat PRIVATE SdsDroneSdk)

add_executable(heartbeat_manager heartbeat_manager.cpp)
target_link_libraries(heartbeat_manager PRIVATE SdsDroneSdk)

if (CMAKE_SYSTEM_NAME STREQUAL "Windows")
    if (CMAKE_CXX_COMPILER_ID MATCHES GNU)
        # this is needed so it can be run on computers that dont have mingw dlls in their PATH
        target_link_options(sixdof_tracking PRIVATE -static-libgcc -static-libstdc++)
        target_link_options(relative_beacon_tracking PRIVATE -static-libgcc -static-libstdc++)
        target_link_options(relative_angle_tracking PRIVATE -static-libgcc -static-libstdc++)
        target_link_options(field_of_view_report PRIVATE -static-libgcc -static-libstdc++)
        target_link_options(gyro_calibration PRIVATE -static-libgcc -static-libstdc++)
        target_link_options(heartbeat PRIVATE -static-libgcc -static-libstdc++)
        target_link_options(heartbeat_manager PRIVATE -static-libgcc -static-libstdc++)
    endif()
elseif (CMAKE_SYSTEM_NAME STREQUAL "Linux")
	target_link_options(sixdof_tracking PRIVATE -static-libgcc -lstdc++ -pthread)
	target_link_options(relative_beacon_tracking PRIVATE -static-libgcc -lstdc++ -pthread)
	target_link_options(relative_angle_tracking PRIVATE -static-libgcc -lstdc++ -pthread)
	target_link_options(field_of_view_report PRIVATE -static-libgcc -lstdc++ -pthread)
	target_link_options(gyro_calibration PRIVATE -static-libgcc -lstdc++ -pthread)
	target_link_options(heartbeat PRIVATE -static-libgcc -lstdc++ -pthread)
	target_link_options(heartbeat_manager PRIVATE -static-libgcc -lstdc++ -pthread)
else()
    message(FATAL_ERROR "Unrecognized CMAKE_SYSTEM_NAME")
endif()
