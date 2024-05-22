cmake_minimum_required(VERSION 3.4...3.18)

project(SdsDroneSdkExamples LANGUAGES C CXX)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

message("CMAKE_CXX_COMPILER_ID: ${CMAKE_CXX_COMPILER_ID}")
message("CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")
message("CMAKE_SOURCE_DIR: ${CMAKE_SOURCE_DIR}")
message("CMAKE_SYSTEM: ${CMAKE_SYSTEM}")
message("CMAKE_SYSTEM_NAME: ${CMAKE_SYSTEM_NAME}")
message("CMAKE_SYSTEM_VERSION: ${CMAKE_SYSTEM_VERSION}")
message("CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")
message("CMAKE_HOST_SYSTEM: ${CMAKE_HOST_SYSTEM}")


# get drone_sdk relase directory based on system
set(SDS_RELEASE_DIR_NAME "UNKNOWN")
if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    if (CMAKE_CXX_COMPILER_ID MATCHES GNU)
        set(SDS_RELEASE_DIR_NAME "win/gcc")
    elseif (CMAKE_CXX_COMPILER_ID MATCHES MSVC)
        set(SDS_RELEASE_DIR_NAME "win/msvc")
    endif()
elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
        set(SDS_RELEASE_DIR_NAME "ux64")
    elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "armv7l")
        set(SDS_RELEASE_DIR_NAME "rpi")
    elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
        set(SDS_RELEASE_DIR_NAME "rpi64")
    else()
        message(FATAL_ERROR "Unrecognized CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")
    endif()
else()
    message(FATAL_ERROR "Unrecognized CMAKE_SYSTEM_NAME: ${CMAKE_SYSTEM_NAME}")
endif()
message("SDS_RELEASE_DIR_NAME: ${SDS_RELEASE_DIR_NAME}")

set(CMAKE_BUILD_RPATH $ORIGIN) # for Linux to search for .so libraries in the same folder as executable
include_directories("${CMAKE_SOURCE_DIR}/../include")
link_directories("${CMAKE_SOURCE_DIR}/../${SDS_RELEASE_DIR_NAME}")


add_executable(sixdof_tracking sixdof_tracking.cpp)
target_link_libraries(sixdof_tracking PUBLIC SdsDroneSdk)

add_executable(relative_beacon_tracking relative_beacon_tracking.cpp)
target_link_libraries(relative_beacon_tracking PUBLIC SdsDroneSdk)

add_executable(relative_angle_tracking relative_angle_tracking.cpp)
target_link_libraries(relative_angle_tracking PUBLIC SdsDroneSdk)

add_executable(field_of_view_report field_of_view_report.cpp)
target_link_libraries(field_of_view_report PUBLIC SdsDroneSdk)

add_executable(gyro_calibration gyro_calibration.cpp)
target_link_libraries(gyro_calibration PUBLIC SdsDroneSdk)

add_executable(heartbeat heartbeat.cpp)
target_link_libraries(heartbeat PUBLIC SdsDroneSdk)

add_executable(heartbeat_manager heartbeat_manager.cpp)
target_link_libraries(heartbeat_manager PUBLIC SdsDroneSdk)

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