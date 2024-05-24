# specify allowed CMake versions
cmake_minimum_required(VERSION 3.4...3.18)

# create project
project(SdsDroneSdkPythonBindings LANGUAGES C CXX)

# un-comment the desired platform
set(PLATFORM "windows_gcc")
#set(PLATFORM "windows_msvc")
#set(PLATFORM "ubuntu")
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

include_directories("${CMAKE_SOURCE_DIR}/../include")
link_directories("${SHARED_LIB_DIR}")

add_subdirectory(pybind11)

pybind11_add_module(pySdsDroneSdk pySdsDroneSdk.cpp)
if (CMAKE_SYSTEM_NAME STREQUAL "Windows")
    set_target_properties(pySdsDroneSdk PROPERTIES SUFFIX ".pyd") # get rid of compiler details that were added
elseif (CMAKE_SYSTEM_NAME STREQUAL "Linux")
    set_target_properties(pySdsDroneSdk PROPERTIES SUFFIX ".so") # get rid of compiler details that were added
else()
    message(FATAL_ERROR "Unrecognized CMAKE_SYSTEM_NAME")
endif()
set_target_properties(pySdsDroneSdk PROPERTIES DEBUG_POSTFIX "") # get rid of _d for debug builds
target_link_libraries(pySdsDroneSdk PRIVATE "${SHARED_LIB_PATH}")
target_link_options(pySdsDroneSdk PUBLIC -static-libgcc -static-libstdc++ -Wl,-Bstatic,--whole-archive -lwinpthread -Wl,--no-whole-archive)