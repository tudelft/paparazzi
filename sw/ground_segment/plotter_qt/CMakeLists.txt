cmake_minimum_required(VERSION 3.16)
project(PaparazziPlotterQt VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Default to Release for single-config generators when no build type is provided.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
endif()

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets Xml Network Charts)

add_subdirectory(../cockpit_qt/ext/pprzlinkQt ${CMAKE_BINARY_DIR}/pprzlinkqt)

set(SRC_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/plotter.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/resources.qrc"
)

add_executable(plotter ${SRC_FILES})

target_include_directories(plotter PRIVATE .)

target_compile_options(plotter PRIVATE
  $<$<CONFIG:Release>:-O2 -DNDEBUG -fdata-sections -ffunction-sections -flto -g0>
)
target_link_options(plotter PRIVATE
  $<$<CONFIG:Release>:-Wl,--gc-sections -Wl,-O1 -Wl,-s -flto>
)

# Enable link-time optimization for the Release configuration.
set_target_properties(plotter PROPERTIES INTERPROCEDURAL_OPTIMIZATION_RELEASE TRUE)

find_program(STRIP_EXECUTABLE strip)
if(STRIP_EXECUTABLE)
  add_custom_command(TARGET plotter POST_BUILD
    COMMAND ${STRIP_EXECUTABLE} $<TARGET_FILE:plotter>
  )
endif()

target_link_libraries(plotter PRIVATE 
    Qt6::Core Qt6::Gui Qt6::Widgets Qt6::Xml Qt6::Network Qt6::Charts
    pprzlinkQt
)

# Copy the final executable to the logalizer directory
add_custom_command(TARGET plotter POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
        $<TARGET_FILE:plotter>
        ${CMAKE_CURRENT_SOURCE_DIR}/../../logalizer/
)

install(TARGETS plotter DESTINATION bin)