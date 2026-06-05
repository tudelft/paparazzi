cmake_minimum_required(VERSION 3.16)
project(PaparazziPlayQt VERSION 1.0 LANGUAGES CXX)

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

find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets Xml Network)

set(SRC_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/play.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/play.qrc"
)

add_executable(play ${SRC_FILES})

target_include_directories(play PRIVATE .)

target_compile_options(play PRIVATE
  $<$<CONFIG:Release>:-O2 -DNDEBUG -fdata-sections -ffunction-sections -flto -g0>
)
target_link_options(play PRIVATE
  $<$<CONFIG:Release>:-Wl,--gc-sections -Wl,-O1 -Wl,-s -flto>
)

# Enable link-time optimization for the Release configuration.
set_target_properties(play PROPERTIES INTERPROCEDURAL_OPTIMIZATION_RELEASE TRUE)

find_program(STRIP_EXECUTABLE strip)
if(STRIP_EXECUTABLE)
  add_custom_command(TARGET play POST_BUILD
    COMMAND ${STRIP_EXECUTABLE} $<TARGET_FILE:play>
  )
endif()

target_link_libraries(play PRIVATE 
    Qt6::Core Qt6::Gui Qt6::Widgets Qt6::Xml Qt6::Network
)

# Copy the final executable to the logalizer directory
add_custom_command(TARGET play POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
        $<TARGET_FILE:play>
        ${CMAKE_CURRENT_SOURCE_DIR}/../../logalizer/
)

install(TARGETS play DESTINATION bin)

target_link_libraries(play PRIVATE IvyQt)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../ground_segment/cockpit_qt/ext/pprzlinkQt pprzlinkQt)
target_link_libraries(play PRIVATE pprzlinkQt)
