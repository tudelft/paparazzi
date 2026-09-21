cmake_minimum_required(VERSION 3.16)
project(PaparazziGaiaQt VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
endif()

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets Xml Network)


set(SRC_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/gaia.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/gaia.qrc"
)

add_executable(gaia ${SRC_FILES})
target_include_directories(gaia PRIVATE .)

target_compile_options(gaia PRIVATE
  # -ffat-lto-objects: ld.lld cannot consume GCC's slim (IR-only) LTO
  # objects, which is what -flto produces by default on toolchains built
  # with linker-plugin support (as Ubuntu's GCC is). Fat objects carry real
  # machine code alongside the IR, so lld can link them directly without
  # the plugin.
  $<$<CONFIG:Release>:-O2 -DNDEBUG -fdata-sections -ffunction-sections -flto=auto -ffat-lto-objects -g0>
)
target_link_options(gaia PRIVATE
  $<$<CONFIG:Release>:-Wl,--gc-sections -Wl,-O1 -Wl,-s -flto=auto>
)

find_program(STRIP_EXECUTABLE strip)
if(STRIP_EXECUTABLE)
  add_custom_command(TARGET gaia POST_BUILD
    COMMAND ${STRIP_EXECUTABLE} $<TARGET_FILE:gaia>
  )
endif()

target_link_libraries(gaia PRIVATE 
    Qt6::Core Qt6::Gui Qt6::Widgets Qt6::Xml Qt6::Network
    pprzlinkQt
    pprz_version
)

# Copy the final executable to the simulator directory
add_custom_command(TARGET gaia POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_CURRENT_SOURCE_DIR}/
    COMMAND ${CMAKE_COMMAND} -E copy
        $<TARGET_FILE:gaia>
        ${CMAKE_CURRENT_SOURCE_DIR}/
)

install(TARGETS gaia DESTINATION bin)
