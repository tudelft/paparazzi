cmake_minimum_required(VERSION 3.16)
project(PaparazziCockpitQt VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -DNDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -DNDEBUG")
set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -s")

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang|AppleClang")
    add_compile_options($<$<CONFIG:Release>:-march=native>)
endif()

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets Xml Network Svg)

find_library(PROJ proj)
if(${PROJ} STREQUAL "PROJ-NOTFOUND")
    find_package(PROJ4 REQUIRED)
else()
    set(PROJ4_LIBRARIES proj)
endif()
find_package(LibXml2 REQUIRED)

add_subdirectory(ext/pprzlinkQt ${CMAKE_BINARY_DIR}/pprzlinkqt)

file(GLOB_RECURSE SRC_FILES 
    "src/*.cpp"
    "src/*.c"
)
list(FILTER SRC_FILES EXCLUDE REGEX ".*/test_[^/]+\\.cpp$")
file(GLOB_RECURSE PROTOS "protos/*.cpp") 

add_executable(equinoxgcs
    ${SRC_FILES}
    ${PROTOS}
    resources/ressources.qrc
)

set_target_properties(equinoxgcs PROPERTIES
    OUTPUT_NAME gcs
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}
)

target_include_directories(equinoxgcs PRIVATE 
    src
    src/common
    src/geometry
    src/tools
    src/widgets
    src/widgets/basics
    src/widgets/map
    src/widgets/map/map_items
    src/widgets/map/graphics_objects
    src/widgets/map/gvf_trajectories
    src/widgets/map/fpedit_statemachines
    src/widgets/map/obstacle_points
    ${LIBXML2_INCLUDE_DIR}
)

target_link_libraries(equinoxgcs PRIVATE 
    Qt6::Core Qt6::Gui Qt6::Widgets Qt6::Xml Qt6::Network Qt6::Svg 
    pprzlinkQt
    ${PROJ4_LIBRARIES}
    ${LIBXML2_LIBRARIES}
    zip
)

# Copy data directory to build folder
add_custom_command(TARGET equinoxgcs POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_CURRENT_SOURCE_DIR}/data
        ${CMAKE_CURRENT_BINARY_DIR}/data
)

install(TARGETS equinoxgcs DESTINATION bin)
install(DIRECTORY data/ DESTINATION bin/data)
install(FILES assets/org.openuas.equinoxgcs.desktop DESTINATION share/applications)
install(FILES assets/equinoxgcs.svg DESTINATION share/icons/hicolor/scalable/apps)
