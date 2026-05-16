cmake_minimum_required(VERSION 3.16)
project(PaparazziPlotterQt VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets Xml Network Charts)

add_subdirectory(../cockpit_qt/ext/pprzlinkQt ${CMAKE_BINARY_DIR}/pprzlinkqt)

file(GLOB_RECURSE SRC_FILES "src/*.cpp")

add_executable(plotter_qt ${SRC_FILES})

target_include_directories(plotter_qt PRIVATE src)

target_link_libraries(plotter_qt PRIVATE 
    Qt6::Core Qt6::Gui Qt6::Widgets Qt6::Xml Qt6::Network Qt6::Charts
    pprzlinkQt
)

install(TARGETS plotter_qt DESTINATION bin)
set_target_properties(plotter_qt PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR})