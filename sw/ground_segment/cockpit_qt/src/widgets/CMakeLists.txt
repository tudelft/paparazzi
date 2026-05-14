
set(SOURCE
    ${SOURCE}
    ${CMAKE_CURRENT_SOURCE_DIR}/strip.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ac_selector.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/settings_viewer.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/settings_classic_viewer.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/settings_classic_viewer.h
    ${CMAKE_CURRENT_SOURCE_DIR}/pprzmap.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/pfd.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/listcontainer.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/mini_strip.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/flightplan_viewerv2.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/stackcontainer.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/layer_combo.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/widget_utils.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/pprzmap.ui
    ${CMAKE_CURRENT_SOURCE_DIR}/commands.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/gps_classic_viewer.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/flightplaneditor.ui
    ${CMAKE_CURRENT_SOURCE_DIR}/flightplaneditor.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plotter.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/wind_sock.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/pancontrol.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/pancontrol.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/link_status.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/grid_viewer.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/gvf_viewer.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/chat.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/chat.ui
    ${CMAKE_CURRENT_SOURCE_DIR}/checklist.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/checklist.ui
)

set(INC_DIRS ${INC_DIRS} ${CMAKE_CURRENT_SOURCE_DIR})

add_subdirectory(map)
add_subdirectory(basics)

set(SOURCE ${SOURCE}  PARENT_SCOPE)
set(INC_DIRS ${INC_DIRS}  PARENT_SCOPE)
