
set(SOURCE
    ${SOURCE}
    ${CMAKE_CURRENT_SOURCE_DIR}/aircraft.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/waypoint.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/sector.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/block.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/flightplan.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/setting_menu.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/setting.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/airframe.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/aircraft_status.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/gcs_utils.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/pprzpalette.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/aircraft_watcher.cpp
)

set(INC_DIRS ${INC_DIRS} ${CMAKE_CURRENT_SOURCE_DIR})

set(SOURCE ${SOURCE}  PARENT_SCOPE)
set(INC_DIRS ${INC_DIRS}  PARENT_SCOPE)
