
set(SOURCE
    ${SOURCE}
    ${CMAKE_CURRENT_SOURCE_DIR}/map_item.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/waypoint_item.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/path_item.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/circle_item.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/aircraft_item.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/intruder_item.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/arrow_item.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/quiver_item.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/grid_item.cpp
)

set(INC_DIRS ${INC_DIRS} ${CMAKE_CURRENT_SOURCE_DIR})

set(SOURCE ${SOURCE}  PARENT_SCOPE)
set(INC_DIRS ${INC_DIRS}  PARENT_SCOPE)

    ${CMAKE_CURRENT_SOURCE_DIR}/particule_item.cpp
