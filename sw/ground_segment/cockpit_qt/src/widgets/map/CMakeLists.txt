
set(SOURCE
    ${SOURCE}
    ${CMAKE_CURRENT_SOURCE_DIR}/tileprovider.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/tileproviderconfig.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/tileitem.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/map2d.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/mapwidget.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/maplayercontrol.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/mapscene.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/maputils.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/waypointeditor.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/acitemmanager.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/papget.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/papgetconfig.cpp
)

add_subdirectory(map_items)
add_subdirectory(graphics_objects)
add_subdirectory(fpedit_statemachines)
add_subdirectory(gvf_trajectories)

set(INC_DIRS ${INC_DIRS} ${CMAKE_CURRENT_SOURCE_DIR})

set(SOURCE ${SOURCE}  PARENT_SCOPE)
set(INC_DIRS ${INC_DIRS}  PARENT_SCOPE)
