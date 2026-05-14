
set(SOURCE
    ${SOURCE}
    ${CMAKE_CURRENT_SOURCE_DIR}/graphics_object.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/graphics_point.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/graphics_line.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/graphics_circle.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/graphics_aircraft.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/graphics_text.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/graphics_intruder.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/graphics_group.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/graphics_icon.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/graphics_quiver.cpp
    
)

set(INC_DIRS ${INC_DIRS} ${CMAKE_CURRENT_SOURCE_DIR})

set(SOURCE ${SOURCE}  PARENT_SCOPE)
set(INC_DIRS ${INC_DIRS}  PARENT_SCOPE)

