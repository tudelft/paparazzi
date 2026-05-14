
#set(SOURCE
#    ${SOURCE}
#    ${CMAKE_CURRENT_SOURCE_DIR}/item_edit_state_machine.cpp
#    ${CMAKE_CURRENT_SOURCE_DIR}/waypointitem_sm.cpp
#    ${CMAKE_CURRENT_SOURCE_DIR}/circleitem_sm.cpp
#    ${CMAKE_CURRENT_SOURCE_DIR}/pathitem_sm.cpp
#)

set(INC_DIRS ${INC_DIRS} ${CMAKE_CURRENT_SOURCE_DIR})

set(SOURCE ${SOURCE}  PARENT_SCOPE)
set(INC_DIRS ${INC_DIRS}  PARENT_SCOPE)

