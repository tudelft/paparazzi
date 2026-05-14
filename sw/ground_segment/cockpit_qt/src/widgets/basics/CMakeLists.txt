
set(SOURCE
    ${SOURCE}
    ${CMAKE_CURRENT_SOURCE_DIR}/imagebutton.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/double_slider.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/switch.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/graphlabel.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/jaugelabel.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/colorlabel.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/flowlayout.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/mbpushbutton.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/lock_button.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/graphwidget.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/colorbar.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/chatbubble.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/chatbubble.ui
)

set(INC_DIRS ${INC_DIRS} ${CMAKE_CURRENT_SOURCE_DIR})

set(SOURCE ${SOURCE}  PARENT_SCOPE)
set(INC_DIRS ${INC_DIRS}  PARENT_SCOPE)
