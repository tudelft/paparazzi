
set(SOURCE ${SOURCE})

list(APPEND SOURCE
    ${CMAKE_CURRENT_SOURCE_DIR}/srtm_manager.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/coordinatestransform.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/dispatcher_ui.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/pprz_dispatcher.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/AircraftManager.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/units.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Particules.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Live.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Shapes.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Intruders.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/watcher.cpp
)

if(TRUE)
    list(APPEND SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/speaker.cpp)
endif()

if(${GRPC})
    list(APPEND SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/grpcconnector.cpp)
endif()

set(INC_DIRS ${INC_DIRS} ${CMAKE_CURRENT_SOURCE_DIR})

set(SOURCE ${SOURCE}  PARENT_SCOPE)
set(INC_DIRS ${INC_DIRS}  PARENT_SCOPE)
