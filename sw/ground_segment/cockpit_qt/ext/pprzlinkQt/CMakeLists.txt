cmake_minimum_required(VERSION 3.14)

project(pprzlinkQt LANGUAGES CXX)

include(GNUInstallDirs)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

if(DEFINED ENV{QT_DIR})
    set(Qt6_DIR $ENV{QT_DIR})
endif()
find_package(QT NAMES Qt6 COMPONENTS Core Xml REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS
    Core
    Xml
    Network
    REQUIRED)

add_subdirectory(IvyQt ${CMAKE_BINARY_DIR}/ivyqt)

set(SOURCES 
    src/FieldValue.cpp
    src/Message.cpp
    src/MessageDefinition.cpp
    src/MessageDictionary.cpp
    src/MessageField.cpp
    src/MessageFieldTypes.cpp
    src/IvyQtLink.cpp
    include/pprzlinkQt/IvyQtLink.h
)    


add_library(pprzlinkQt ${SOURCES})

target_include_directories(
    ${PROJECT_NAME}
    PRIVATE
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
           $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

set(LINK_LIBRARIES 
    Qt${QT_VERSION_MAJOR}::Core
    Qt${QT_VERSION_MAJOR}::Xml
    Qt${QT_VERSION_MAJOR}::Network
    IvyQt
)

include_directories(${CMAKE_PREFIX_PATH}/include)
link_directories(${CMAKE_PREFIX_PATH}/lib)

target_link_libraries(pprzlinkQt PRIVATE ${LINK_LIBRARIES})

target_compile_definitions(pprzlinkQt PRIVATE PPRZLINKQT_LIBRARY)

install(TARGETS pprzlinkQt
        EXPORT ${PROJECT_NAME}Config 
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)


install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/include/${PROJECT_NAME}/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME})

export(TARGETS
        pprzlinkQt
        FILE "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
)

install(EXPORT ${PROJECT_NAME}Config DESTINATION "${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/cmake")

