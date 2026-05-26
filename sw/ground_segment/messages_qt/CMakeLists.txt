cmake_minimum_required(VERSION 3.16)
project(PaparazziMessagesQt VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Default to Release for single-config generators when no build type is provided.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
endif()

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets Xml Network)

add_subdirectory(../cockpit_qt/ext/pprzlinkQt ${CMAKE_BINARY_DIR}/pprzlinkqt)

set(SRC_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/messages.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/resources.qrc"
)

add_executable(messages ${SRC_FILES})

target_compile_options(messages PRIVATE
  $<$<CONFIG:Release>:-O2 -DNDEBUG -fdata-sections -ffunction-sections -flto -g0>
)
target_link_options(messages PRIVATE
  $<$<CONFIG:Release>:-Wl,--gc-sections -Wl,-O1 -Wl,-s -flto>
)

# Enable link-time optimization for the Release configuration.
set_target_properties(messages PROPERTIES INTERPROCEDURAL_OPTIMIZATION_RELEASE TRUE)

find_program(STRIP_EXECUTABLE strip)
if(STRIP_EXECUTABLE)
  add_custom_command(TARGET messages POST_BUILD
    COMMAND ${STRIP_EXECUTABLE} $<TARGET_FILE:messages>
  )
endif()

target_link_libraries(messages PRIVATE 
    Qt6::Core Qt6::Gui Qt6::Widgets Qt6::Xml Qt6::Network
    pprzlinkQt
)

# Copy the final executable to the tmtc directory
add_custom_command(TARGET messages POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_CURRENT_SOURCE_DIR}/../tmtc
    COMMAND ${CMAKE_COMMAND} -E copy
        $<TARGET_FILE:messages>
        ${CMAKE_CURRENT_SOURCE_DIR}/../tmtc/
)

install(TARGETS messages DESTINATION bin)
install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/assets/paparazzi-messages.desktop" DESTINATION share/applications)
install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/../../../../data/pictures/penguin_logo.svg"
        DESTINATION share/icons/hicolor/scalable/apps
        RENAME paparazzi-messages.svg)
