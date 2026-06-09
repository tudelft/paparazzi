cmake_minimum_required(VERSION 3.16)
project(PaparazziQtSuperBuild LANGUAGES C CXX)

# ---------------------------------------------------------------------------
# Aggregating ("super") build for every native Qt6/C++ ground-segment & tool
# application shipped with Paparazzi:
#
#     ground_segment/cockpit_qt   -> gcs       (equinoxgcs target)
#     ground_segment/tmtc         -> messages
#     logalizer                   -> plotter, logplotter, play
#     simulator                   -> gaia
#
# This file lives at the root of the source tree it governs (sw/), which is the
# idiomatic CMake layout: every add_subdirectory() below is a plain, in-tree
# relative path -- no "../" reaching out of the listfile's own directory
#
# All applications are configured into a SINGLE build tree so the shared
# libraries (pprzlinkQt and IvyQt) are configured and compiled exactly once and
# then linked by every consumer. The canonical entry points are the per-app
# Makefiles, which delegate here through conf/Makefile.qt; you can also drive it
# directly, e.g.:
#
#     cmake -B var/build_qt -S sw -G Ninja -DCMAKE_BUILD_TYPE=Release
#     cmake --build var/build_qt --target gcs messages gaia plotter logplotter play
# ---------------------------------------------------------------------------

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
    set(CMAKE_CXX_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
    set(CMAKE_C_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
endif()

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -DNDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -DNDEBUG")
set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -s")

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang|AppleClang")
    add_compile_options($<$<CONFIG:Release>:-march=native>)
endif()

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets Xml Network Svg Charts)

# One central PprzlinkQt compilation (this also builds the IvyQt library that
# pprzlinkQt depends on).
add_subdirectory(ground_segment/cockpit_qt/ext/pprzlinkQt ${CMAKE_BINARY_DIR}/pprzlinkqt)

# ---------------------------------------------------------------------------
# logalizer tools: plotter, logplotter, play
#
# These are single-translation-unit Qt6 apps that share one optimized build
# recipe (Release LTO + section garbage collection + link-time strip) and are
# emitted next to their own sources in sw/logalizer/. They are declared directly
# here -- rather than via a near-empty sw/logalizer/CMakeLists.txt -- because the
# toolchain, Qt6 packages, AUTOMOC/RCC/UIC and the pprzlinkQt/IvyQt libraries are
# already set up above. Only their link libraries differ, so the shared recipe
# lives in the loop and the libraries are attached afterwards.
# ---------------------------------------------------------------------------
foreach(app plotter logplotter play)
    add_executable(${app}
        "logalizer/${app}.cpp"
        "logalizer/${app}.qrc"
    )
    target_include_directories(${app} PRIVATE logalizer ground_segment)
    target_compile_options(${app} PRIVATE
        $<$<CONFIG:Release>:-O2 -DNDEBUG -fdata-sections -ffunction-sections -flto -g0>
    )
    target_link_options(${app} PRIVATE
        $<$<CONFIG:Release>:-Wl,--gc-sections -Wl,-O1 -Wl,-s -flto>
    )
    set_target_properties(${app} PROPERTIES
        INTERPROCEDURAL_OPTIMIZATION_RELEASE TRUE
        # Emit the executable beside its sources (e.g. sw/logalizer/play).
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/logalizer"
    )
    install(TARGETS ${app} DESTINATION bin)
endforeach()

# plotter and logplotter render with Qt Charts and decode telemetry via pprzlinkQt.
target_link_libraries(plotter    PRIVATE Qt6::Core Qt6::Gui Qt6::Widgets Qt6::Xml Qt6::Network Qt6::Charts pprzlinkQt)
target_link_libraries(logplotter PRIVATE Qt6::Core Qt6::Gui Qt6::Widgets Qt6::Xml Qt6::Network Qt6::Charts pprzlinkQt)
# play only re-broadcasts already-formatted log lines on the Ivy bus (no Charts,
# no pprzlink): it reads the few message facts it needs straight from
# messages.xml via Qt6::Xml, so it links just IvyQt.
target_link_libraries(play       PRIVATE Qt6::Core Qt6::Gui Qt6::Widgets Qt6::Xml Qt6::Network IvyQt)

# ---------------------------------------------------------------------------
# Remaining Qt applications. Explicit (short) binary directory names keep the
# build tree flat and predictable: each app's executable lands in
# $(QT_BUILD_DIR)/<name>/ regardless of where its sources live under sw/.
# ---------------------------------------------------------------------------
add_subdirectory(ground_segment/tmtc tmtc)
add_subdirectory(ground_segment/cockpit_qt cockpit_qt)
add_subdirectory(simulator simulator)
