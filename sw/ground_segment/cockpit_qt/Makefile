.PHONY: all build clean run setup strip

include ../../../conf/Makefile.qt

# gcs is the native Qt6/C++ ground control station (CMake target "equinoxgcs",
# output name "gcs"), built through the shared Qt super-build defined in
# sw/CMakeLists.txt and driven via conf/Makefile.qt.
all: qt-configure
	$(Q)$(QT_CMAKE_BUILD) equinoxgcs

# Backwards-compatible alias for the shared configure step.
setup: qt-configure

build: all

clean: qt-clean
	$(Q)rm -f gcs

strip: all
	$(Q)strip -s "$(QT_BUILD_DIR)/cockpit_qt/gcs" || true

run: all
	$(Q)"$(QT_BUILD_DIR)/cockpit_qt/gcs"
