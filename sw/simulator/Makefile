# Hey Emacs, this is a -*- makefile -*-
#
# Copyright (C) 2003-2006  Pascal Brisset, Antoine Drouin
#
# This file is part of paparazzi.
#
# paparazzi is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2, or (at your option)
# any later version.
#
# paparazzi is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with paparazzi; see the file COPYING.  If not, write to
# the Free Software Foundation, 59 Temple Place - Suite 330,
# Boston, MA 02111-1307, USA.

# Quiet compilation
Q=@

include ../../conf/Makefile.local

BUILD_TYPE ?= Release
BUILD_DIR ?= ../../var/build_gaia

all : gaia

gaia :
	@echo "Building Gaia (C++/Qt) via CMake"
	@if [ ! -f "../../var/build_qt_super/CMakeCache.txt" ]; then \
		if command -v ninja >/dev/null 2>&1; then \
			cmake -B ../../var/build_qt_super -S ../../sw/qt_superbuild -G Ninja -DCMAKE_BUILD_TYPE=$(BUILD_TYPE); \
		else \
			cmake -B ../../var/build_qt_super -S ../../sw/qt_superbuild -DCMAKE_BUILD_TYPE=$(BUILD_TYPE); \
		fi \
	fi
	cmake --build ../../var/build_qt_super --config $(BUILD_TYPE) --target gaia --parallel $$(nproc)

clean :
	$(Q)rm -f *~ *.out .depend *.o *.a gaia
	$(Q)rm -rf $(BUILD_DIR)

.PHONY: all clean gaia
