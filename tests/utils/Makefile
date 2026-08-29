# Copyright (C) 2014 Piotr Esden-Tempski
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
# along with paparazzi; see the file COPYING.  If not, see
# <http://www.gnu.org/licenses/>.

# The default is to produce a quiet echo of compilation commands
# Launch with "make Q=''" to get full echo

# Make sure all our environment is set properly in case we run make not from toplevel director.
Q ?= @

PAPARAZZI_SRC ?= $(shell pwd)/../..
ifeq ($(PAPARAZZI_HOME),)
PAPARAZZI_HOME=$(PAPARAZZI_SRC)
endif

# export the PAPARAZZI environment to sub-make
export PAPARAZZI_SRC
export PAPARAZZI_HOME

#####################################################
# If you add more test files you add their names here
TESTS = test_airspeed_sdp3x.run test_airspeed_sdp3x_driver.run test_airspeed_sdp3x_driver_bidirectional.run test_airspeed_sdp3x_driver_filter.run test_framed_ring_buffer.run test_mesh_clock.run test_mesh_mode_policy.run test_tcas_policy.run test_traffic_info_reclaim.run test_traffic_info_units.run

###################################################
# You should not need to touch the rest of the file

TEST_VERBOSE ?= 0
ifneq ($(TEST_VERBOSE), 0)
VERBOSE = --verbose
endif

FLAGS=-Wall -g -O0

all: test

build_tests: $(TESTS)

test: build_tests
	prove $(VERBOSE) --exec '' ./*.run

test_framed_ring_buffer.run: $(PAPARAZZI_SRC)/sw/airborne/utils/framed_ring_buffer.c

test_airspeed_sdp3x.run: test_airspeed_sdp3x.c \
	$(PAPARAZZI_SRC)/sw/airborne/modules/sensors/airspeed_sdp3x.h
	@echo BUILD $@
	$(Q)$(CC) $(FLAGS) -Wextra -Werror -fsanitize=undefined \
		-I$(PAPARAZZI_SRC)/sw/airborne -I$(PAPARAZZI_SRC)/sw/include $< -lm -o $@

test_airspeed_sdp3x_driver.run: test_airspeed_sdp3x_driver.c \
	$(PAPARAZZI_SRC)/sw/airborne/modules/sensors/airspeed_sdp3x.c
	@echo BUILD $@
	$(Q)$(CC) $(FLAGS) -Wextra -Werror -fsanitize=undefined \
		-I$(PAPARAZZI_SRC)/tests/utils/sdp3x_stubs \
		-I$(PAPARAZZI_SRC)/sw/airborne -I$(PAPARAZZI_SRC)/sw/include \
		-DPERIODIC_TELEMETRY=0 -DUSE_AIRSPEED=1 -DUSE_AIRSPEED_SDP3X=1 \
		-DUSE_AIRSPEED_LOWPASS_FILTER=0 -DSDP3X_SYNC_SEND=0 \
		-DSDP3X_PERIODIC_PERIOD=0.01f -DSDP3X_I2C_DEV=i2c2 \
		-DSDP3X_PRESSURE_SCALE=20 -DSDP3X_VERIFY_PRESSURE_SCALE=1 \
		-DSDP3X_DYNAMIC_PRESSURE_SWAPPED=TRUE \
		$^ -lm -o $@

test_airspeed_sdp3x_driver_bidirectional.run: test_airspeed_sdp3x_driver.c \
	$(PAPARAZZI_SRC)/sw/airborne/modules/sensors/airspeed_sdp3x.c
	@echo BUILD $@
	$(Q)$(CC) $(FLAGS) -Wextra -Werror -fsanitize=undefined \
		-I$(PAPARAZZI_SRC)/tests/utils/sdp3x_stubs \
		-I$(PAPARAZZI_SRC)/sw/airborne -I$(PAPARAZZI_SRC)/sw/include \
		-DPERIODIC_TELEMETRY=0 -DUSE_AIRSPEED=1 -DUSE_AIRSPEED_SDP3X=1 \
		-DUSE_AIRSPEED_LOWPASS_FILTER=0 -DSDP3X_SYNC_SEND=0 \
		-DSDP3X_PERIODIC_PERIOD=0.01f -DSDP3X_I2C_DEV=i2c2 \
		-DSDP3X_PRESSURE_SCALE=20 -DSDP3X_VERIFY_PRESSURE_SCALE=1 \
		-DSDP3X_DYNAMIC_PRESSURE_SWAPPED=TRUE -DSDP3X_ENABLE_BIDIRECTIONAL=TRUE \
		$^ -lm -o $@

test_airspeed_sdp3x_driver_filter.run: test_airspeed_sdp3x_driver.c \
	$(PAPARAZZI_SRC)/sw/airborne/modules/sensors/airspeed_sdp3x.c
	@echo BUILD $@
	$(Q)$(CC) $(FLAGS) -Wextra -Werror -fsanitize=undefined \
		-I$(PAPARAZZI_SRC)/tests/utils/sdp3x_stubs \
		-I$(PAPARAZZI_SRC)/sw/airborne -I$(PAPARAZZI_SRC)/sw/include \
		-DPERIODIC_TELEMETRY=0 -DUSE_AIRSPEED=1 -DUSE_AIRSPEED_SDP3X=1 \
		-DUSE_AIRSPEED_LOWPASS_FILTER=1 -DSDP3X_SYNC_SEND=0 \
		-DSDP3X_PERIODIC_PERIOD=0.01f -DSDP3X_I2C_DEV=i2c2 \
		-DSDP3X_PRESSURE_SCALE=20 -DSDP3X_VERIFY_PRESSURE_SCALE=1 \
		-DSDP3X_DYNAMIC_PRESSURE_SWAPPED=TRUE -DSDP3X_TEST_FILTER_TRANSIENT=1 \
		$^ -lm -o $@

test_tcas_policy.run: test_tcas_policy.c \
	$(PAPARAZZI_SRC)/sw/airborne/modules/multi/tcas_policy.h \
	$(PAPARAZZI_SRC)/sw/airborne/modules/multi/traffic_info_policy.h
	@echo BUILD $@
	$(Q)$(CC) $(FLAGS) -Wextra -Werror -I$(PAPARAZZI_SRC)/sw/airborne $< -o $@

test_mesh_clock.run: test_mesh_clock.c \
	$(PAPARAZZI_SRC)/sw/airborne/modules/multi/traffic_info_mesh_policy.h
	@echo BUILD $@
	$(Q)$(CC) $(FLAGS) -Wextra -Werror -I$(PAPARAZZI_SRC)/sw/airborne $< -o $@

test_mesh_mode_policy.run: test_mesh_mode_policy.c \
	$(PAPARAZZI_SRC)/sw/airborne/modules/multi/traffic_info_mesh_policy.h
	@echo BUILD $@
	$(Q)$(CC) $(FLAGS) -Wextra -Werror -I$(PAPARAZZI_SRC)/sw/airborne $< -o $@

test_traffic_info_units.run: test_traffic_info_units.c \
	$(PAPARAZZI_SRC)/sw/airborne/modules/multi/traffic_info_policy.h
	@echo BUILD $@
	$(Q)$(CC) $(FLAGS) -Wextra -Werror -fsanitize=undefined \
		-I$(PAPARAZZI_SRC)/sw/airborne $< -o $@

test_traffic_info_reclaim.run: test_traffic_info_reclaim.c \
	$(PAPARAZZI_SRC)/sw/airborne/modules/multi/traffic_info_policy.h
	@echo BUILD $@
	$(Q)$(CC) $(FLAGS) -Wextra -Werror -fsanitize=undefined \
		-I$(PAPARAZZI_SRC)/sw/airborne $< -o $@

%.run: %.c
	@echo BUILD $@
	$(Q)$(CC) $(FLAGS) -I$(PAPARAZZI_SRC)/sw/airborne/utils -I$(PAPARAZZI_SRC)/sw/include -I$(PAPARAZZI_SRC)/tests/common $(USER_CFLAGS) $(PAPARAZZI_SRC)/tests/common/tap.c $^ -o $@

clean:
	$(Q)rm -f $(TESTS)


.PHONY: build_tests test clean all
