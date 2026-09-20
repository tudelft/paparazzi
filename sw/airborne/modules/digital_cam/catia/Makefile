
PAPARAZZI_HOME ?= $(abspath ../../../../..)
LIBEXIF = $(PAPARAZZI_HOME)/sw/ext/libexif
LIBJPEG = $(PAPARAZZI_HOME)/sw/ext/opencv_bebop/opencv/3rdparty/libjpeg
CC = gcc
CXX = g++
OPTFLAGS ?= -O2
BUILD_DIR = .build
OUTPUT_SUFFIX ?=
CROSS_COMPILE ?= aarch64-linux-gnu-
MORA_INSTALL_DIR ?= /home/air/digital_cam
MORA_PHOTO_DIR ?= /home/air/Pictures
MORA_DEBUG_DIR ?= /home/air/usher_debug_data
ARM64_SYSROOT ?=
ARM64_PKG_CONFIG_LIBDIR ?= $(ARM64_SYSROOT)/usr/lib/aarch64-linux-gnu/pkgconfig:$(ARM64_SYSROOT)/usr/share/pkgconfig
CATIA_OUTPUT = catia$(OUTPUT_SUFFIX)
SODA_OUTPUT = soda$(OUTPUT_SUFFIX)
CAMERA_SUFFIX = $(if $(OUTPUT_SUFFIX),$(OUTPUT_SUFFIX),-native)

CATIA_SERIAL_DEVICE ?= /dev/ttySAC0
CATIA_MOCK_IMAGE ?= $(PAPARAZZI_HOME)/sw/airborne/modules/digital_cam/catia/mock_image_01.jpg
CATIA_SODA ?= $(PAPARAZZI_HOME)/sw/airborne/modules/digital_cam/catia/soda
CATIA_LOCAL_PHOTO_DIR ?= ../Pictures
CATIA_AI_CAM_PHOTO_DIR ?= $(PAPARAZZI_HOME)/sw/airborne/modules/digital_cam/catia/photos
CATIA_AI_CAM_COMMAND ?= rpicam-still
CATIA_VEHICLE_DETECT_COMMAND ?= $(MORA_INSTALL_DIR)/vehicle_detect_server.py
CATIA_VEHICLE_DETECT_MODEL ?= $(MORA_INSTALL_DIR)/models/vehicle/network.rpk
CATIA_VEHICLE_DETECT_LABELS ?= $(MORA_INSTALL_DIR)/models/vehicle/labels.txt
CATIA_HOME_VECTOR_COMMAND ?= $(MORA_INSTALL_DIR)/home_vector_server.py
CATIA_HOME_VECTOR_MODEL ?= $(MORA_INSTALL_DIR)/models/home_vector/network.rpk
CATIA_LWIR_CAM_PHOTO_DIR ?= $(PAPARAZZI_HOME)/sw/airborne/modules/digital_cam/catia/photos
CATIA_LWIR_CAM_COMMAND ?= $(CURDIR)/lwircam-native
CATIA_CHDK_PHOTO_DIR ?= $(PAPARAZZI_HOME)/sw/airborne/modules/digital_cam/catia/photos
CATIA_EAR_CAM_COMMAND ?= $(CURDIR)/earcam-native
CATIA_EAR_CAM_DEVICE ?= auto
CATIA_EAR_CAM_LOG_DIR ?= $(PAPARAZZI_HOME)/sw/airborne/modules/digital_cam/catia/earlogs
CATIA_EAR_CAM_PHOTO_DIR ?= $(PAPARAZZI_HOME)/sw/airborne/modules/digital_cam/catia/photos
CATIA_POSE_LOG_DIR ?=
NEAR_HORIZON_CASE_BLACK ?= 0
NEAR_HORIZON_ROLL_DEG ?= 75.0
NEAR_HORIZON_PITCH_DEG ?= 75.0
NEAR_HORIZON_COMBINED_TILT_DEG ?= 70.0
LWIR_ARCH ?= native
LWIR_STATIC ?= 1
LWIR_VIDEO_DISPLAY ?= 0
LWIR_OPENCV_ROOT ?= $(abspath $(BUILD_DIR))/lwir-opencv/install
CONFIG_STAMP = $(BUILD_DIR)/config.stamp
CATIA_GIT_SHA := $(shell git -C '$(CURDIR)' rev-parse --short=12 HEAD 2>/dev/null || printf unknown)

CPPFLAGS = -I$(PAPARAZZI_HOME)/sw/ext -I$(LIBEXIF) -I$(LIBJPEG) \
	-DNO_VERBOSE_TAG_DATA -DNO_VERBOSE_TAG_STRINGS -DCATIA_GIT_SHA='"$(CATIA_GIT_SHA)"'
CFLAGS = $(OPTFLAGS) -Wall -Wextra -Werror -Wno-expansion-to-defined \
	-DCATIA_SERIAL_DEVICE='"$(CATIA_SERIAL_DEVICE)"' \
	-DCATIA_MOCK_IMAGE='"$(CATIA_MOCK_IMAGE)"' \
	-DCATIA_SODA='"$(CATIA_SODA)"' \
	-DCATIA_LOCAL_PHOTO_DIR='"$(CATIA_LOCAL_PHOTO_DIR)"' \
	-DCATIA_AI_CAM_PHOTO_DIR='"$(CATIA_AI_CAM_PHOTO_DIR)"' \
	-DCATIA_AI_CAM_COMMAND='"$(CATIA_AI_CAM_COMMAND)"' \
	-DCATIA_VEHICLE_DETECT_COMMAND='"$(CATIA_VEHICLE_DETECT_COMMAND)"' \
	-DCATIA_VEHICLE_DETECT_MODEL='"$(CATIA_VEHICLE_DETECT_MODEL)"' \
	-DCATIA_VEHICLE_DETECT_LABELS='"$(CATIA_VEHICLE_DETECT_LABELS)"' \
	-DCATIA_HOME_VECTOR_COMMAND='"$(CATIA_HOME_VECTOR_COMMAND)"' \
	-DCATIA_HOME_VECTOR_MODEL='"$(CATIA_HOME_VECTOR_MODEL)"' \
	-DCATIA_LWIR_CAM_PHOTO_DIR='"$(CATIA_LWIR_CAM_PHOTO_DIR)"' \
	-DCATIA_LWIR_CAM_COMMAND='"$(CATIA_LWIR_CAM_COMMAND)"' \
	-DCATIA_CHDK_PHOTO_DIR='"$(CATIA_CHDK_PHOTO_DIR)"' \
	-DCATIA_EAR_CAM_COMMAND='"$(CATIA_EAR_CAM_COMMAND)"' \
	-DCATIA_EAR_CAM_DEVICE='"$(CATIA_EAR_CAM_DEVICE)"' \
	-DCATIA_EAR_CAM_LOG_DIR='"$(CATIA_EAR_CAM_LOG_DIR)"' \
	-DCATIA_EAR_CAM_PHOTO_DIR='"$(CATIA_EAR_CAM_PHOTO_DIR)"' \
	$(if $(CATIA_POSE_LOG_DIR),-DCATIA_POSE_LOG_DIR='"$(CATIA_POSE_LOG_DIR)"') \
	-Dnear_horizon_case_black=$(NEAR_HORIZON_CASE_BLACK) \
	-Dnear_horizon_roll_deg=$(NEAR_HORIZON_ROLL_DEG) \
	-Dnear_horizon_pitch_deg=$(NEAR_HORIZON_PITCH_DEG) \
	-Dnear_horizon_combined_tilt_deg=$(NEAR_HORIZON_COMBINED_TILT_DEG)
CXXFLAGS = $(OPTFLAGS) -Wall -Wextra -Werror
VENDOR_CFLAGS = $(OPTFLAGS) -w
DEPFLAGS = -MMD -MP
LDFLAGS += -static
LDLIBS = -lpthread -lm
CFILES = ai_cam_pipe.c \
	 vehicle_detect_pipe.c \
	 home_vector_pipe.c \
	 camera_speedtest.c \
	 ear_cam_pipe.c \
	 ear_heatmap.c \
	 lwir_cam_pipe.c \
	 serial.c \
	 serial_tx.c \
	 clock_alignment.c \
	 chdk_pipe.c \
	 image_exif.c \
	 image_mock_transform.c \
	 local_pipe.c \
	 socket.c \
	 pose_log.c \
	 protocol.c
HFILES = $(CFILES:.c=.h) version.h path_utils.h
PROJECT_CFILES = $(CFILES) catia.c
PROJECT_OBJECTS = $(addprefix $(BUILD_DIR)/project/,$(PROJECT_CFILES:.c=.o))
LIBEXIF_CFILES = $(addprefix $(LIBEXIF)/, \
	exif-content.c \
	exif-entry.c \
	exif-tag.c \
	exif-format.c \
	exif-utils.c \
	exif-mem.c \
	exif-data.c \
	exif-log.c \
	exif-loader.c \
	exif-ifd.c \
	exif-mnote-data.c)
LIBJPEG_CFILES = $(filter-out $(LIBJPEG)/jmemnobs.c,$(wildcard $(LIBJPEG)/*.c))
LIBEXIF_OBJECTS = $(patsubst $(LIBEXIF)/%.c,$(BUILD_DIR)/libexif/%.o,$(LIBEXIF_CFILES))
LIBJPEG_OBJECTS = $(patsubst $(LIBJPEG)/%.c,$(BUILD_DIR)/libjpeg/%.o,$(LIBJPEG_CFILES))


all: native arm64

native: catia soda lwircam earcam

architecture-build: $(CATIA_OUTPUT) $(SODA_OUTPUT) lwircam earcam

arm64:
	$(MAKE) architecture-build BUILD_DIR=.build/arm64 OUTPUT_SUFFIX=-arm64 \
	  CC=$(CROSS_COMPILE)gcc CXX=$(CROSS_COMPILE)g++ LWIR_ARCH=aarch64-gnu \
	  OPTFLAGS='$(OPTFLAGS) -g0' \
	  LWIR_STATIC=1 LWIR_VIDEO_DISPLAY=0 \
	  CATIA_SERIAL_DEVICE=/dev/serial0 \
	  CATIA_MOCK_IMAGE='$(MORA_INSTALL_DIR)/mock_image_01.jpg' \
	  CATIA_SODA='$(MORA_INSTALL_DIR)/soda' \
	  CATIA_LOCAL_PHOTO_DIR='$(MORA_PHOTO_DIR)' \
	  CATIA_AI_CAM_PHOTO_DIR='$(MORA_PHOTO_DIR)' \
	  CATIA_LWIR_CAM_PHOTO_DIR='$(MORA_PHOTO_DIR)' \
	  CATIA_LWIR_CAM_COMMAND='$(MORA_INSTALL_DIR)/lwircam' \
	  CATIA_CHDK_PHOTO_DIR='$(MORA_PHOTO_DIR)' \
	  CATIA_EAR_CAM_COMMAND='$(MORA_INSTALL_DIR)/earcam' \
	  CATIA_EAR_CAM_LOG_DIR='$(MORA_DEBUG_DIR)' \
	  CATIA_EAR_CAM_PHOTO_DIR='$(MORA_PHOTO_DIR)' \
	  CATIA_POSE_LOG_DIR='$(MORA_DEBUG_DIR)'
	$(CROSS_COMPILE)strip --strip-all catia-arm64 soda-arm64 lwircam-arm64 earcam-arm64

$(CATIA_OUTPUT): $(PROJECT_OBJECTS) $(LIBEXIF_OBJECTS) $(LIBJPEG_OBJECTS)
	$(CC) $(LDFLAGS) $^ -o $@ $(LDLIBS)

$(BUILD_DIR)/project/%.o: %.c $(HFILES) $(CONFIG_STAMP)
	@mkdir -p $(@D)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(DEPFLAGS) -c $< -o $@

$(BUILD_DIR)/libexif/%.o: $(LIBEXIF)/%.c $(CONFIG_STAMP)
	@mkdir -p $(@D)
	$(CC) $(CPPFLAGS) $(VENDOR_CFLAGS) $(DEPFLAGS) -c $< -o $@

$(BUILD_DIR)/libjpeg/%.o: $(LIBJPEG)/%.c $(CONFIG_STAMP)
	@mkdir -p $(@D)
	$(CC) $(CPPFLAGS) $(VENDOR_CFLAGS) $(DEPFLAGS) -c $< -o $@

$(CONFIG_STAMP): FORCE
	@mkdir -p $(@D)
	@printf '%s\n' \
	  'CC=$(CC)' \
	  'CXX=$(CXX)' \
	  'OPTFLAGS=$(OPTFLAGS)' \
	  'CPPFLAGS=$(CPPFLAGS)' \
	  'CFLAGS=$(CFLAGS)' \
	  'CXXFLAGS=$(CXXFLAGS)' \
	  'VENDOR_CFLAGS=$(VENDOR_CFLAGS)' \
	  'LDFLAGS=$(LDFLAGS)' \
	  'LDLIBS=$(LDLIBS)' \
	  'CATIA_SERIAL_DEVICE=$(CATIA_SERIAL_DEVICE)' \
	  'CATIA_MOCK_IMAGE=$(CATIA_MOCK_IMAGE)' \
	  'CATIA_SODA=$(CATIA_SODA)' \
	  'CATIA_LOCAL_PHOTO_DIR=$(CATIA_LOCAL_PHOTO_DIR)' \
	  'CATIA_AI_CAM_PHOTO_DIR=$(CATIA_AI_CAM_PHOTO_DIR)' \
	  'CATIA_AI_CAM_COMMAND=$(CATIA_AI_CAM_COMMAND)' \
	  'CATIA_VEHICLE_DETECT_COMMAND=$(CATIA_VEHICLE_DETECT_COMMAND)' \
	  'CATIA_VEHICLE_DETECT_MODEL=$(CATIA_VEHICLE_DETECT_MODEL)' \
	  'CATIA_VEHICLE_DETECT_LABELS=$(CATIA_VEHICLE_DETECT_LABELS)' \
	  'CATIA_HOME_VECTOR_COMMAND=$(CATIA_HOME_VECTOR_COMMAND)' \
	  'CATIA_HOME_VECTOR_MODEL=$(CATIA_HOME_VECTOR_MODEL)' \
	  'CATIA_LWIR_CAM_PHOTO_DIR=$(CATIA_LWIR_CAM_PHOTO_DIR)' \
	  'CATIA_LWIR_CAM_COMMAND=$(CATIA_LWIR_CAM_COMMAND)' \
	  'CATIA_CHDK_PHOTO_DIR=$(CATIA_CHDK_PHOTO_DIR)' \
	  'CATIA_EAR_CAM_COMMAND=$(CATIA_EAR_CAM_COMMAND)' \
	  'CATIA_EAR_CAM_DEVICE=$(CATIA_EAR_CAM_DEVICE)' \
	  'CATIA_EAR_CAM_LOG_DIR=$(CATIA_EAR_CAM_LOG_DIR)' \
	  'CATIA_EAR_CAM_PHOTO_DIR=$(CATIA_EAR_CAM_PHOTO_DIR)' \
	  'CATIA_POSE_LOG_DIR=$(CATIA_POSE_LOG_DIR)' \
	  'NEAR_HORIZON_CASE_BLACK=$(NEAR_HORIZON_CASE_BLACK)' \
	  'NEAR_HORIZON_ROLL_DEG=$(NEAR_HORIZON_ROLL_DEG)' \
	  'NEAR_HORIZON_PITCH_DEG=$(NEAR_HORIZON_PITCH_DEG)' \
	  'NEAR_HORIZON_COMBINED_TILT_DEG=$(NEAR_HORIZON_COMBINED_TILT_DEG)' > $@.tmp
	@cmp -s $@.tmp $@ || cp $@.tmp $@
	@rm -f $@.tmp

$(SODA_OUTPUT): soda.cpp version.h $(CONFIG_STAMP)
	$(CXX) $(CPPFLAGS) $(LDFLAGS) $(CXXFLAGS) soda.cpp -o $@

# Ground-side: re-render an earlogs/*.csv session into the heatmap JPEG + .geo sidecar.
ear_heatmap_replay: $(BUILD_DIR)/project/ear_heatmap_replay.o $(BUILD_DIR)/project/ear_cam_pipe.o \
		$(BUILD_DIR)/project/ear_heatmap.o $(LIBJPEG_OBJECTS)
	$(CC) $(LDFLAGS) $^ -o $@ $(LDLIBS)

lwircam: $(LIBJPEG_OBJECTS) $(LIBEXIF_OBJECTS) $(BUILD_DIR)/project/image_exif.o
	$(MAKE) -C lwircam lwircam \
	  ARCH=$(LWIR_ARCH) \
	  STATIC=$(LWIR_STATIC) \
	  VIDEO_DISPLAY=$(LWIR_VIDEO_DISPLAY) \
	  CXX=$(CXX) \
	  TARGET_OUT_DIR='$(CURDIR)' TARGET_NAME='lwircam$(CAMERA_SUFFIX)' \
	  OPTFLAGS='$(OPTFLAGS)' \
	  JPEG_INCLUDE='$(LIBJPEG)' \
	  JPEG_OBJECTS='$(addprefix $(abspath .)/,$(LIBJPEG_OBJECTS))' \
	  EXIF_INCLUDE='$(LIBEXIF)' \
	  EXIF_OBJECTS='$(addprefix $(abspath .)/,$(LIBEXIF_OBJECTS) $(BUILD_DIR)/project/image_exif.o)' \
	  OPENCV_ROOT='$(LWIR_OPENCV_ROOT)'

earcam:
	$(if $(OUTPUT_SUFFIX),PKG_CONFIG_SYSROOT_DIR='$(ARM64_SYSROOT)' PKG_CONFIG_LIBDIR='$(ARM64_PKG_CONFIG_LIBDIR)') \
	  $(MAKE) -C earcam all CC=$(CC) \
	  BUILD_ARCH=$(if $(OUTPUT_SUFFIX),arm64,native) \
	  TARGET='$(CURDIR)/earcam$(CAMERA_SUFFIX)' OPTFLAGS='$(OPTFLAGS)' LDFLAGS=

aicam chdkcam: $(CATIA_OUTPUT)
	@printf '%s\n' '$@ backend is compiled into $(CATIA_OUTPUT); its camera driver is external.'

documentation:
	./documentation/update-documentation.sh
	doxygen Doxyfile
	bash ./postprocess-doxygen-graphs.sh doxygen/html

clean:
	rm -rf ./catia ./soda ./catia-arm64 ./soda-arm64 ./lwircam-native ./lwircam-arm64 ./earcam-native ./earcam-arm64 ./ear_heatmap_replay $(BUILD_DIR) ./.near-horizon-*
	$(MAKE) -C lwircam clean
	$(MAKE) -C earcam clean

.PHONY: all native arm64 architecture-build clean documentation lwircam earcam aicam chdkcam FORCE

-include $(PROJECT_OBJECTS:.o=.d) $(LIBEXIF_OBJECTS:.o=.d) $(LIBJPEG_OBJECTS:.o=.d)
