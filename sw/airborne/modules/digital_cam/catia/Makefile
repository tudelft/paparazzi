
PAPARAZZI_HOME ?= $(abspath ../../../../..)
LIBEXIF = $(PAPARAZZI_HOME)/sw/ext/libexif
LIBJPEG = $(PAPARAZZI_HOME)/sw/ext/opencv_bebop/opencv/3rdparty/libjpeg
CC = gcc
CXX = g++
OPTFLAGS ?= -O2
BUILD_DIR = .build

CATIA_SERIAL_DEVICE ?= /dev/ttySAC0
CATIA_MOCK_IMAGE ?= $(PAPARAZZI_HOME)/sw/airborne/modules/digital_cam/catia/mock_image_01.jpg
CATIA_LOCAL_SODA ?= $(PAPARAZZI_HOME)/sw/airborne/modules/digital_cam/catia/soda_local
CATIA_LOCAL_PHOTO_DIR ?= $(PAPARAZZI_HOME)/sw/airborne/modules/digital_cam/catia/photos
CATIA_AI_CAM_PHOTO_DIR ?= $(PAPARAZZI_HOME)/sw/airborne/modules/digital_cam/catia/photos
CATIA_AI_CAM_COMMAND ?= rpicam-still
CATIA_LWIR_CAM_PHOTO_DIR ?= $(PAPARAZZI_HOME)/sw/airborne/modules/digital_cam/catia/photos
CATIA_LWIR_CAM_COMMAND ?= $(PAPARAZZI_HOME)/sw/airborne/modules/digital_cam/catia/lwircam/sample
CATIA_CHDK_PHOTO_DIR ?= $(PAPARAZZI_HOME)/sw/airborne/modules/digital_cam/catia/photos
NEAR_HORIZON_CASE_BLACK ?= 0
NEAR_HORIZON_ROLL_DEG ?= 75.0
NEAR_HORIZON_PITCH_DEG ?= 75.0
NEAR_HORIZON_COMBINED_TILT_DEG ?= 70.0
LWIR_ARCH ?= native
LWIR_STATIC ?= 0
LWIR_VIDEO_DISPLAY ?= 1
LWIR_OPENCV_ROOT ?=
CONFIG_STAMP = $(BUILD_DIR)/config.stamp

CPPFLAGS = -I$(PAPARAZZI_HOME)/sw/ext -I$(LIBEXIF) -I$(LIBJPEG) \
	-DNO_VERBOSE_TAG_DATA -DNO_VERBOSE_TAG_STRINGS
CFLAGS = $(OPTFLAGS) -Wall -Wextra -Werror -Wno-expansion-to-defined \
	-DCATIA_SERIAL_DEVICE='"$(CATIA_SERIAL_DEVICE)"' \
	-DCATIA_MOCK_IMAGE='"$(CATIA_MOCK_IMAGE)"' \
	-DCATIA_LOCAL_SODA='"$(CATIA_LOCAL_SODA)"' \
	-DCATIA_LOCAL_PHOTO_DIR='"$(CATIA_LOCAL_PHOTO_DIR)"' \
	-DCATIA_AI_CAM_PHOTO_DIR='"$(CATIA_AI_CAM_PHOTO_DIR)"' \
	-DCATIA_AI_CAM_COMMAND='"$(CATIA_AI_CAM_COMMAND)"' \
	-DCATIA_LWIR_CAM_PHOTO_DIR='"$(CATIA_LWIR_CAM_PHOTO_DIR)"' \
	-DCATIA_LWIR_CAM_COMMAND='"$(CATIA_LWIR_CAM_COMMAND)"' \
	-DCATIA_CHDK_PHOTO_DIR='"$(CATIA_CHDK_PHOTO_DIR)"' \
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
	 lwir_cam_pipe.c \
	 serial.c \
	 chdk_pipe.c \
	 image_exif.c \
	 image_mock_transform.c \
	 local_pipe.c \
	 socket.c \
	 protocol.c
HFILES = $(CFILES:.c=.h)
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


all:	catia soda_local lwir_sample

catia:	$(PROJECT_OBJECTS) $(LIBEXIF_OBJECTS) $(LIBJPEG_OBJECTS)
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
	  'CATIA_LOCAL_SODA=$(CATIA_LOCAL_SODA)' \
	  'CATIA_LOCAL_PHOTO_DIR=$(CATIA_LOCAL_PHOTO_DIR)' \
	  'CATIA_AI_CAM_PHOTO_DIR=$(CATIA_AI_CAM_PHOTO_DIR)' \
	  'CATIA_AI_CAM_COMMAND=$(CATIA_AI_CAM_COMMAND)' \
	  'CATIA_LWIR_CAM_PHOTO_DIR=$(CATIA_LWIR_CAM_PHOTO_DIR)' \
	  'CATIA_LWIR_CAM_COMMAND=$(CATIA_LWIR_CAM_COMMAND)' \
	  'CATIA_CHDK_PHOTO_DIR=$(CATIA_CHDK_PHOTO_DIR)' \
	  'NEAR_HORIZON_CASE_BLACK=$(NEAR_HORIZON_CASE_BLACK)' \
	  'NEAR_HORIZON_ROLL_DEG=$(NEAR_HORIZON_ROLL_DEG)' \
	  'NEAR_HORIZON_PITCH_DEG=$(NEAR_HORIZON_PITCH_DEG)' \
	  'NEAR_HORIZON_COMBINED_TILT_DEG=$(NEAR_HORIZON_COMBINED_TILT_DEG)' > $@.tmp
	@cmp -s $@.tmp $@ || cp $@.tmp $@
	@rm -f $@.tmp

soda_local:	soda_local.cpp $(CONFIG_STAMP)
	$(CXX) $(LDFLAGS) $(CXXFLAGS) soda_local.cpp -o soda_local

lwir_sample: $(LIBJPEG_OBJECTS)
	$(MAKE) -C lwircam sample \
	  ARCH=$(LWIR_ARCH) \
	  STATIC=$(LWIR_STATIC) \
	  VIDEO_DISPLAY=$(LWIR_VIDEO_DISPLAY) \
	  CXX=$(CXX) \
	  OPTFLAGS='$(OPTFLAGS)' \
	  JPEG_INCLUDE='$(LIBJPEG)' \
	  JPEG_OBJECTS='$(addprefix $(abspath .)/,$(LIBJPEG_OBJECTS))' \
	  OPENCV_ROOT='$(LWIR_OPENCV_ROOT)'

documentation:
	./documentation/update-documentation.sh

clean:
	rm -rf ./catia ./soda_local $(BUILD_DIR) ./.near-horizon-*
	$(MAKE) -C lwircam clean

.PHONY: all clean documentation lwir_sample FORCE

-include $(PROJECT_OBJECTS:.o=.d) $(LIBEXIF_OBJECTS:.o=.d) $(LIBJPEG_OBJECTS:.o=.d)
