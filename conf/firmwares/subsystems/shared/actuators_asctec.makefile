# asctec controllers
#
# required xml configuration:
#
#  servo section with driver="Asctec"
#  command_laws section to map motor_mixing commands to servos
#

$(TARGET).CFLAGS += -DACTUATORS
ACTUATORS_ASCTEC_SRCS = subsystems/actuators/actuators_asctec.c


# set default i2c device if not already configured
ifeq ($(ARCH), lpc21)
ACTUATORS_ASCTEC_I2C_DEV ?= i2c0
else ifeq ($(ARCH), stm32)
ACTUATORS_ASCTEC_I2C_DEV ?= i2c1
endif

ifndef ACTUATORS_ASCTEC_I2C_DEV
$(error Error: ACTUATORS_ASCTEC_I2C_DEV not configured!)
endif

# convert i2cx to upper/lower case
ACTUATORS_ASCTEC_I2C_DEV_UPPER=$(shell echo $(ACTUATORS_ASCTEC_I2C_DEV) | tr a-z A-Z)
ACTUATORS_ASCTEC_I2C_DEV_LOWER=$(shell echo $(ACTUATORS_ASCTEC_I2C_DEV) | tr A-Z a-z)

ACTUATORS_ASCTEC_CFLAGS += -DACTUATORS_ASCTEC_I2C_DEV=$(ACTUATORS_ASCTEC_I2C_DEV_LOWER)
ACTUATORS_ASCTEC_CFLAGS += -DUSE_$(ACTUATORS_ASCTEC_I2C_DEV_UPPER)

ifeq ($(ARCH), lpc21)
# set default i2c timing if not already configured
ACTUATORS_ASCTEC_I2C_SCL_TIME ?= 150
ACTUATORS_ASCTEC_CFLAGS += -D$(ACTUATORS_ASCTEC_I2C_DEV_UPPER)_SCLL=$(ACTUATORS_ASCTEC_I2C_SCL_TIME)
ACTUATORS_ASCTEC_CFLAGS += -D$(ACTUATORS_ASCTEC_I2C_DEV_UPPER)_SCLH=$(ACTUATORS_ASCTEC_I2C_SCL_TIME)
endif

ap.CFLAGS += $(ACTUATORS_ASCTEC_CFLAGS)
ap.srcs   += $(ACTUATORS_ASCTEC_SRCS)


# Simulator
nps.srcs += subsystems/actuators/actuators_asctec.c
nps.CFLAGS += -DUSE_I2C0 -DACTUATORS_ASCTEC_I2C_DEV=i2c0

