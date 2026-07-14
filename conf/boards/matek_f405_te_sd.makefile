# Hey Emacs, this is a -*- makefile -*-
#
# matek_f405_te_sd.makefile
#
# For a Mateksys F405-TE-SD or compatible flightcontroller like FlyingRC F4WSE MK1.5
# only compatible with ChibiOS

BOARD=mateksys
BOARD_VERSION=F405-TE-SD
BOARD_DIR=$(BOARD)/$(BOARD_VERSION)
BOARD_CFG=\"boards/$(BOARD_DIR)/matek$(BOARD_VERSION).h\"

ARCH=chibios
$(TARGET).ARCHDIR = $(ARCH)

RTOS=chibios

## FPU on F4
USE_FPU=hard
USE_FPU_OPT=-mfloat-abi=hard -mfpu=fpv4-sp-d16

#USE_LTO ?= yes

$(TARGET).CFLAGS += -DPPRZLINK_ENABLE_FD

########################################################################
# Architecture or project specific options
#
# Define project name here (target)
PROJECT = $(TARGET)

# Project specific files and paths (see Makefile.chibios for details)
CHIBIOS_BOARD_PLATFORM = STM32F4xx/platform.mk
CHIBIOS_LINKER_DIR = $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/ld
CHIBIOS_BOARD_LINKER = STM32F405xG.ld
CHIBIOS_BOARD_STARTUP = startup_stm32f4xx.mk

########################################################################
# Compiler settings
#
MCU  = cortex-m4

# default flash mode is the DFU
# possibilities: DFU-UTIL, SWD, PX4, Ardupilot bootloader
FLASH_MODE ?= PX4_BOOTLOADER
#DFU_ADDR = 0x08004000
PX4_TARGET = "ap"
PX4_PROTOTYPE ?= "$(PAPARAZZI_HOME)/sw/tools/px4/matek_f405_te_sd.prototype"
PX4_BL_PORT ?= "/dev/serial/by-id/*F4*,/dev/serial/by-id/*F4_*"

#
# default LED configuration
#
RADIO_CONTROL_LED  ?= none
BARO_LED           ?= none
AHRS_ALIGNER_LED   ?= none
GPS_LED            ?= none
SYS_TIME_LED       ?= 1

#
# default UART configuration (modem, gps, rc receiver)
#

# USART1, USART2, USART3, UART4, UART5, USART6 available

MODEM_PORT ?= UART3
MODEM_BAUD ?= B57600

GPS_PORT ?= UART2
GPS_BAUD ?= B57600
DFU_ADDR = 0x08000000
RADIO_CONTROL_SPEKTRUM_PRIMARY_PORT ?= UART1

# single mode
SBUS_PORT ?= UART6

#
# default actuator configuration
#
# you can use different actuators by adding a configure option to your firmware section
# e.g. <configure name="ACTUATORS" value="actuators_ppm/>
# and by setting the correct "driver" attribute in servo section
# e.g. <servo driver="Ppm">
#
ACTUATORS ?= actuators_pwm

# FIXME Bidirectionnal DSHOT timer for input capture timeout
# DSHOT1_GPT_TIM ?= 7
