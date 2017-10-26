# Hey Emacs, this is a -*- makefile -*-
#
# lisa_mx_2.1.makefile
#
# http://wiki.paparazziuav.org/wiki/Lisa/M_v20
#

BOARD=lisa_mx
BOARD_VERSION=2.1
BOARD_CFG=\"boards/$(BOARD)_$(BOARD_VERSION).h\"

ARCH=stm32
ARCH_L=f4
HARD_FLOAT=yes
$(TARGET).ARCHDIR = $(ARCH)
$(TARGET).LDSCRIPT=$(SRC_ARCH)/lisa-mx.ld

# -----------------------------------------------------------------------

HAS_LUFTBOOT ?= 0
ifeq (,$(findstring $(HAS_LUFTBOOT),0 FALSE))
$(TARGET).CFLAGS+=-DLUFTBOOT
$(TARGET).LDFLAGS+=-Wl,-Ttext=0x8004000
DFU_ADDR = 0x8004000
DFU_PRODUCT = Lisa/Lia
endif

include $(PAPARAZZI_SRC)/conf/boards/lisa_m_defaults.makefile

# Added by Karl on 25/10/17 to try to fix the SD logging on Transformer

ASPIRIN_2_SPI_DEV ?= spi2
ASPIRIN_2_SPI_SLAVE_IDX ?= SPI_SLAVE2

SDLOGGER_DIRECT_SPI ?= spi1
SDLOGGER_DIRECT_SPI_SLAVE ?= SPI_SLAVE1
HS_LOG_SPI_DEV ?= spi1
HS_LOG_SPI_SLAVE_IDX ?= SPI_SLAVE1