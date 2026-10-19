#
#  Makefile for fe_ext_fw (SDCC + libfx2)
#
#  Output: fe_ext_fw.ihex (Intel HEX, RAM load or fx2tool program)
#          fe_ext_fw.iic  (EEPROM C2 boot image, CyControl)
#

LIBFX2    ?= libfx2/firmware/library
TARGET     = fe_ext_fw
SOURCES    = fe_ext_fw
LIBRARIES  = fx2 fx2usb fx2isrs
MODEL      = small
CFLAGS     = -DSYNCDELAYLEN=16
PYTHON    ?= python3

include $(LIBFX2)/fx2rules.mk

all: $(TARGET).iic

$(TARGET).iic: $(TARGET).ihex
	$(PYTHON) hex2iic.py $< $@
