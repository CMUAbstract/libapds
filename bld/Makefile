LIB = libapds

#include ../Makefile.config
#include ../Makefile.options

OBJECTS = \
	proximity.o \
  gesture.o \

DEPS += \
	libmsp \
  libmspware \
  libfxl \
  libcapybara

override SRC_ROOT = ../../src

override CFLAGS += \
	-I$(SRC_ROOT)/include \
	-I$(SRC_ROOT)/include/$(LIB) \

