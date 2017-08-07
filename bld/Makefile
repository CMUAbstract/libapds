LIB = libapds

#include ../Makefile.config
#include ../Makefile.options

OBJECTS = \
	proximity.o \

DEPS += \
	libmsp \
  libio \
  libmspware \
  libchain\

override SRC_ROOT = ../../src

override CFLAGS += \
	-I$(SRC_ROOT)/include \
	-I$(SRC_ROOT)/include/$(LIB) \

