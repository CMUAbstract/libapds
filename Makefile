LIB = libapds

OBJECTS = \
          min.o \
          min_prox.o \
          proximity.o \
          gesture.o \
          color.o \

DEPS += libmsp libmspware libio libfxl libcapybara

override SRC_ROOT = ../../src

override CFLAGS += \
	-I$(SRC_ROOT)/include \
	-I$(SRC_ROOT)/include/$(LIB) \

include $(MAKER_ROOT)/Makefile.$(TOOLCHAIN)
