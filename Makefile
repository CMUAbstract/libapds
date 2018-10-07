LIB = libapds

OBJECTS = proximity.o \
          gesture.o

DEPS += libmsp libmspware libio libfxl libcapybara

override SRC_ROOT = ../../src

override CFLAGS += \
	-I$(SRC_ROOT)/include \
	-I$(SRC_ROOT)/include/$(LIB) \

include $(MAKER_ROOT)/Makefile.$(TOOLCHAIN)
