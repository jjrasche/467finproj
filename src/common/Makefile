include ../common.mk

CFLAGS = $(CFLAGS_STD) $(CFLAGS_COMMON) $(CFLAGS_GLIB) $(CFLAGS_GTK) -fPIC -O4
LDFLAGS = $(LDFLAGS_STD) $(LDFLAGS_COMMON) $(LDFLAGS_GLIB) $(LDFLAGS_GTK) $(LDFLAGS_LCMTYPES)

COMMON_OBJS = getopt.o gps_linearize.o ioutils.o math_util.o \
	      timespec.o timestamp.o serial.o \
              ssocket.o string_util.o timesync.o \
              c5.o matd.o zarray.o zhash.o task_thread.o \
	      image_u8.o image_u8x3.o image_u32.o pnm.o \
	      image_util.o\
	      pg.o param_widget.o	\
	      dynamixel_bus.o dynamixel_serial_bus.o dynamixel_device.o dynamixel_axseries.o dynamixel_mxseries.o \
		homography.o

LIBCOMMON=$(LIB_PATH)/libcommon.a

all: $(LIBCOMMON) ../../bin/c5 ed_test

ED_TEST_OBJS = ed_test.o matd.o zarray.o homography.o
ed_test: $(ED_TEST_OBJS)
	gcc -o $@ $(ED_TEST_OBJS) -lm

$(LIBCOMMON): $(COMMON_OBJS)
	@echo "\t$@"
	@ar rc $@ $(COMMON_OBJS)

pixels.o: pixels.c
	@echo "\t$@"
	@$(CC) $(CFLAGS) -msse3 -c $<

gridmap.o: gridmap.c
	@echo "\t$@";
	@$(CC) $(CFLAGS) -msse -c $<

../../bin/c5:
	@echo "\t$@";
	@$(CC) -o $@ -O4 -Wall -std=gnu99 -D_C5_MAIN c5.c

clean:
	@rm -f *.o *~ *.a ../../bin/c5
	@rm -f gu_minheap_tester
