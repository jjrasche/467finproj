include ../common.mk

CFLAGS = $(CFLAGS_STD) $(CFLAGS_COMMON) $(CFLAGS_LCMTYPES) $(CFLAGS_GLIB) $(CFLAGS_GTK) -fPIC -O4
LDFLAGS = $(LDFLAGS_STD) $(LDFLAGS_COMMON) $(LDFLAGS_LCMTYPES) $(LDFLAGS_GLIB) $(LDFLAGS_GTK) -llcm

MAEBOT_OBJS = types.o

#LCMTYPES_C := $(shell cd ../lcmtypes && ls *.c)
#LCMTYPES_C := $(addprefix ../lcmtypes/,$(LCMTYPES_C))

LIBMAEBOT=$(LIB_PATH)/libmaebot.a

all: $(LIBMAEBOT) driver led_test laser_test motor_feedback_test diff_drive_test


$(LIBMAEBOT): $(MAEBOT_OBJS)
	@echo "\t$@"
	@ar rc $@ $(MAEBOT_OBJS)

driver: $(LIBMAEBOT) maebot_driver.c
	@gcc -o maebot_driver maebot_driver.c $(MAEBOT_OBJS) $(LDFLAGS) $(CFLAGS)
	@cp maebot_driver ../../bin/

led_test: led_test.c
	@gcc -o led_test led_test.c $(LCMTYPES_C) $(LDFLAGS) $(CFLAGS)
	@cp led_test ../../bin/

laser_test: laser_test.c
	@gcc -o laser_test laser_test.c $(LCMTYPES_C) $(LDFLAGS) $(CFLAGS)
	@cp laser_test ../../bin/

motor_feedback_test: motor_feedback_test.c
	@gcc -o motor_feedback_test motor_feedback_test.c $(LCMTYPES_C) $(LDFLAGS) $(CFLAGS)
	@cp motor_feedback_test ../../bin/

diff_drive_test: diff_drive_test.c
	@gcc -o diff_drive_test diff_drive_test.c $(LCMTYPES_C) $(LDFLAGS) $(CFLAGS)
	@cp diff_drive_test ../../bin/

clean:
	@rm -f *.o *~ *.a
	@rm -f maebot_driver maebot_test arm_driver arm_example led_test laser_test motor_feedback_test diff_drive_test
