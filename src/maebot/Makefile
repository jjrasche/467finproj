include ../common.mk

CFLAGS = $(CFLAGS_STD) $(CFLAGS_COMMON) $(CFLAGS_LCMTYPES) $(CFLAGS_GLIB) $(CFLAGS_GTK) -fPIC -O4
LDFLAGS = $(LDFLAGS_STD) $(LDFLAGS_COMMON) $(LDFLAGS_LCMTYPES) $(LDFLAGS_GLIB) $(LDFLAGS_GTK) -llcm

MAEBOT_OBJS = types.o

#LCMTYPES_C := $(shell cd ../lcmtypes && ls *.c)
#LCMTYPES_C := $(addprefix ../lcmtypes/,$(LCMTYPES_C))

LIBMAEBOT=$(LIB_PATH)/libmaebot.a

all: $(LIBMAEBOT) driver


$(LIBMAEBOT): $(MAEBOT_OBJS)
	@echo "\t$@"
	@ar rc $@ $(MAEBOT_OBJS)

driver: $(LIBMAEBOT) maebot_driver.c
	@gcc -o maebot_driver maebot_driver.c $(MAEBOT_OBJS) $(LDFLAGS) $(CFLAGS)
	@cp maebot_driver ../../bin/

clean:
	@rm -f *.o *~ *.a
	@rm -f maebot_test arm_driver arm_example
