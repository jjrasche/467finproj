include ../common.mk

CFLAGS = $(CFLAGS_STD)

# Construct the pathnames to objs
LCMTYPES_NAMES := $(shell cd ../../lcmtypes && ls *.lcm)
LCMTYPES_OBJS := $(LCMTYPES_NAMES:%.lcm=%.o)

LIBLCMTYPES=$(LIB_PATH)/liblcmtypes.a
LIBLCMTYPES_SO=$(LIB_PATH)/liblcmtypes.so

all: $(LIBLCMTYPES) $(LIBLCMTYPES_SO)

$(LIBLCMTYPES): $(LCMTYPES_OBJS)
	@echo "\t$@"
	@ar rc $@ $(LCMTYPES_OBJS)

$(LIBLCMTYPES_SO): $(LCMTYPES_OBJS)
	@echo "\t$@"
	@gcc -shared -o $@ $(LCMTYPES_OBJS)

clean:
	@rm -f *~ *.o *.a *.h *.c
