include ../common.mk

# flags for building the gtk library
CFLAGS = $(CFLAGS_STD) $(CFLAGS_COMMON) $(CFLAGS_VX) $(CFLAGS_GTK) $(CFLAGS_LCMTYPES) $(CFLAGS_LCM) $(CFLAGS_VX) -fPIC -g
LDFLAGS =  $(LDFLAGS_VX_GTK) $(LDFLAGS_VX) $(LDFLAGS_GTK) $(LDFLAGS_IMAGESOURCE) $(LDFLAGS_COMMON) $(LDFLAGS_LCMTYPES) $(LDFLAGS_LCM) $(LDFLAGS_STD)

BINS= fin distortion
TESTS= map_test move_to_point_test pid_test 

UTILS = custom_util.o camera_util.o homography_botlab.o

all: $(BINS)

test: $(TESTS)

fin: finalproj.o $(UTILS)
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

bot_controller: bot_controller.o $(UTILS) blob_detect.o bot_map.o priority_queue.o controller_params.o
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

gyro_drift_test: gyro_drift_test.o bot_utils.o
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

angle_print: angle_print.o bot_utils.o
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

camera: camera_stuff.o custom_util.o blob_detect.o homography_botlab.o camera_util.o bot_map.o priority_queue.o bot_utils.o
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

distortion: distortion_testing.o custom_util.o camera_util.o homography_botlab.o
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

make_table: distortion_table_build.o camera_util.o blob_detect.o
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

maebot_uncertainty: maebot_uncertainty.o bot_utils.o custom_util.o
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

map_test: map_test.o custom_util.o bot_map.o priority_queue.o
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

move_to_point_test: move_to_point_test.o bot_utils.o
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

turn_on_motors: turn_on_motors.o
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

turn_off_motors: turn_off_motors.o
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

#p_queue_test: p_queue_test.o priority_queue.o bot_map.o
#	@echo "\t$@"
#	@$(CC) -o $@ $^ $(LDFLAGS)

pid_test: pid_test.o bot_utils.o custom_util.o
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

target_test: target_detection_test.o
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

hsv_test: hsv_test.o
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

clean:
	@rm -rf  $(BINS) $(TESTS)  *.o *~
