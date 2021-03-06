include ../common.mk

CFLAGS = $(CFLAGS_STD) $(CFLAGS_COMMON) -fPIC `pkg-config --cflags freetype2` -pedantic -O0
LDFLAGS = $(LDFLAGS_STD) -lpthread $(LDFLAGS_COMMON)


LIBVX_DRAWABLE_OBJS = vxo_chain.o vxo_mat.o vxo_mesh.o vxo_lines.o vxo_points.o vxo_depth_test.o vxo_text.o vxo_rect.o vxo_robot.o vxo_box.o vxo_arrow.o vxo_axes.o vxo_square_pyramid.o vxo_tetrahedron.o vxo_cylinder.o vxo_image.o vxo_grid.o vxo_objmtl.o vxo_circle.o vxo_pix_coords.o vxo_sphere.o
LIBVX_USER_OBJS = vx_world.o vx_layer.o vx_util.o vx_program.o vx_matrix_stack.o vxp.o vx_camera_pos.o default_camera_mgr.o default_event_handler.o vx_ray3.o vx_viewport_mgr.o
LIBVX_CORE_OBJS = vx_code_input_stream.o vx_code_output_stream.o vx_resc.o vx_resc_manager.o vx_tcp_display.o vx_remote_display_source.o vx_global.o vx_tcp_util.o
LIBVX_OBJS = $(LIBVX_CORE_OBJS) $(LIBVX_USER_OBJS) $(LIBVX_DRAWABLE_OBJS)

# The only objects with direct GL references are:
LIBVXGL_OBJS = vx_gl_renderer.o glcontext.o glcontext-x11.o

LIB_VX = ../../lib/libvx.a
LIB_VXGL = ../../lib/libvxgl.a

VX_REMOTE_TEST=../../bin/vx_remote_test

ALL = $(LIB_VX) $(LIB_VXGL) vx_make_font

#############################################################
#gcc -shared -Wl,-soname,$(LIB) -o $(LIB) $(LIBVX_OBJS)

all: $(ALL) fonts

$(LIB_VX):  $(LIBVX_OBJS)
	@echo "\t$@"
	@ar rc $@ $^

$(LIB_VXGL):  $(LIBVXGL_OBJS)
	@echo "\t$@"
	@ar rc $@ $^

$(VX_REMOTE_TEST) : vx_remote_test.o $(LIB)
	@echo "\t$@"
	$(CC) -o $@ $^ $(LDFLAGS) $(LDFLAGS_VX)

# Delete the font directory to pull again
fonts :
	curl http://umbrella.eecs.umich.edu/vx/vx_fonts_1.0.tar.gz > /tmp/vx_fonts_1.0.tar.gz
	tar -xf /tmp/vx_fonts_1.0.tar.gz
	if [ -d fonts ]; then mv fonts /tmp/old_vx_fonts; fi
	mv vx_fonts fonts

vx_make_font: vx_make_font.o
	@echo "\t$@"
	@$(CC) $(CFLAGS) -o vx_make_font $< $(LDFLAGS_COMMON) $(LDFLAGS_STD)

clean:
	@rm -f *~ *.o *.d vx_test $(LIB) $(DESTLIB) $(LIBVXGL) $(DESTLIBVXGL)

test:
	@if [ -d test ]; then $(MAKE) -C test test ; else echo "  No tests"; fi

test-%:
	@if [ -d test ]; then $(MAKE) -C test $@ ; else echo "  No tests"; fi

.PHONY: test
