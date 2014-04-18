#include "blob_util.h"
#include "camera_util.h"
#include "../common/matd.h"
#include "../common/homography.h"

#define ISTESTCHART 25
#define NUM_TARGETS 4

char* matrix_format = "%15.5f";
float chart_coords[NUM_CHART_BLOBS * 2] = {  0,      0,
                                                17,     0,
                                                34,     0,
                                                51,     0,
                                                68,     0,
                                                0,      17,
                                                17,     17,
                                                34,     17,
                                                51,     17,
                                                68,     17,
                                                0,      34,
                                                17,     34,
                                                34,     34,
                                                51,     34,
                                                68,     34,
                                                0,      51,
                                                17,     51,
                                                34,     51,
                                                51,     51,
                                                68,     51,
                                                0,      68,
                                                17,     68,
                                                34,     68,
                                                51,     68,
                                                68,     68};

float target_coords[NUM_TARGETS * 2] = {    0,  0,
                                            0,  8,
                                            8,  0,
                                            8,  8
                                        };

// float chart_coords[NUM_CHART_BLOBS * 2] = {     -1,      -1,        //lower left
//                                                -.5,      -1,
//                                                  0,      -1,
//                                                 .5,      -1,
//                                                  1,      -1,
//                                                 -1,     -.5,
//                                                -.5,     -.5,
//                                                  0,     -.5,
//                                                 .5,     -.5,
//                                                  1,     -.5,
//                                                 -1,       0,
//                                                -.5,       0,
//                                                  0,       0,
//                                                 .5,       0,
//                                                  1,       0,
//                                                 -1,      .5,
//                                                -.5,      .5,
//                                                  0,      .5,
//                                                 .5,      .5,
//                                                  1,      .5,
//                                                 -1,       1,
//                                                -.5,       1,
//                                                  0,       1,
//                                                 .5,       1,
//                                                  1,       1,};       // upper rigth


void add_circle_to_buffer(vx_buffer_t* buf, double size, loc_t loc, const float* color)
{
    vx_buffer_add_back(buf,
             vxo_pix_coords(VX_ORIGIN_BOTTOM_LEFT,
                    vxo_chain(vxo_mat_translate3(loc.x, loc.y, 0),
                        vxo_mat_scale(size),
                        vxo_circle(vxo_mesh_style(color)))));
}


node_t* resolve_r(node_t* n) {
    if(n->parent_id == n->id) return n;

    n->parent_node = resolve_r(n->parent_node);
    n->parent_id = n->parent_node->id;
    return n->parent_node;
}

// if two nodes have conflicting parents, choose 1 parent to make overall parent
static void connect(node_t* n1, node_t* n2) {
    node_t* n1_parent = resolve_r(n1);          
    node_t* n2_parent = resolve_r(n2);

    n1_parent->parent_node = n2_parent;
    n1_parent->parent_id = n2_parent->id;       //n2 becomes parent of n1
    // n1's child nodes, are not changed to n2
}


// void add_locs_to_buffer(zarray_t* locs, vx_buffer_t* buf)
// {
//     for(int i = 0; i < zarray_size(locs); i++) {
//         loc_t pos;
//         zarray_get(locs, i, &pos);
//         vx_buffer_add_back(buf,
//                  vxo_pix_coords(VX_ORIGIN_BOTTOM_LEFT,
//                         vxo_chain(vxo_mat_translate3(pos.x, pos.y, 0),
//                             vxo_mat_scale(4.0f),
//                             vxo_circle(vxo_mesh_style(vx_maroon)))));
//     }
// }

void project_measurements_through_homography(matd_t* H, vx_buffer_t* buf,
        zarray_t* pix_found, int size)
{
    int npoints = NUM_CHART_BLOBS * 2;          //  line per chart blob
    float points[npoints*3];

    float* real_world_coords;
    if(size == NUM_TARGETS) real_world_coords = target_coords;
    else if(size == NUM_CHART_BLOBS) real_world_coords = chart_coords;
    else assert(0);

    for(int i = 0; i < size; i++) {
        // run each real world point through homography and add to buf
        
        double tmp[3] = {real_world_coords[i*2], real_world_coords[i*2+1], 1};
        matd_t* xy_matrix = matd_create_data(3,1,tmp);
        matd_t* pix_estimated = matd_op("(M)*M",H, xy_matrix);
        MATD_EL(pix_estimated,0,0) /= MATD_EL(pix_estimated,2, 0);
        MATD_EL(pix_estimated,1,0) /= MATD_EL(pix_estimated,2, 0);
        
        vx_buffer_add_back(buf,
                 vxo_pix_coords(VX_ORIGIN_BOTTOM_LEFT,
                        vxo_chain(vxo_mat_translate3(MATD_EL(pix_estimated,0,0), MATD_EL(pix_estimated,1,0), 0),
                            vxo_mat_scale(2.0),
                            vxo_circle(vxo_mesh_style(vx_green)))));

        // create endpoints for lines
        loc_t pos;
        zarray_get(pix_found, i, &pos); //     

        points[6*i + 0] = pos.x;
        points[6*i + 1] = pos.y;
        points[6*i + 2] = 0;
        points[6*i + 3] = MATD_EL(pix_estimated,0,0);
        points[6*i + 4] = MATD_EL(pix_estimated,1,0);
        points[6*i + 5] = 0;
    }

    // make lines
    vx_resc_t *verts = vx_resc_copyf(points, npoints*3);
    vx_buffer_add_back(buf, vxo_pix_coords(VX_ORIGIN_BOTTOM_LEFT,
                                    vxo_lines(verts, npoints, GL_LINES, 
                                        vxo_points_style(vx_blue, 2.0f))));
}

void connect_lines(zarray_t* locs, vx_buffer_t* buf)
{
    connection_t lines[12]; 
    int idx = 0;
    // iterate 
    for(int i = 0; i < zarray_size(locs); i++) {
        loc_t curr;
        zarray_get(locs, i, &curr);
        for(int j = 0; j < zarray_size(locs); j++) {
            // make a connection to every point except self and add to the lines
            loc_t other;
            zarray_get(locs, j, &other);
            if(i != j) {     // not same point
                lines[idx].start = curr;
                lines[idx].end = other;
                lines[idx].length = sqrt((curr.x - other.x)*(curr.x - other.x) 
                                            + (curr.y - other.y)*(curr.y - other.y));
                idx++;
            }
        }
    }
    // sort the array by distance
    int flag = 1;    
    connection_t tmp;             
    for(int i = 0; (i < 12) && flag; i++)
    {
        flag = 0;       // only go over again if swap has been made
        for(int j = 0; j < (11); j++)
        {
            if(lines[j+1].length < lines[j].length)      
            { 
                tmp = lines[j];            
                lines[j] = lines[j+1];
                lines[j+1] = tmp;
                flag = 1;              
            }
        }
    }

    for(int i = 0; (i < 12); i++)
    {
        // printf("%lf\n", lines[i].length);
    }

    // usleep(2000000);

    int npoints = 8;
    float points[npoints*3];

    for(int i = 0; i < 4; i++) {
        points[6*i + 0] = lines[i*2].start.x;
        points[6*i + 1] = lines[i*2].start.y;
        points[6*i + 2] = 1;
        points[6*i + 3] = lines[i*2].end.x;
        points[6*i + 4] = lines[i*2].end.y;
        points[6*i + 5] = 1;
    }
    vx_resc_t *verts = vx_resc_copyf(points, npoints*3);
    vx_buffer_add_back(buf, vxo_pix_coords(VX_ORIGIN_BOTTOM_LEFT,
                                        vxo_lines(verts, npoints, GL_LINES, 
                                        vxo_points_style(vx_blue, 2.0f))));

}

uint32_t abgr_ave_and_int(abgr_t abgr, int num)
{
    abgr.r /= num;
    abgr.g /= num;
    abgr.b /= num;
    assert(abgr.r <= 0xFF && abgr.g <= 0xFF && abgr.b <= 0xFF);

    uint32_t out = 0;
    out += 0xFF000000;
    out += (abgr.b << 16);
    out += (abgr.g << 8);
    out += (abgr.r << 0);

    return(out);
}

int hsv_error_check(int abgr, metrics_t met, int hsv_calib)
{
    hsv_t hsv = {0,0,0};
    rgb_to_hsv(abgr, &hsv);
    hsv_t error = {fabs(hsv.hue - met.hsv_data[hsv_calib].hsv.hue), 
                    fabs(hsv.sat - met.hsv_data[hsv_calib].hsv.sat), 
                    fabs(hsv.val - met.hsv_data[hsv_calib].hsv.val)};

    // 'Acceptable'
     if((error.hue < met.hsv_data[hsv_calib].error.hue) && 
        (error.sat < met.hsv_data[hsv_calib].error.sat) && 
        (error.val < met.hsv_data[hsv_calib].error.val))
    {
        return(1);
    }   
    return(0);
}

void set_color(float* color_out, const float* in)
{
   for(int i = 0; i < 4; i++){
       color_out[i] = in[i];
   } 
}

int matches_background(uint32_t abgr, metrics_t met, float* color_out)
{
    // matches black        black background calib number is 1
    if(hsv_error_check(abgr, met, BLACKBACKGROUND)) {
        set_color(color_out, vx_black);
        return(1);
    }

    // matches grey    aluminum background calib number is 2
    if(hsv_error_check(abgr, met, ALUMINUMBACKGROUND)) {
        set_color(color_out, vx_gray);
        return(1);
    }
    set_color(color_out, vx_red);
    return(0);
}

void is_extreme(box_t* extremes, int x, int y)
{
    if(x < extremes->left) extremes->left = x;
    if(x > extremes->right) extremes->right = x;
    if(y < extremes->bottom) extremes->bottom = y;
    if(y > extremes->top) extremes->top = y;

}

int get_slope(loc_t a , loc_t b) 
{
    if((a.x - b.x) == 0)
        return(10);
    int ret = (a.y - b.y) /  (a.x - b.x); 
    return(ret); 
}

// test for size, color around blob is one of the two background colors 
int blob_qualifies(image_u32_t* im, node_t* n, metrics_t met, vx_buffer_t* buf)
{
    // check size 
    if(n->num_children < met.min_size) return(0);
    // if(met.std_dev_from_square == 0) return();
    //check squareness
    assert(n->box.left != INT_MIN && n->box.right != INT_MAX &&
            n->box.top != INT_MIN && n->box.bottom != INT_MAX);
    loc_t corners[4] = {{n->box.right, n->box.top},
                        {n->box.left, n->box.bottom},
                        {n->box.right, n->box.bottom},
                        {n->box.left, n->box.top}};
    // // find length of sides
    // double side_lengths[4] = {  sqrt((corners[0].x - corners[1].x)*(corners[0].x - corners[1].x) + (corners[0].y - corners[1].y)*(corners[0].y - corners[1].y)),
    //                             sqrt((corners[1].x - corners[2].x)*(corners[1].x - corners[2].x) + (corners[1].y - corners[2].y)*(corners[1].y - corners[2].y)),
    //                             sqrt((corners[2].x - corners[3].x)*(corners[2].x - corners[3].x) + (corners[2].y - corners[3].y)*(corners[2].y - corners[3].y)),
    //                             sqrt((corners[3].x - corners[0].x)*(corners[3].x - corners[0].x) + (corners[3].y - corners[0].y)*(corners[3].y - corners[0].y))};
    // // find mean of lengths
    // double mean = (side_lengths[0] + side_lengths[1] + side_lengths[2] + side_lengths[3]) / 4;
    // // add differences from mean
    // double diff_sum = 0;
    // for(int i = 0; i < 4; i++) {
    //     diff_sum += (side_lengths[i] - mean)*(side_lengths[i] - mean);
    // }
    // // calc variance
    // double std_dev = sqrt(diff_sum/4);
    // // printf("std_dev %lf    comp %lf\n", std_dev, met.std_dev_from_square);
    // if(std_dev > met.std_dev_from_square) {
    //     // printf("bad\n");
    //     return(0);
    // }


    //check that corners of node are on apropriate background
    // method 1: take average of 5 pixels from two opposite sides, and compare to background colors
    for(int j = 0; j < 2; j++) {        // diaganol lines
        abgr_t ave_color_of_line = {0xFF,0,0,0}; 
        int slope = get_slope(corners[j*2], corners[j*2+1]); 
        for(int k = 2; k < 7; k++) {    // pixels in line 
            loc_t loc = {corners[j*2].x + k, corners[j*2].y + k*slope};
            if(in_range(im, loc.x, loc.y)) {
                ave_color_of_line.r += (im->buf[loc.y*im->stride + loc.x] & 0xFF);
                ave_color_of_line.g += (im->buf[loc.y*im->stride + loc.x] >> 8 & 0xFF);
                ave_color_of_line.b += (im->buf[loc.y*im->stride + loc.x] >> 16 & 0xFF);
            }
        }
        float color_out[4];
        int matches = matches_background(abgr_ave_and_int(ave_color_of_line, 5), met, color_out);
        for(int k = 2; k < 7; k++) {    // if is a match add the color it matches to buf 
            loc_t loc = {corners[j*2].x + k, corners[j*2].y + k*slope};
            add_circle_to_buffer(buf, 1.5, loc, color_out);
        }
        if(!matches) return(0); 
    }
    return(1);
}


void hsv_find_balls_blob_detector(image_u32_t* im, frame_t frame, metrics_t met, 
                                    zarray_t* blobs_out, vx_buffer_t* buf)
{
    assert(frame.xy0.x < frame.xy1.x && frame.xy0.y < frame.xy1.y);
    assert(frame.xy0.x >= 0 && frame.xy0.y >= 0 && frame.xy1.x < im->width && frame.xy1.y < im->height);
    assert(frame.ex0.x < frame.ex1.x && frame.ex0.y < frame.ex1.y);
    assert(frame.ex0.x >= 0 && frame.ex0.y >= 0 && frame.ex1.x < im->width && frame.ex1.y < im->height);


    // Int to node
    zhash_t* node_map = zhash_create(sizeof(uint32_t), sizeof(node_t*),
            zhash_uint32_hash, zhash_uint32_equals);

    for(int i = frame.xy0.y; i < frame.xy1.y; i++) {
        for(int j = frame.xy0.x; j < frame.xy1.x; j++) {
            if((i < frame.ex0.y || i > frame.ex1.y) || (j < frame.ex0.x || j > frame.ex1.x)) {

                uint32_t idx_im = i * im->stride + j; // Indframe.ex relative to image

                // 'Acceptable'
                 if(hsv_error_check(im->buf[idx_im], met, TARGETCOLOR))
                 {
                    // Create new node, set itself up as a parent
                    node_t* n = calloc(1, sizeof(node_t));
                    n->id = idx_im;
                    n->parent_id = idx_im;
                    n->parent_node = n;
                    n->num_children = 1;
                    n->ave_loc.x = 0;
                    n->ave_loc.y = 0;
                    box_t tmp = {INT_MAX,INT_MIN, INT_MIN, INT_MAX};
                    n->box = tmp;

                    node_t* tmp_node;
                    uint32_t tmp_idx;

                    // Add node to node map
                    if(zhash_put(node_map, &idx_im, &n, &tmp_idx, &tmp_node)==1) 
                    {
                        assert(0);
                    }

                    //Check if apart of another blob, or starting a new blob 
                    // if apart of another, point to the parent, if a new blob, point to self 
                    //Check neighbours
                    if(!met.lines) {    // only check this if don't want lines for tape detection
                        if(j > frame.xy0.x) {
                            tmp_idx = idx_im - 1; // is Left neighbour similar color 
                            if(zhash_get(node_map, &tmp_idx, &tmp_node) == 1) {
                                node_t* neighbour = tmp_node;
                                connect(n, neighbour);                    
                            }
                        }
                    }
                    if(i > frame.xy0.y) { 
                        tmp_idx = idx_im - im->stride; // is Bottom neighbor similar color
                        if(tmp_idx > 0 && zhash_get(node_map, &tmp_idx, &tmp_node) == 1) {
                            node_t* neighbour = tmp_node;
                            connect(neighbour,n);                    
                        }
                    }
                }
            }
        }
    }

    /*
    ways to weed out false positives
    1) check around area for both black and grey pixels
        - gather more data, try and find center of the blob 
    2) form vectors from all matches, and if they don't have two 
        perpindicular of the same magnitude throw out
    3) ensure that the blob is a square 
    */

    zarray_t* vals = zhash_values(node_map);
    for(int i = 0; i < zarray_size(vals); i++) {
        node_t* node;
        zarray_get(vals, i, &node);
        resolve_r(node);

        if(node->parent_id != node->id) {
            node->parent_node->num_children++;
            node->parent_node->ave_loc.x += node->id%im->stride;
            node->parent_node->ave_loc.y += node->id/im->stride;
            is_extreme(&node->parent_node->box, node->id%im->stride, node->id/im->stride);

            // key should exist, if it doesn't find out why
            assert(zhash_remove(node_map, &node->id, NULL, NULL));
        }
    }

    // search parent only hash and add to blobs out conditionally
    vals = zhash_values(node_map);
    for(int i = 0; i < zarray_size(vals); i++) {
        node_t* node;
        zarray_get(vals, i, &node);
        if(node->num_children > met.min_size) {

            // loc_t pos;
            // pos.x = node->ave_loc.x/node->num_children; //node->parent_id%im->stride;
            // pos.y = node->ave_loc.y/node->num_children; //node->parent_id/im->stride;
            zarray_add(blobs_out, node);
            // printf("parent %d\n", node->id);
        }
    }
    zarray_destroy(vals);
    zhash_vmap_values(node_map, free);
    zhash_destroy(node_map);
}


// form arguments for homography_compute and return the homography matrix
matd_t* dist_homography(int* pix, int size)
{
    zarray_t* correspondences = zarray_create(sizeof(float[4]));
    // printf("\ncorrespondences\n");
    
    float* real_world_coords;
    if(size == NUM_TARGETS) real_world_coords = target_coords;
    else if(size == NUM_CHART_BLOBS) real_world_coords = chart_coords;
    else assert(0);

    for(int i = 0; i < size; i++)
    {
        float tmp[4] = {real_world_coords[i*2], real_world_coords[i*2+1], 
                        pix[i*2], pix[i*2+1]};
        zarray_add(correspondences, &tmp);
        // printf("%f %f %f %f \n", tmp[0], tmp[1], tmp[2], tmp[3]);    
    }
    matd_t * H = homography_compute(correspondences);
    return(H);
}

// return true if a is above or to the left of b
int compare(const void* a, const void* b)
{
    // note, this could be a source of error, So if homography 
    // goes crazy from small movements look here!! The sorting may be wrong
    int pixel_error_margin = 15;

    loc_t* c = (loc_t*)a;
    loc_t* d = (loc_t*)b;

    // same row
    if(abs(c->y - d->y) < pixel_error_margin) 
    {
        // shouldn't be two points within 2 pixels of each other
       // assert(abs(a_xy[0] - b_xy[0]) > pixel_error_margin);
        // a is right of b
        if(c->x > d->x) {
            return(1);
        }
    }   // a above b
    else if(c->y > d->y) 
        return(1);
    return(0);
}


// returns the 35 points associated to the test chart in [x1,y1,x2,y2] 
// format if there are more than 35 points will return NULL
matd_t* build_homography(image_u32_t* im, vx_buffer_t* buf, metrics_t met)
{
    frame_t frame = {{0,0}, {im->width-1, im->height-1},
                        {0,0}, {1,1}};
    int good_size = 0;

    // metrics_t met = {   {180, .249, .196},      // hue
    //                     {42.1, .131, .068},     // error
    //                     40,                     // num_blobs
    //                     0                       // no lines
    //                 };

    zarray_t* blobs = zarray_create(sizeof(node_t));
    hsv_find_balls_blob_detector(im, frame, met, blobs, buf);

    // remove unqualified blobs
    if(met.qualify) {
        for(int i = 0; i < zarray_size(blobs); i++) {
            node_t n;
            zarray_get(blobs, i, &n);

            if(!blob_qualifies(im, &n, met, buf))
                zarray_remove_index(blobs, i, 0);
        }
    }
    if(zarray_size(blobs) == NUM_TARGETS ||zarray_size(blobs) == NUM_CHART_BLOBS) good_size = 1;

    zarray_sort(blobs, compare);
    int pix_array[zarray_size(blobs)*2];

    // iterate through
    int idx = 0;
    double size = 2.0;
    for(int i = 0; i < zarray_size(blobs); i++) {
        node_t n;
        zarray_get(blobs, i, &n);
        loc_t pos = {   .x = n.ave_loc.x/n.num_children,
                        .y = n.ave_loc.y/n.num_children};

        if(buf != NULL) {
            vx_buffer_add_back(buf,
                     vxo_pix_coords(VX_ORIGIN_BOTTOM_LEFT,
                            vxo_chain(vxo_mat_translate3(pos.x, pos.y, 0),
                                vxo_mat_scale(size),
                                vxo_circle(vxo_mesh_style(vx_maroon)))));
            loc_t corners[4] = {{n.box.right, n.box.top},
                                {n.box.right, n.box.bottom},
                                {n.box.left, n.box.top},
                                {n.box.left, n.box.bottom}};
            // print sides of box
            if(1) {
                for(int j = 0; j < 4; j++) {
                    add_circle_to_buffer(buf, size, corners[j], vx_maroon);
                }
            }
        }
        // size += .1;
        pix_array[idx*2] = pos.x;
        pix_array[idx*2+1] = pos.y;
        idx++;
    }

    matd_t* H;
    if(0) {//zarray_size(blobs) == NUM_CHART_BLOBS){
        H = dist_homography(pix_array, NUM_CHART_BLOBS);
    }
    else if(zarray_size(blobs) == NUM_TARGETS){
        H = dist_homography(pix_array, NUM_TARGETS);
        if(met.add_lines) connect_lines(blobs, buf);
    }
    else {
        printf("num figures: %d\n", zarray_size(blobs));
        return(NULL);
    }

    // make projected points
    // project_measurements_through_homography(H, buf, blobs, zarray_size(blobs));
    zarray_destroy(blobs);

    return(H);
}

// if buf is NULL, will not fill with points of the homography
void take_measurements(image_u32_t* im, vx_buffer_t* buf, metrics_t met)
{
    // form homography
    matd_t* H = build_homography(im, buf, met);
    if(H == NULL) return;

    // get model view from homography
    matd_t* Model = homography_to_pose(H, 654, 655, 334, 224);
    // printf("\n");
    // matd_print(H, matrix_format);
    // printf("\n\n");
    // printf("model:\n");
    // matd_print(Model, "%15f");
    // printf("\n\n");
    // matd_print(matd_op("M^-1",Model), matrix_format);
    // printf("\n");
    // extrapolate metrics from model view
    double TZ = MATD_EL(Model, 2, 3);
    double TX = MATD_EL(Model, 0, 3);
    double TY = MATD_EL(Model, 1, 3);

    double cosine = MATD_EL(Model, 0, 0);
    double theta = acos(cosine);
    char str[100];
    sprintf(str, "<<#00ffff,serif-30>> DIST:%lf   Rot:%lf\n Offset (%lf, %lf)", 
                TZ, theta, TX, TY);
    vx_object_t *text = vxo_text_create(VXO_TEXT_ANCHOR_BOTTOM_LEFT, str); 
    vx_buffer_add_back(buf, vxo_pix_coords(VX_ORIGIN_BOTTOM_LEFT, text));
    // vxo_text_create(VXO_TEXT_ANCHOR_CENTER,
    //                                           "<<right,#aa00ff,monospaced>>Example Text"));
    // printf("dist: %lf   cos:%lf  angle: %lf\n", TZ, cosine, theta);
}
