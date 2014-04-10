#include "blob_util.h"
#include "camera_util.h"
#include "../common/matd.h"
#include "../common/homography.h"

char* matrix_format = "%15.5f";
float xy_plane_coords[NUM_CHART_BLOBS * 2] = {  0,      0,
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

void hsv_find_balls_blob_detector(image_u32_t* im, frame_t frame, metrics_t met, zarray_t* blobs_out)
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

                // Pixel color data
                uint32_t abgr = im->buf[idx_im];
                hsv_t hsv = {0,0,0};
                rgb_to_hsv(abgr, &hsv);
                hsv_t error = {fabs(hsv.hue - met.hsv.hue), 
                                fabs(hsv.sat - met.hsv.sat), 
                                fabs(hsv.val - met.hsv.val)};

                // 'Acceptable'
                 if((error.hue < met.error.hue) && 
                    (error.sat < met.error.sat) && 
                    (error.val < met.error.val)) 
                 {
                    // Create new node, set itself up as a parent
                    node_t* n = calloc(1, sizeof(node_t));
                    n->id = idx_im;
                    n->parent_id = idx_im;
                    n->parent_node = n;
                    n->num_children = 0;

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


    //count number of children for each parent, go through node_map
    // if a node is not a parent, add 1 to it's parent->num_children and delete from hash
    // if is a parent do nothing
    zarray_t* vals = zhash_values(node_map);
    for(int i = 0; i < zarray_size(vals); i++) {
        node_t* node;
        zarray_get(vals, i, &node);
        resolve_r(node);

        if(node->parent_id != node->id) {
            node->parent_node->num_children++;
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
            loc_t pos;
            pos.x = node->parent_id%im->stride;
            pos.y = node->parent_id/im->stride;
            zarray_add(blobs_out, &pos);
            // printf("parent %d\n", node->id);
        }
    }
    zarray_destroy(vals);
    zhash_vmap_values(node_map, free);
    zhash_destroy(node_map);
}


// form arguments for homography_compute and return the homography matrix
matd_t* dist_homography(int* pix)
{
    zarray_t* correspondences = zarray_create(sizeof(float[4]));
    for(int i = 0; i < NUM_CHART_BLOBS; i++)
    {
        float tmp[4] = {pix[i*2], pix[i*2+1], 
                        xy_plane_coords[i*2], xy_plane_coords[i*2+1]};
        zarray_add(correspondences, &tmp);
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

    // metrics_t met = {   {180, .249, .196},      // hue
    //                     {42.1, .131, .068},     // error
    //                     40,                     // num_blobs
    //                     0                       // no lines
    //                 };

    int pix_array[NUM_CHART_BLOBS*2];

    zarray_t* blobs = zarray_create(sizeof(loc_t));
    hsv_find_balls_blob_detector(im, frame, met, blobs);
    zarray_sort(blobs, compare);

    // iterate through
    int idx = 0;
    double size = 2.0;
    for(int i = 0; i < zarray_size(blobs); i++) {
        loc_t pos;
        zarray_get(blobs, i, &pos);
        if(buf != NULL) {
            vx_buffer_add_back(buf,
                     vxo_pix_coords(VX_ORIGIN_BOTTOM_LEFT,
                            vxo_chain(vxo_mat_translate3(pos.x, pos.y, 0),
                                vxo_mat_scale(size),
                                vxo_circle(vxo_mesh_style(vx_maroon)))));
        }
        // size += .2;
        // printf("(%d, %d)\n", pos.x, pos.y);
        if(zarray_size(blobs) == NUM_CHART_BLOBS) {
            pix_array[idx*2] = pos.x;
            pix_array[idx*2+1] = pos.y;
            idx++;
        }
    }
    if(zarray_size(blobs) != NUM_CHART_BLOBS) {
        printf("num figures: %d\n", zarray_size(blobs));
        return(NULL);
    }

    zarray_destroy(blobs);

    return(dist_homography(pix_array));
}

// if buf is NULL, will not fill with points of the homography
void take_measurements(image_u32_t* im, vx_buffer_t* buf, metrics_t met)
{
    // form homography
    matd_t* H = build_homography(im, buf, met);
    if(H == NULL) return;

    // get model view from homography
    matd_t* Model = homography_to_pose(H, 949, 949, 0, 0);
    printf("\n");
    matd_print(H, matrix_format);
    printf("\n\n");
    matd_print(Model, matrix_format);
    printf("\n");
    // extrapolate metrics from model view
    double TZ = MATD_EL(Model, 2, 3);
    double cosine = MATD_EL(Model, 0, 0);
    double theta = acos(cosine);
    printf("dist: %lf   cos:%lf  angle: %lf\n", TZ, cosine, theta);
}