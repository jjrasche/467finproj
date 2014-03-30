#include "blob_detect.h"
#include "camera_util.h"

typedef struct node node_t;
struct node {
    uint32_t id;
    uint32_t parent_id;
    node_t *parent_node;
    int num_children;
};

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

void find_balls_blob_detector(image_u32_t* im, float* xy0, float* xy1, float* ex0, float* ex1,
        int32_t* rgb_in, int32_t* max_error, zarray_t* blobs_out, int min_blob_size, int lines) 
{
    assert(xy0[0] < xy1[0] && xy0[1] < xy1[1]);
    assert(xy0[0] >= 0 && xy0[1] >= 0 && xy1[0] < im->width && xy1[1] < im->height);
    assert(ex0[0] < ex1[0] && ex0[1] < ex1[1]);
    assert(ex0[0] >= 0 && ex0[1] >= 0 && ex1[0] < im->width && ex1[1] < im->height);

    // Int to node
    zhash_t* node_map = zhash_create(sizeof(uint32_t), sizeof(node_t*), 
            zhash_uint32_hash, zhash_uint32_equals);

    for(int i = xy0[1]; i < xy1[1]; i++) {
        for(int j = xy0[0]; j < xy1[0]; j++) {
            if((i < ex0[1] || i > ex1[1]) || (j < ex0[0] || j > ex1[0])) {

                uint32_t idx_im = i * im->stride + j; // Index relative to image

                // Pixel color data
                uint32_t abgr = im->buf[idx_im];

                int32_t r = (abgr >> 0) & 0xff;
                int32_t g = (abgr >> 8) & 0xff;
                int32_t b = (abgr >> 16) & 0xff;

                int32_t error[3] = {abs(r - rgb_in[0]), abs(g - rgb_in[1]), abs(b - rgb_in[2])};

                // 'Acceptable' px
                if(error[0] < max_error[0] && error[1] < max_error[1] && error[2] < max_error[2]) {
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
                    if(!lines) {    // only check this if don't want lines for tape detection
                        if(j > xy0[0]) {
                            tmp_idx = idx_im - 1; // is Left neighbour similar color 
                            if(zhash_get(node_map, &tmp_idx, &tmp_node) == 1) {
                                node_t* neighbour = tmp_node;
                                connect(n, neighbour);                    
                            }
                        }
                    }
                    if(i > xy0[1]) { 
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
            // node_t* tmp;
            int ret = zhash_remove(node_map, &node->id, NULL, NULL);//&tmp);
            // free(tmp);
            assert(ret);

        }
    }
    zarray_destroy(vals);
    // search parent only hash and add to blobs out conditionally 
    vals = zhash_values(node_map);
    for(int i = 0; i < zarray_size(vals); i++) {
        node_t* node;
        zarray_get(vals, i, &node);
        if(node->num_children > min_blob_size) {
            blob_detector_ballpos_t pos;
            pos.position[0] = node->parent_id%im->stride;
            pos.position[1] = node->parent_id/im->stride;
            zarray_add(blobs_out, &pos);
            // printf("parent %d\n", node->id);
        }
    } 
    zarray_destroy(vals);
    zhash_vmap_values(node_map, free);
    zhash_destroy(node_map);
}


// same code as above, but will iterate over pixels from top->bottom and right->left
void blob_detect_td_rl(image_u32_t* im, float* xy0, float* xy1, float* ex0, float* ex1,
        int32_t* rgb_in, int32_t* max_error, zarray_t* blobs_out, int min_blob_size, int lines) 
{
    assert(xy0[0] < xy1[0] && xy0[1] < xy1[1]);
    assert(xy0[0] >= 0 && xy0[1] >= 0 && xy1[0] < im->width && xy1[1] < im->height);
    assert(ex0[0] < ex1[0] && ex0[1] < ex1[1]);
    assert(ex0[0] >= 0 && ex0[1] >= 0 && ex1[0] < im->width && ex1[1] < im->height);

    // Int to node
    zhash_t* node_map = zhash_create(sizeof(uint32_t), sizeof(node_t*), 
            zhash_uint32_hash, zhash_uint32_equals);

    // 
    for(int i = xy1[1]; i > xy0[1]; i--) {
        for(int j = xy1[0]; j > xy0[0]; j--) {
            if((i < ex0[1] || i > ex1[1]) || (j < ex0[0] || j > ex1[0])) {

                uint32_t idx_im = i * im->stride + j; // Index relative to image

                // Pixel color data
                uint32_t abgr = im->buf[idx_im];

                int32_t r = (abgr >> 0) & 0xff;
                int32_t g = (abgr >> 8) & 0xff;
                int32_t b = (abgr >> 16) & 0xff;

                int32_t error[3] = {abs(r - rgb_in[0]), abs(g - rgb_in[1]), abs(b - rgb_in[2])};

                // 'Acceptable' px
                if(error[0] < max_error[0] && error[1] < max_error[1] && error[2] < max_error[2]) {
                    // Create new node, set itself up as a parent
                    node_t* n = calloc(1, sizeof(node_t));
                    n->id = idx_im;
                    n->parent_id = idx_im;
                    n->parent_node = n;
                    n->num_children = 0;
                    node_t* tmp_node;
                    uint32_t tmp_idx;
                    // Add node to node map
                    zhash_put(node_map, &idx_im, &
                        n, &tmp_idx, &tmp_node);

                    if(!lines) {    // only check this if don't want lines for tape detection
                        if(j < xy1[0]) {
                            tmp_idx = idx_im + 1; // is Right neighbour similar color 
                            if(zhash_get(node_map, &tmp_idx, &tmp_node) == 1) {
                                node_t* neighbour = tmp_node;
                                connect(n, neighbour);                    
                            }
                        }
                    }

                    if(i < xy1[1]) { 
                        tmp_idx = idx_im + im->stride; // is Top neighbor similar color
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
        if(node->num_children > min_blob_size) {
            blob_detector_ballpos_t pos;
            pos.position[0] = node->parent_id%im->stride;
            pos.position[1] = node->parent_id/im->stride;
            zarray_add(blobs_out, &pos);
           // printf("parent %d\n", node->id);
        }
    } 
    zarray_destroy(vals);
    zhash_vmap_values(node_map, free);
    zhash_destroy(node_map);
}


/*
void find_balls_template_matcher(image_u32_t* im, float* xy0, float* xy1, float* ex0, float* ex1, 
        image_u32_t* im_template, uint32_t max_error, zarray_t* blobs_out) {
    assert(xy0[0] < xy1[0] && xy0[1] < xy1[1]);
    assert(xy0[0] >= 0 && xy0[1] >= 0 && xy1[0] < im->width && xy1[1] < im->height);
    assert(ex0[0] < ex1[0] && ex0[1] < ex1[1]);
    assert(ex0[0] >= 0 && ex0[1] >= 0 && ex1[0] < im->width && ex1[1] < im->height);
    assert(im_template->width > 0);
    assert(im_template->height > 0);

    for(int y = xy0[1]; y < xy1[1] - im_template->height; y++) {
        for(int x = xy0[0]; x < xy1[0] - im_template->width; x++) {
            if((x < ex0[0] - im_template->height || x > ex1[0]) || 
                    (y < ex0[1] - im_template->width || y > ex1[1])) {
                double error = 0;
                for(int ty = 0; ty < im_template->height; ty++) {
                    for(int tx = 0; tx < im_template->width; tx++) {

                        int32_t idx_im = ((ty+y)*im->stride) + (tx+x); // Index relative to image
                        int32_t idx_template = (ty*im_template->stride) + tx;

                        // Pixel color data
                        uint32_t abgr_im = im->buf[idx_im];
                        uint32_t abgr_template = im_template->buf[idx_template];

                        int32_t r_im = (abgr_im >> 0) & 0xff;
                        int32_t g_im = (abgr_im >> 8) & 0xff;
                        int32_t b_im = (abgr_im >> 16) & 0xff;
                        int32_t r_template = (abgr_template >> 0) & 0xff;
                        int32_t g_template = (abgr_template >> 8) & 0xff;
                        int32_t b_template = (abgr_template >> 16) & 0xff;

                        error += abs(r_im - r_template) + 
                            abs(g_im - g_template) + 
                            abs(b_im - b_template);
                        if(error > max_error) break;
                    }
                    if(error > max_error) break;
                }
                if(error <= max_error){//(im_template->height+im_template->width)) {
                    blob_detector_ballpos_t pos;
                    pos.position[0] = x + im_template->width/2.0;
                    pos.position[1] = y + im_template->height/2.0;
                    zarray_add(blobs_out, &pos);
                    printf("found ball at: %d, %d\n", x, y);
                }
            }
        }
    }
    printf("template match done.\n");
}

*/

void find_diamonds_blob_detector(image_u32_t* im, float* xy0, float* xy1, float* ex0, float* ex1,
        int32_t* rgb_in, int32_t* max_error, zarray_t* blobs_out, int min_blob_size, int lines) 
{
    find_balls_blob_detector(im, xy0, xy1, ex0, ex1, rgb_in, max_error, blobs_out, min_blob_size, lines);

    // Check if there is a 'white' pixel after each blob, discard if not
    int white_below_green = 10;
    int white_error = 25;
    int min_white = 0xe0;
    for(int i = 0; i < zarray_size(blobs_out); i++) {

        blob_detector_ballpos_t blob;
        zarray_get(blobs_out, i, &blob);
         if (blob.position[1] + white_below_green >= im->height) continue;

        uint32_t abgr = im->buf[(int)((blob.position[1] + white_below_green) * im->stride + blob.position[0])];
        int32_t r = (abgr >> 0) & 0xff;
        int32_t g = (abgr >> 8) & 0xff;
        int32_t b = (abgr >> 16) & 0xff;

        int32_t avg = (r + g + b) / 3;

        if(abs(r - avg) > white_error || abs(g - avg) > white_error || abs(b - avg) > white_error || avg < min_white) {
            // printf("removing cause not white\n");
            //zarray_remove_index(blobs_out, i--, 1);
        }
    }
}

void hsv_find_diamonds_blob_detector(image_u32_t* im, float* xy0, float* xy1, float* ex0, float* ex1,
        double* hsv_in, double* max_error, zarray_t* blobs_out, int min_blob_size, int lines) 
{
    hsv_find_balls_blob_detector(im, xy0, xy1, ex0, ex1, hsv_in, max_error, blobs_out, min_blob_size, lines);

    // Check if there is a 'white' pixel after each blob, discard if not
    int white_below_green = 10;
    int white_error = 25;
    int min_white = 0xe0;
    for(int i = 0; i < zarray_size(blobs_out); i++) {

        blob_detector_ballpos_t blob;
        zarray_get(blobs_out, i, &blob);
         if (blob.position[1] + white_below_green >= im->height) continue;

        uint32_t abgr = im->buf[(int)((blob.position[1] + white_below_green) * im->stride + blob.position[0])];
        int32_t r = (abgr >> 0) & 0xff;
        int32_t g = (abgr >> 8) & 0xff;
        int32_t b = (abgr >> 16) & 0xff;

        int32_t avg = (r + g + b) / 3;

        if(abs(r - avg) > white_error || abs(g - avg) > white_error || abs(b - avg) > white_error || avg < min_white) {
            // printf("removing cause not white\n");
            //zarray_remove_index(blobs_out, i--, 1);
        }
    }
}

void hsv_find_balls_blob_detector(image_u32_t* im, float* xy0, float* xy1, float* ex0, float* ex1,
        double* hsv_in, double* max_error, zarray_t* blobs_out, int min_blob_size, int lines)
{
    assert(xy0[0] < xy1[0] && xy0[1] < xy1[1]);
    assert(xy0[0] >= 0 && xy0[1] >= 0 && xy1[0] < im->width && xy1[1] < im->height);
    assert(ex0[0] < ex1[0] && ex0[1] < ex1[1]);
    assert(ex0[0] >= 0 && ex0[1] >= 0 && ex1[0] < im->width && ex1[1] < im->height);

    // Int to node
    zhash_t* node_map = zhash_create(sizeof(uint32_t), sizeof(node_t*),
            zhash_uint32_hash, zhash_uint32_equals);

    for(int i = xy0[1]; i < xy1[1]; i++) {
        for(int j = xy0[0]; j < xy1[0]; j++) {
            if((i < ex0[1] || i > ex1[1]) || (j < ex0[0] || j > ex1[0])) {

                uint32_t idx_im = i * im->stride + j; // Index relative to image

                // Pixel color data
                uint32_t abgr = im->buf[idx_im];
                double hsv[3];
                rgb_to_hsv(abgr, hsv);
                double error[3] = {fabs(hsv[0] - hsv_in[0]), fabs(hsv[1] - hsv_in[1]), fabs(hsv[2] - hsv_in[2])};

                // 'Acceptable'
                 if(error[0] < max_error[0] && error[1] < max_error[1] && error[2] < max_error[2]) {
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
                    if(!lines) {    // only check this if don't want lines for tape detection
                        if(j > xy0[0]) {
                            tmp_idx = idx_im - 1; // is Left neighbour similar color 
                            if(zhash_get(node_map, &tmp_idx, &tmp_node) == 1) {
                                node_t* neighbour = tmp_node;
                                connect(n, neighbour);                    
                            }
                        }
                    }
                    if(i > xy0[1]) { 
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
        if(node->num_children > min_blob_size) {
            blob_detector_ballpos_t pos;
            pos.position[0] = node->parent_id%im->stride;
            pos.position[1] = node->parent_id/im->stride;
            zarray_add(blobs_out, &pos);
           // printf("parent %d\n", node->id);
        }
    }
    zarray_destroy(vals);
    zhash_vmap_values(node_map, free);
    zhash_destroy(node_map);
}

