#include "camera_util.h"


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