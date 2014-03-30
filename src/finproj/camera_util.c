#include "camera_util.h"
#include "limits.h"
// #include "math.h"
#define  Pr  .299
#define  Pg  .587
#define  Pb  .114

void deep_copy_image (image_u32_t* in, image_u32_t* out) {
    out->height = in->height;
    out->width = in->width;
    out->stride = in->stride;

    out->buf = calloc((in->height*in->stride), sizeof(uint32_t));

    for (int i = 0; i < (in->height*in->stride); i++) { 
        out->buf[i] = 0xFFFFFFFF;
    }

}

void capture_image(image_u32_t *im, int num)
{
    char file_name[100];
    printf("value %d\n",sprintf(file_name, "/afs/umich.edu/user/r/o/ronakrm/eecs467/eecs467_botlab/src/botlab/pic%d.pnm"
                , num));
    int ret = image_u32_write_pnm(im, file_name);
    printf("take image %d, err = %d", num, ret);

}

// return int with average RGB values
int average_surrounding_pixels(image_u32_t* image, int x, int y) {
    int ave[3] = {0,0,0};
    int num_pixels = 0;

    int space = 0x00000000;

    if(x !=0 && image->buf[y*image->stride + (x-1)] != space) {   // if not on left side check left pixel
        ave[2] += (image->buf[y*image->stride+ (x-1)] & 0xff);
        ave[1] += (image->buf[y*image->stride+ (x-1)]>>8 & 0xff);
        ave[0] += (image->buf[y*image->stride+ (x-1)]>>16 & 0xff);
        // printf("1st = 0x%x\n", image->buf[y*image->stride+ (x-1)]);

        num_pixels++;
    }
    if(x != image->width && image->buf[y*image->stride + (x+1)] != space) {   // if not on right side
        ave[2] += (image->buf[y*image->stride+ (x+1)] & 0xff);
        ave[1] += (image->buf[y*image->stride+ (x+1)]>>8 & 0xff);
        ave[0] += (image->buf[y*image->stride+ (x+1)]>>16 & 0xff);
        // printf("2nd = 0x%x\n", image->buf[y*image->stride+ (x+1)]);

        num_pixels++;
    } 
    if(y != 0 && image->buf[(y-1)*image->stride + (x)] != space) {   // if not on right side
        ave[2] += (image->buf[(y-1)*image->stride+ x] & 0xff);
        ave[1] += (image->buf[(y-1)*image->stride+ x]>>8 & 0xff);
        ave[0] += (image->buf[(y-1)*image->stride+ x]>>16 & 0xff);
        // printf("3rd = 0x%x\n", image->buf[(y-1)*image->stride+ x]);

        num_pixels++;
    }       
    if(y != image->height && image->buf[(y+1)*image->stride + (x)] != space) {   // if not on right side
        ave[2] += (image->buf[(y+1)*image->stride+ x] & 0xff);
        ave[1] += (image->buf[(y+1)*image->stride+ x]>>8 & 0xff);
        ave[0] += (image->buf[(y+1)*image->stride+ x]>>16 & 0xff);
        // printf("4th = 0x%x\n", image->buf[(y+1)*image->stride+ x]);

        num_pixels++;
    }    

    if(num_pixels ==0)
        return(0xFF0000FF);
    // average the three values and for an ABGR 
    int average = 0xFF000000;
    average |= (ave[2]/num_pixels << 0);
    average |= (ave[1]/num_pixels << 8);
    average |= (ave[0]/num_pixels << 16);
    // printf("average = 0x%x\n", average);
    return(average);
}

void rgb_to_hsv(uint32_t abgr, double* hsv)
{
    double rgb[3];
    rgb[2] = (double)(abgr & 0xff) / 255.0f;
    rgb[1] = (double)(abgr >> 8 & 0xff) / 255.0f;
    rgb[0] = (double)(abgr >> 16 & 0xff) / 255.0f;

    double rgb_min, rgb_max, rgb_range;
    rgb_min = MIN3(rgb[2], rgb[1], rgb[0]);
    rgb_max = MAX3(rgb[2], rgb[1], rgb[0]);
    rgb_range = rgb_max - rgb_min;

    //Compute Value
    hsv[2] = rgb_max;
    if (hsv[2] == 0) {
        hsv[1] = hsv[0] = 0;
        return;
    }

    //Compute Saturation
    hsv[1] = rgb_range/rgb_max;
    if (hsv[1] == 0) {
        hsv[0] = 0;
        return;
    }

    //Compute Hue
    if (rgb_max == rgb[2]) {
        hsv[0] = 60*(fmod(((rgb[1]-rgb[0])/rgb_range),6));
    } else if (rgb_max == rgb[1]) {
        hsv[0] = 60*(((rgb[0]-rgb[2])/rgb_range)+2);
    } else if (rgb_max == rgb[0]) {
        hsv[0] = 60*(((rgb[2]-rgb[1])/rgb_range)+4);
    } else {
        //how did you get here???
        assert(0);
    }

    return;
}

int changeSaturation(int abgr, double change)
{
    double R = (double)(abgr & 0xff);
    double G = (double)(abgr >> 8 & 0xff);
    double B = (double)(abgr >> 16 & 0xff);

    double P=sqrt(
    R*R*Pr+
    G*G*Pg+
    B*B*Pb ) ;

    R = P + (R-P)*change;
    G = P + (G-P)*change;
    B = P + (B-P)*change;

    int new_abgr = 0xFF000000;
    new_abgr |= (((int)R & 0xff) << 0);
    new_abgr |= (((int)G & 0xff) << 8);
    new_abgr |= (((int)B & 0xff) << 16);
    new_abgr |= ((0xff) << 24);

    return new_abgr;
}


g_node_t* resolve_r(g_node_t* n) {
    if(n->parent_id == n->id) return n;

    n->parent_node = resolve_r(n->parent_node);
    n->parent_id = n->parent_node->id;
    return n->parent_node;
}

// if two nodes have conflicting parents, choose 1 parent to make overall parent
static void connect(g_node_t* n1, g_node_t* n2) {
    g_node_t* n1_parent = resolve_r(n1);          
    g_node_t* n2_parent = resolve_r(n2);

    n1_parent->parent_node = n2_parent;
    n1_parent->parent_id = n2_parent->id;       //n2 becomes parent of n1
    // n1's child nodes, are not changed to n2
}


int compare_pix(color_t a, color_t b)
{
    return(abs(a.r - b.r) + 
            abs(a.g - b.g) + 
            abs(a.b - b.b));
}





grad_t get_pix_gradient(image_u32_t* im, int idx)
{
    int abgr = im->buf[idx];
    pixel_t curr_pix = {{(abgr >> 24) & 0xff, (abgr >> 16) & 0xff, 
                            (abgr >> 8) & 0xff, (abgr) & 0xff}, {0, 0},
                            {-1, -1}};

    // add gradient from left
    int tmp_buf = im->buf[idx - 1];
    color_t left_color = {(tmp_buf >> 24) & 0xff, (tmp_buf >> 16) & 0xff, 
                            (tmp_buf >> 8) & 0xff, (tmp_buf) & 0xff};    
    int diff = compare_pix(left_color, curr_pix.color);
    curr_pix.grad.x += (-1)*diff;

    // add gradient from right
    tmp_buf = im->buf[idx + 1];
    color_t right_color = {(tmp_buf >> 24) & 0xff, (tmp_buf >> 16) & 0xff, 
                            (tmp_buf >> 8) & 0xff, (tmp_buf) & 0xff};    
    diff = compare_pix(right_color, curr_pix.color);
    curr_pix.grad.x += diff;  

    // add gradient from top
    tmp_buf = im->buf[idx + im->stride];
    color_t top_color = {(tmp_buf >> 24) & 0xff, (tmp_buf >> 16) & 0xff, 
                            (tmp_buf >> 8) & 0xff, (tmp_buf) & 0xff};    
    diff = compare_pix(top_color, curr_pix.color);
    curr_pix.grad.y += diff;

    // add gradient from bottom
    tmp_buf = im->buf[idx - im->stride];
    color_t bottom_color = {(tmp_buf >> 24) & 0xff, (tmp_buf >> 16) & 0xff, 
                            (tmp_buf >> 8) & 0xff, (tmp_buf) & 0xff};    
    diff = compare_pix(bottom_color, curr_pix.color);
    curr_pix.grad.y += (-1)*diff;

    return(curr_pix.grad);
}

double dot_product(grad_t a, grad_t b)
{
    return(a.x*b.x + a.y*b.y);
}


// convert image: pixels -> gradient nodes
zhash_t* build_gradient_image(image_u32_t* im)
{

    // idx -> g_node_t
    zhash_t* node_map = zhash_create(sizeof(uint32_t), sizeof(g_node_t*), 
            zhash_uint32_hash, zhash_uint32_equals);

    for(int y = 0; y < (im->height-1); y++) {
        for(int x = 0; x < (im->width-1); x++) {

            int idx = y*im->stride + x;
            g_node_t* n = malloc(sizeof(g_node_t));
            n->id = idx;
            n->parent_id = idx;
            n->parent_node = n;
            n->loc.x = x;
            n->loc.y = y;
            n->grad = get_pix_gradient(im, idx);

            if(zhash_put(node_map, &idx, &n, NULL, NULL) == 1)
                assert(0); 
        }
    }

    zarray_t* items = zhash_values(node_map);

    for(int i = 0; i < zarray_size(items); i++)
    {
        g_node_t* n;
        zarray_get(items, i, &n);
        printf("(%d, %d)   grad: (%lf, %lf) \n", n->loc.x, n->loc.y,
                n->grad.x, n->grad.y);
    }
    return(node_map);
}


// connect similar gradient nodes, close to each other
zhash_t* connect_nodes(image_u32_t* im, double error)
{
    
    zhash_t* node_map = build_gradient_image(im);

    for(int y = 0; y < (im->height-1); y++) {
        for(int x = 0; x < (im->width-1); x++) {
            
            int idx = y*im->stride + x;
            g_node_t* tmp_n;
            g_node_t* curr_n;
            zhash_get(node_map, &idx, &curr_n);

            // TODO: may need to check more areas
            // check left 
            if(x > 0) {
                int tmp_idx = idx - 1;
                if(zhash_get(node_map, &tmp_idx, &tmp_n) != 1)
                    assert(0);
                if(dot_product(curr_n->grad, tmp_n->grad) < error) {
                    connect(curr_n, tmp_n);                    
                }
            }
            // check bottom 
            if(y > 0) { 
                int tmp_idx = idx - im->stride; 
                zhash_get(node_map, &tmp_idx, &tmp_n);
                if(dot_product(curr_n->grad, tmp_n->grad) < error) {
                    connect(curr_n, tmp_n);  
                }                  
            }
        }
    }
    return(node_map);
}

// iterate over every node, putting into an arary based on its parent
// returns zhash of zarray that contain g_node
zarray_t* form_objects(zhash_t* node_map, image_u32_t* im)
{
    // node->parent_id -> zarray_t  (full of nodes)
    zhash_t* obj_hash = zhash_create(sizeof(uint32_t), sizeof(zarray_t*), 
            zhash_uint32_hash, zhash_uint32_equals);

    for(int y = 0; y < (im->height-1); y++) {
        for(int x = 0; x < (im->width-1); x++) {

            int idx = y*im->stride + x;
            g_node_t* n;
            zhash_get(node_map, &idx, &n);
            zarray_t* node_arr;

            // check to see if zarray already created, else create it
            if(zhash_get(obj_hash, &n->parent_id, &node_arr) != 1) {
                node_arr = zarray_create(sizeof(g_node_t*));
                if(zhash_put(obj_hash, &n->parent_id, &node_arr, NULL, NULL) == 1)
                    assert(0);
            }
            zarray_add(node_arr, &n);
        }
    }
}



// create lines from gradient objects

