#include "camera_util.h"
#include "limits.h"
#include <stdlib.h>
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
    sprintf(file_name, "/home/jjrasche/finalProject/src/finproj/pic%d.pnm", num);

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

void rgb_to_hsv(uint32_t abgr, hsv_t* hsv)
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
    hsv->val = rgb_max;
    if (hsv->val == 0) {
        hsv->sat = hsv->hue = 0;
        return;
    }

    //Compute Saturation
    hsv->sat = rgb_range/rgb_max;
    if (hsv->sat == 0) {
        hsv->hue = 0;
        return;
    }

    //Compute Hue
    if (rgb_max == rgb[2]) {
        hsv->hue = 60*(fmod(((rgb[1]-rgb[0])/rgb_range),6));
    } else if (rgb_max == rgb[1]) {
        hsv->hue = 60*(((rgb[0]-rgb[2])/rgb_range)+2);
    } else if (rgb_max == rgb[0]) {
        hsv->hue = 60*(((rgb[2]-rgb[1])/rgb_range)+4);
    } else {
        //how did you get here???
        assert(0);
    }

    return;
}

uint32_t hsv_to_rgb(hsv_t hsv)
{
    double h, f, p, q, t;
    int i;
    abgr_t out;
    uint32_t ret = 0xFF000000;

    if(hsv.sat == 0.0) {  //grey    
        ret += (int)(hsv.val*0xFF);
        ret += (int)(hsv.val*0xFF) << 8;
        ret += (int)(hsv.val*0xFF) << 16;
        return(ret);
    }


    h = hsv.hue/60;            // sector 0 to 5
    i = floor(h);
    f = h - i;                  // factorial part of h
    p = hsv.val * ( 1 - hsv.sat );
    q = hsv.val * ( 1 - hsv.sat * f );
    t = hsv.val * ( 1 - hsv.sat * ( 1 - f ) );
    switch( i ) {
        case 0:
            out.r = hsv.val * 0xFF;
            out.g = t * 0xFF;
            out.b = p * 0xFF;
            break;
        case 1:
            out.r = q * 0xFF;
            out.g = hsv.val * 0xFF;
            out.b = p * 0xFF;
            break;
        case 2:
            out.r = p * 0xFF;
            out.g = hsv.val * 0xFF;
            out.b = t * 0xFF;
            break;
        case 3:
            out.r = p * 0xFF;
            out.g = q * 0xFF;
            out.b = hsv.val * 0xFF;
            break;
        case 4:
            out.r = t * 0xFF;
            out.g = p * 0xFF;
            out.b = hsv.val * 0xFF;
            break;
        default:        // case 5:
            out.r = hsv.val * 0xFF;
            out.g = p * 0xFF;
            out.b = q * 0xFF;
    }
    // rgb[2] = (double)(abgr & 0xff) / 255.0f;
    // rgb[1] = (double)(abgr >> 8 & 0xff) / 255.0f;
    // rgb[0] = (double)(abgr >> 16 & 0xff) / 255.0f;

    ret += out.r;
    ret += out.g << 8;
    ret += out.b << 16;

    // printf("red:%x , blue:%x , green:%x ,  out:%x\n", out.r, out.b, out.g, ret);

    return(ret);
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

// image is taken flipped, if one wants to process
// the corrected image, must first flip it vertically 
// it was hard to analyze how the image processing was working,
// because I had to flip the expectations in my head, also
// there were many places where I was individually flipping
// a pixel or part of an image, so this consolidates that
void flip_image(image_u32_t* im, image_u32_t* flipped)
{
    for(int y = 0; y < im->height; y++) {
        for(int x = 0; x < im->width; x++) {
            int orig_idx = y * im->stride + x;
            int flip_idx = abs(y - (im->height-1)) * im->stride + x;
            flipped->buf[flip_idx] = im->buf[orig_idx];
        }
    }
}

g_node_t* resolve_g(g_node_t* n) {
    if(n->parent_id == n->id) return n;

    n->parent_node = resolve_g(n->parent_node);
    n->parent_id = n->parent_node->id;
    return n->parent_node;
}

// if two nodes have conflicting parents, choose 1 parent to make overall parent
static void connect_g(g_node_t* n1, g_node_t* n2) {
    g_node_t* n1_parent = resolve_g(n1);          
    g_node_t* n2_parent = resolve_g(n2);

    n1_parent->parent_node = n2_parent;
    n1_parent->parent_id = n2_parent->id;       //n2 becomes parent of n1
    // n1's child nodes, are not changed to n2
}

double hsv_val_diff(abgr_t a, abgr_t b)
{
    // return(sqrt((a.r-b.r)*(a.r-b.r)+
    //             (a.b-b.b)*(a.b-b.b)+
    //             (a.g-b.g)*(a.g-b.g)));

    // difference in value in HSV should give gray scale
    uint8_t max_a = MAX3(a.r, a.b, a.g);
    uint8_t max_b = MAX3(b.r, b.b, b.g);
    return(max_a - max_b);


}


// returns {-1, -1} if lines do not intersect inside the image box
// loc_t find_line_intersect()
// {
//    double det = A1*B2 - A2*B1
//     if(det == 0){
//         //Lines are parallel
//     }else{
//         double x = (B2*C1 - B1*C2)/det
//         double y = (A1*C2 - A2*C1)/det
//     }
// }

grad_t get_pix_gradient(image_u32_t* im, int x, int y)
{
    int idx = y*im->stride + x;
    int abgr = im->buf[idx];
    pixel_t curr_pix = {    .abgr = {  (abgr >> 24) & 0xff, 
                                        (abgr >> 16) & 0xff, 
                                        (abgr >> 8) & 0xff, 
                                        (abgr) & 0xff}, 
                            .grad = {0, 0},
                            .loc = {-1, -1}
                        };

    // // add gradient from left
    // if(x > 0) {
    //     int left_buf = im->buf[idx - 1];
    //     // int right_buf = im->buf[idx + 1];

    //     curr_pix.grad.x = abgr - left_buf;
    // }

    // // add gradient from bottom
    // if(y > 0) {
    //     // int top_buf = im->buf[idx + im->stride];
    //     int bottom_buf = im->buf[idx - im->stride];

    //     curr_pix.grad.y = abgr - bottom_buf;
    // }

    // add gradient from left
    if(x > 0) {
        int left_buf = im->buf[idx - 1];
        abgr_t left_color = {(left_buf >> 24) & 0xff, (left_buf >> 16) & 0xff, 
                                (left_buf >> 8) & 0xff, (left_buf) & 0xff};    
        double diff = hsv_val_diff(left_color, curr_pix.abgr);
        curr_pix.grad.x += diff;
    }

    // add gradient from bottom
    if(y > 0) {
        int bottom_buf = im->buf[idx - im->stride];
        abgr_t bottom_color =  {(bottom_buf >> 24) & 0xff, (bottom_buf >> 16) & 0xff, 
                                (bottom_buf >> 8) & 0xff, (bottom_buf) & 0xff};    
        double diff = hsv_val_diff(bottom_color, curr_pix.abgr);
        curr_pix.grad.y += diff;
    }

    // add gradient from bottom-left
    // if(x > 0 && y > 0) {
    //     int bl_buf = im->buf[idx - 1 - im->stride];
    //     abgr_t bl_color = {(bl_buf >> 24) & 0xff, (bl_buf >> 16) & 0xff, 
    //                             (bl_buf >> 8) & 0xff, (bl_buf) & 0xff};    
    //     double diff = compare_pix(bl_color, curr_pix.abgr);
    //     //TODO: think about the amount of weight the corner pix should get
    //     // current thought is equal 
    //     curr_pix.grad.x += (-1)*diff;
    //     curr_pix.grad.y += (-1)*diff;
    // }  

    // // add gradient from right
    // if(x < (im->width-1)) {
    //     int right_buf = im->buf[idx + 1];
    //     abgr_t right_color = {(right_buf >> 24) & 0xff, (right_buf >> 16) & 0xff, 
    //                             (right_buf >> 8) & 0xff, (right_buf) & 0xff};    
    //     double diff = compare_pix(right_color, curr_pix.abgr);
    //     curr_pix.grad.x += diff;  
    // }

    // // add gradient from top
    // if(y < (im->height-1)) {
    //     int top_buf = im->buf[idx + im->stride];
    //     abgr_t top_color = {(top_buf >> 24) & 0xff, (top_buf >> 16) & 0xff, 
    //                             (top_buf >> 8) & 0xff, (top_buf) & 0xff};    
    //     double diff = compare_pix(top_color, curr_pix.abgr);
    //     curr_pix.grad.y += diff;
    // }

    return(curr_pix.grad);
}

// convert image: pixels -> gradient nodes
zhash_t* build_gradient_image(image_u32_t* im)
{

    // idx -> g_node_t
    zhash_t* node_map = zhash_create(sizeof(uint32_t), sizeof(g_node_t*), 
            zhash_uint32_hash, zhash_uint32_equals);

    for(int y = 0; y < im->height; y++) {
        for(int x = 0; x < im->width; x++) {

            int idx = y*im->stride + x;
            g_node_t* n = malloc(sizeof(g_node_t));
            n->id = idx;
            n->parent_id = idx;
            n->parent_node = n;
            n->loc.x = x;
            n->loc.y = y;
            n->grad = get_pix_gradient(im, x, y);

            if(zhash_put(node_map, &idx, &n, NULL, NULL) == 1)
                assert(0); 
        }
    }
    return(node_map);
}


uint32_t mag_to_gray(g_node_t* n, int add)
{
    // calc gradient magnitude
    double mag = sqrt(n->grad.x*n->grad.x + n->grad.y*n->grad.y);

    // normalize to 0xff or 255
    int brightness = (mag*255) / 625;
    // assert(brightness < 256);
    if((brightness + add) <= 0xFF)  brightness += add;

    uint32_t color = 0xff000000;
    color += (brightness &0xFF) << 16;
    color += (brightness &0xFF) << 8;
    color += (brightness &0xFF);

    // if(mag > 0) {
        // printf("color:%x  ,  mag:(%lf, %lf)  loc:(%d, %d) \n", 
        //         color, n->grad.x, n->grad.y, n->loc.x, n->loc.y);
    // }
    
    return(color);
}

// convert to gray scale image where the whiter a pixel, 
// the larger the gradient
void convert_to_grad_image(image_u32_t* im, int add)
{
    zhash_t* node_map = build_gradient_image(im);

    for(int y = 0; y < (im->height-1); y++) {
        for(int x = 0; x < (im->width-1); x++) {
            int idx = y * im->stride + x;

            g_node_t* tmp;
            zhash_get(node_map, &idx, &tmp);
            
            // uint32_t a = tmp->grad.x & 0xffffff;
            // uint32_t b = tmp->grad.y & 0xffffff;
            // uint32_t mag = sqrt((a*a)+(b*b));
            im->buf[idx] = mag_to_gray(tmp, add);
            // printf("(%d, %d)  (%d, %d) val:%d\n", x, y, a, b, mag);
            free(tmp);
        }
    }  
    // zhash_vmap_keys(node_map, free);
    zhash_destroy(node_map);
}

int in_range(image_u32_t* im, int x, int y)
{
    if(y < 0 || x < 0)  return(0);
    else if(y > (im->height-1) || x > (im->width-1)) return(0);
    else return(1);
}

int radius = 2;
int gaussian_weights[25] =             {1, 4, 7, 4, 1, 
                                        4, 16, 26, 16, 4, 
                                        7, 26, 41, 26, 7, 
                                        4, 16, 26, 16, 4, 
                                         1, 4, 7, 4, 1};

uint32_t get_pix_gaussian_ave(image_u32_t* im, loc_t pix)
{
    // Do I need a in this or can assume = 0xFF
    double r=0, g=0, b=0;
    int i=0;
    int gaus_divisor = 0;

    // printf("\n\n");
    for(int y = (-1)*radius; y <= radius; y++) {
        for(int x = (-1)*radius; x <= radius; x++) {
            // get the rgb values of the pixel 
            if(in_range(im, (pix.x+x), (pix.y+y))) {
                uint32_t tmp = im->buf[(pix.y+y)*im->stride + (pix.x+x)];
                r += ((tmp >> 0) & 0xFF) * gaussian_weights[i];
                g += ((tmp >> 8) & 0xFF) * gaussian_weights[i];
                b += ((tmp >> 16) & 0xFF) * gaussian_weights[i];

                gaus_divisor += gaussian_weights[i];
                
                // printf("pix:(%d, %d)  neighbor:(%d, %d)   %d:%d    %x: (%lf, %lf, %lf)  \n", 
                //         pix.x, pix.y,(pix.x+x), (pix.y+y), i, gaussian_weights[i],
                //         tmp, r, g, b);
            }
            i++;
        }
    }
    // reform blurry pix value
    uint32_t out = 0xFF000000;
    out += (int)(r/gaus_divisor) & 0xFF << 0;
    out += ((int)(g/gaus_divisor) & 0xFF) << 8;
    out += ((int)(b/gaus_divisor) & 0xFF) << 16;

    uint32_t prev = im->buf[pix.y*im->stride + pix.x];
    // printf("before:(%d, %d, %d)  after:(%d, %d, %d)   %d\n\n",
    //         ((prev >> 0) & 0xFF), ((prev >> 8) & 0xFF), ((prev >> 16) & 0xFF),
    //          (int)(r/gaus_divisor) & 0xFF, (int)(g/gaus_divisor) & 0xFF, 
    //          (int)(b/gaus_divisor) & 0xFF, gaus_divisor);
    return(out);

}

image_u32_t* blur_image(image_u32_t* im, int num_passes) 
{
    if(num_passes == 0) return(im);
    image_u32_t* hold_ptr = im;
    image_u32_t* blur_im = image_u32_create(im->width, im->height);
    int r = 3;

    for(int i = 0; i < num_passes; i++)
    {
        for(int y = 0; y < (im->height-1); y++) {
            for(int x = 0; x < (im->width-1); x++) {
                int idx = y * im->stride + x;  
                loc_t tmp = {x, y};  
                uint32_t val = get_pix_gaussian_ave(im, tmp);
                blur_im->buf[idx] = val;
            }
        }
        im = blur_im;
    }
    image_u32_destroy(hold_ptr);
    return(blur_im);
}

uint32_t grad_dir_to_color(g_node_t* n, double min_mag)
{
    // convert from -PI -> PI   to   0 -> 360 degrees 
    double angle = atan2(n->grad.y, n->grad.x);
    double mag = sqrt(n->grad.x*n->grad.x + n->grad.y*n->grad.y);
    if(mag < min_mag)
        return(0xFFFFFFFF);
    angle += PI;
    angle *= 180/PI;
    hsv_t hsv = {angle, 1,1};

    uint32_t out = hsv_to_rgb(hsv);
    // printf("loc:(%d, %d)    grad:(%lf, %lf)  rgb:(%d, %d, %d)\n",
    //         n->loc.x, n->loc.y, n->grad.x, n->grad.y, 
    //         out&0xFF, (out>>8)&0xFF, (out>>16)&0xFF);
    return(out);
}

void convert_to_grad_dir_image(image_u32_t* im, int bright, double min_mag)
{
    zhash_t* node_map = build_gradient_image(im);

    for(int y = 0; y < (im->height-1); y++) {
        for(int x = 0; x < (im->width-1); x++) {
            int idx = y * im->stride + x;

            g_node_t* tmp;
            zhash_get(node_map, &idx, &tmp);
    
            im->buf[idx] = grad_dir_to_color(tmp, min_mag);
            free(tmp);
        }
    }  
    zhash_destroy(node_map);
}

double compare_nodes(grad_t a, grad_t b, threshold_metrics_t thresh)
{
    double mag_a = sqrt(a.x*a.x+a.y*a.y);
    if(mag_a < thresh.min_mag) {
        return(MAXDOT);
    }
    double mag_b = sqrt(b.x*b.x+b.y*b.y);
    if(mag_b < thresh.min_mag) {
        return(MAXDOT);
    }
    // if both above a magnitude, return their dot product
    double ret = fabs(atan2(a.y, a.x) - atan2(b.y, b.x));//* (180/PI);
    if(ret > 360)
    {
        int i = 0;
    }
    if(ret < (-1)*PI) {
        ret += 2*PI;
    }
    if(ret > PI) {
        ret -= 2*PI;
    }
    // if(fabs(ret)*(180/PI) < thresh.max_grad_diff) {
    //     printf("a:(%lf, %lf, %lf)   b(%lf, %lf, %lf)   ret:(%lf, %lf)\n",
    //             a.x, a.y, mag_a, b.x, b.y, mag_b, 
    //             fabs(atan2(a.y, a.x) - atan2(b.y, b.x)), fabs(ret)*(180/PI));
    // }
    return(fabs(ret)*(180/PI));
}

void make_connection(zhash_t* node_map, g_node_t* curr_n, int check_idx, 
                        threshold_metrics_t thresh)
{
    g_node_t* tmp_n;
    if(zhash_get(node_map, &check_idx, &tmp_n) != 1)
        assert(0);
    if(compare_nodes(curr_n->grad, tmp_n->grad, thresh) 
                    < thresh.max_grad_diff ) {
        // printf("L curr:(%d, %d)--(%lf, %lf),  tmp:(%d, %d)--(%lf, %lf)   dot:%lf\n"
        //         , curr_n->loc.x, curr_n->loc.y, curr_n->grad.x, curr_n->grad.y, 
        //         tmp_n->loc.x, tmp_n->loc.y, tmp_n->grad.x, tmp_n->grad.y, 
        //         compare_nodes(curr_n->grad, tmp_n->grad, thresh.min_mag));
        connect_g(curr_n, tmp_n);                    
    }   
}

// connect similar gradient nodes, close to each other
zhash_t* connect_nodes(image_u32_t* im, threshold_metrics_t thresh)
{
    zhash_t* node_map = build_gradient_image(im);

    for(int y = 0; y < (im->height-1); y++) {
        for(int x = 0; x < (im->width-1); x++) {
            
            int idx = y*im->stride + x;
            g_node_t* curr_n;
            zhash_get(node_map, &idx, &curr_n);

            // check left 
            if(x > 0) 
                make_connection(node_map, curr_n, (idx - 1), thresh);
            // check bottom 
            if(y > 0)
                make_connection(node_map, curr_n, (idx - im->stride), thresh);               
            // check bottom left 
            if(x > 0 && y > 0)
                make_connection(node_map, curr_n, (idx - (1 + im->stride)), thresh);
        }
    }
    return(node_map);
}

// iterate over every node, putting into an arary based on its parent
// returns zhash of zarray that contain g_node
zhash_t* form_objects(image_u32_t* im, threshold_metrics_t thresh)
{
    // node->parent_id -> zarray_t  (full of nodes)
    zhash_t* obj_hash = zhash_create(sizeof(uint32_t), sizeof(zarray_t*), 
            zhash_uint32_hash, zhash_uint32_equals);
    zhash_t* node_map = connect_nodes(im, thresh);

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
    zhash_destroy(node_map);
    return(obj_hash);
}


line_t build_line(image_u32_t* im, zarray_t* node_arr)
{
    line_t l = {{-1,-1}, {-1,-1}};
    int n = zarray_size(node_arr);
    g_node_t* node;
    // double Sx=0, Sxx=0, Sxy=0, Sy=0, Syy=0;
    double Sx=0, Stt=0, Sts=0, Sy=0;

    double m, b;

    for (int i = 0; i < n; ++i)
    {
        zarray_get(node_arr, i, &node);
        Sx += node->loc.x;
        Sy += node->loc.y;
    }
    for (int i = 0; i < n; ++i)
    {
        zarray_get(node_arr, i, &node);
        double t = node->loc.x - Sx/n;
        Stt += t*t;
        Sts += t*node->loc.y;
    }
    m = Sts/Stt;
    b = (Sy - Sx*m)/n;

    // for(int i = 0; i < n; i++)
    // {
    //     zarray_get(node_arr, i, &node);
    //     Sx += node->loc.x;
    //     Sxx += node->loc.x * node->loc.x;
    //     Sxy += node->loc.y * node->loc.x;
    //     Sy += node->loc.y;
    //     Syy += node->loc.y * node->loc.y;
    // }
    // m = (n*Sxy - Sx*Sy) / (n*Sxx - Sx*Sx);
    // b = (Sy/n) - (m/n)*Sx;

    double largest_dist = INT_MIN;
    loc_t largest_loc = {-1, -1};
    double smallest_dist = INT_MAX;
    loc_t smallest_loc = {-1, -1};

    // loc_t find_line_intersect(y, m);

    // find point on line to compare every node to, simple solution = set x = im->width/2
    loc_t Q = {im->width/2, m*im->width/2 + b};
    // get line unit vector,  y=mx+b  --> vector = (1,m)
    double mag = sqrt(1 + m*m);
    vec_t u = {1/mag, m/mag};
    // assert(sqrt((u.x*u.x) + (u.y*u.y)) == 1);       // length of u should be 1
    // printf("\nunit vec:(%lf(%lf), %lf)  Point(%d, %d)\n", u.x, 1/mag, u.y, Q.x, Q.y);
    for(int i = 0; i < n; i++)
    {
        zarray_get(node_arr, i, &node);

        // find the projection of point onto line
        vec_t QP = {Q.x - node->loc.x, Q.y - node->loc.y};
        double dist = u.x * QP.x  +  u.y * QP.y; 

        // printf("dist:%lf  small:%lf  larg:%lf   QP:(%lf, %lf)  loc:(%d, %d) \n", 
        //        dist, smallest_dist, largest_dist, QP.x, QP.y, node->loc.x, node->loc.y);
        if(dist < smallest_dist) {
            smallest_dist = dist;
            smallest_loc.x = node->loc.x;
            smallest_loc.y = node->loc.y;
        }
        if(dist > largest_dist) {
            largest_dist = dist;
            largest_loc.x = node->loc.x;
            largest_loc.y = node->loc.y;
        }
    }

    // find end pixels
    l.start.x = largest_loc.x;
    l.start.y = largest_loc.y;

    l.end.x = smallest_loc.x;
    l.end.y = smallest_loc.y;

    if((smallest_loc.x <= 3 && smallest_loc.y <= 3)||
        (largest_loc.x <= 3 && largest_loc.y <= 3))
    {
        int i = 0;
    }
    if(fabs(m) < .005)
    {
        int i = 0;
    }

    return(l);
}

int color_match(int comp, hsv_t base, hsv_t max_error)
{
    // convert comp to hsv
    hsv_t comp_hsv;
    rgb_to_hsv(comp, &comp_hsv);

    hsv_t error = {fabs(base.hue - comp_hsv.hue), 
                        fabs(base.sat - comp_hsv.sat), 
                        fabs(base.val - comp_hsv.val)};

    if(error.hue < max_error.hue && error.sat < max_error.sat && error.val < max_error.val)
        return(1);
    return(0);
}


// compare all 8 pixels around the endpoints of the line
// if one of the pixels is the correct color return true
int right_color(image_u32_t* im, hsv_t color, hsv_t max_error, line_t* l)
{
    // need to check if in bounds of image 
    int idx_max = im->stride * im->width;

    // check start 
    int idx = l->start.y * im->stride + l->start.x;
    int idx_to_checkS[8] = {idx-1, idx-im->stride-1, idx-im->stride, idx-im->stride+1,
                        idx+1, idx+im->stride+1, idx+im->stride, idx+im->stride-1};
    for(int i = 0; i < 8; i++)
    {
        if(idx_to_checkS[i] < idx_max && idx_to_checkS[i] > 0)
        {
            if(color_match(im->buf[idx_to_checkS[i]], color, max_error))
            {
                hsv_t hsv;
                double hsv2[3];
                rgb_to_hsv(im->buf[idx_to_checkS[i]], &hsv);
                return(1);
            }
        }
    }

    // check end 
    idx = l->end.y * im->stride + l->end.x;
    int idx_to_checkE[8] = {idx-1, idx-im->stride-1, idx-im->stride, idx-im->stride+1,
                        idx+1, idx+im->stride+1, idx+im->stride, idx+im->stride-1};
    for(int i = 0; i < 8; i++)
    {
        if(idx_to_checkE[i] < idx_max && idx_to_checkE[i] > 0)
        {
            if(color_match(im->buf[idx_to_checkE[i]], color, max_error))
                return(1);
        }
    }
    return(0);
}

void blank_image(image_u32_t* im)
{
    for(int y = 0; y < (im->height-1); y++) {
        for(int x = 0; x < (im->width-1); x++) {
            im->buf[y+im->stride + x] = 0xFF000000;
        }
    }
}

// color all pixels in array the same color
void color_blob_image(image_u32_t* im, zarray_t* arr)
{
    g_node_t* n;
    zarray_get(arr, 0, &n);
    srand(n->parent_id);
    uint32_t color = 0xFF000000 | (rand() & 0xFFFFFF);
    for(int i = 0; i < zarray_size(arr); i++)
    {
        zarray_get(arr, i, &n);
        int x = n->id % im->stride;
        int y = n->id / im->stride;
        // printf("pix: (%d, %d)     color:%x\n", x, adj_y, color);
        im->buf[y*im->stride + x] = color;
    }
}



// create lines from gradient objects
// qualify objects, and build line
zarray_t* form_lines(image_u32_t*im, threshold_metrics_t thresh, image_u32_t* seg_image)
{
    zhash_t* obj_hash = form_objects(im, thresh);
    zarray_t* arr_arr = zhash_values(obj_hash);
    zarray_t* lines = zarray_create(sizeof(line_t));

    if(seg_image != NULL) {
        blank_image(seg_image);
    }
    // iterate through objects, qualify and form lines
    for(int i = 0; i < zarray_size(arr_arr); i++)
    {
        zarray_t* obj_arr;
        zarray_get(arr_arr, i, &obj_arr);

        if(zarray_size(obj_arr) < thresh.min_size) {
            zarray_vmap(obj_arr, free);
            zarray_destroy(obj_arr);
            continue;
        }        
        line_t l = build_line(im, obj_arr);
        // only add line if a certian color is on one side
        // if(!right_color(im, thresh.obj_hsv, thresh.color_error, &l))
        // {
        //     zarray_vmap(obj_arr, free);
        //     zarray_destroy(obj_arr);
        //     continue;      
        // }

        l.nodes = obj_arr;
        zarray_add(lines, &l);
        if(seg_image != NULL) {
            color_blob_image(seg_image, obj_arr);
        }
    }
    zarray_destroy(arr_arr);
    zhash_destroy(obj_hash);
    return(lines);
}










// void test_build_line()
// {
//     double x[15] = {1.47  ,  1.50  ,  1.52  ,  1.55 ,   1.57  ,  1.60    ,1.63 ,   1.65  ,  1.68   , 1.70  ,  1.73  ,  1.75 ,   1.78  ,  1.80  ,  1.83};
//     double y[15] = {52.21  , 53.12 ,  54.48 ,  55.84  , 57.20  , 58.57  , 59.93   ,61.29  , 63.11  , 64.47  , 66.28  , 68.10  , 69.92 ,  72.19  , 74.46};
//     zarray_t* arr = zarray_create(sizeof(g_node_t*));
    
//     for(int i = 0; i< 15; i++)
//     {
//         g_node_t* tmp = malloc(sizeof(g_node_t));
//         tmp->loc.x = x[i];
//         tmp->loc.y = y[i];
//         zarray_add(arr, &tmp);
//     }
//     line_t l = build_line(arr);
// }