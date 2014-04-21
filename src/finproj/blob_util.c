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

void add_arr_of_locs_to_buffer(zarray_t* arr, vx_buffer_t* buf, double size, 
                                const float* color, metrics_t met);
void add_circle_to_buffer(vx_buffer_t* buf, double size, loc_t loc, const float* color);

int get_slope(loc_t a , loc_t b, int add_line, int reverse, vx_buffer_t* buf) 
{

    if(add_line) {
        int npoints = 2;
        float points[npoints*3];

        points[0] = a.x;
        points[1] = a.y;
        points[2] = 0;
        points[3] = b.x;
        points[4] = b.y;
        points[5] = 0;

        vx_resc_t *verts = vx_resc_copyf(points, npoints*3);
        vx_buffer_add_back(buf, vxo_pix_coords(VX_ORIGIN_BOTTOM_LEFT,
                                            vxo_lines(verts, npoints, GL_LINES, 
                                            vxo_points_style(vx_blue, 2.0f))));   
    }

    if(reverse) {
        if((a.y - b.y) == 0)
            return((a.x - b.x)/.0001);
        int ret = (a.x - b.x) /  (a.y - b.y); 
        return(ret);
    }
    if((a.x - b.x) == 0)
        return((a.y - b.y)/.0001);
    int ret = (a.y - b.y) /  (a.x - b.x); 
    return(ret); 
}

double dist_from_point_to_line(loc_t* p1, loc_t* p2, loc_t* check)
{
    double dist = ((p2->x - p1->x)*(p1->y - check->y) - (p1->x - check->x)*(p2->y-p1->y)) /
                    (sqrt((p2->x - p1->x)*(p2->x - p1->x) + (p2->y - p1->y)*(p2->y - p1->y)));
    return(dist);
}

void add_line_to_buffer(image_u32_t* im, vx_buffer_t* buf, double size, 
                        loc_t p1, loc_t p2, const float* color)
{
    int npoints = 2;          //  line per chart blob
    float points[npoints*3];

    points[0] = p1.x;
    points[1] = p1.y;
    points[2] = 0;
    points[3] = p2.x;
    points[4] = p2.y;
    points[5] = 0;

    // make lines
    vx_resc_t *verts = vx_resc_copyf(points, npoints*3);
    vx_buffer_add_back(buf, vxo_pix_coords(VX_ORIGIN_BOTTOM_LEFT,
                                    vxo_lines(verts, npoints, GL_LINES, 
                                        vxo_points_style(color, size))));
}

void reset_locs(zarray_t* locs)
{
    for(int i = 0; i < zarray_size(locs); i++) {
        loc_t* tmp;
        zarray_get(locs, i, &tmp);
        tmp->valid = 0;
    }
}

loc_t get_line_intersection(loc_t l1p1, loc_t l1p2, loc_t l2p1, loc_t l2p2)
{
    int A1 = l1p2.y - l1p1.y;
    int B1 = l1p1.x - l1p2.x;
    int C1 = (A1*l1p1.x) + (B1*l1p1.y);

    int A2 = l2p2.y - l2p1.y;
    int B2 = l2p1.x - l2p2.x;
    int C2 = (A2*l2p1.x) + (B2*l2p1.y);

    int det = A1*B2 - A2*B1;
    loc_t ret = {-1,-1};
    if(det != 0) {
        ret.x = (B2*C1 - B1*C2)/det;
        ret.y = (A1*C2 - A2*C1)/det;
    }
    return(ret);
}

void extend_lines_to_edge_of_image(image_u32_t* im, loc_t p1, loc_t p2, loc_t* out)
{
    loc_t boundary[8] = {  {1,1}, {1, 10},                               // left-side
                           {1,1}, {10, 1},                               // bottom-side
                           {(im->width-1), 1}, {(im->width-1), 10},      // right-side
                           {1, (im->height-1)}, {10, (im->height-1)}};   // top-side
    int num_point = 0;
    // search over all 4 sides of box for intersection that is within boundary 
    for(int i = 0; i < 4; i++) {    
        loc_t intersect = get_line_intersection(p1, p2, boundary[i*2], boundary[i*2+1]);
        if(in_range(im, intersect.x, intersect.y)) {
            out[num_point++] = intersect;
        }
    }
}

int loc_already_in(loc_t* corners, loc_t loc, int num_corners)
{
    for(int i = 0; i < num_corners; i++) {
        if(corners[i].x == loc.x && corners[i].y == loc.y)
            return(1);
    }
    return(0);
}

void find_corners_from_lines(image_u32_t* im, loc_t* lines, int num_lines, 
                                int* num_corners, loc_t* corners)
{
    // find intersection of each line with the other lines,
    // if intersection is unique, add to the corners
    for(int i = 0; i < num_lines; i++) {
        for(int j = 0; j < num_lines; j++) {
            if(i != j) {
                loc_t intersect = get_line_intersection(lines[i*2], lines[i*2+1], 
                                                        lines[j*2], lines[j*2+1]);
                // printf("(%d, %d)  (%d, %d)   X   (%d, %d)  (%d, %d)  = (%d, %d)\n", 
                //         lines[i*2].x, lines[i*2].y, lines[i*2+1].x, lines[i*2+1].y,
                //         lines[j*2].x, lines[j*2].y, lines[j*2+1].x, lines[j*2+1].y,
                //         intersect.x, intersect.y);
                // intersect inside image
                if(!in_range(im, intersect.x, intersect.y)) continue;
                // not already a corner
                if(loc_already_in(corners, intersect, *num_corners)) continue; 
                // printf("        good\n");
                corners[*num_corners] = intersect;
                (*num_corners)++;
            }
        }
    }
    printf("num_corners: %d\n", *num_corners);
}

line_t find_line_endpoints(zarray_t* loc_arr, loc_t* p1, loc_t* p2)
{
    double largest_dist = INT_MIN;
    loc_t largest_loc = {-1, -1};
    double smallest_dist = INT_MAX;
    loc_t smallest_loc = {-1, -1};

    double slope = get_slope(*p1, *p2, 0, 0,  NULL);
    // find point on line to compare every node to, simple solution = set x = im->width/2
    loc_t* Q = p1;
    // get line unit vector,  y=mx+b  --> vector = (1,m)
    double mag = sqrt(1 + slope*slope);
    vec_t u = {1/mag, slope/mag};
    // assert(sqrt((u.x*u.x) + (u.y*u.y)) == 1);       // length of u should be 1
    // printf("\nunit vec:(%lf(%lf), %lf)  Point(%d, %d)\n", u.x, 1/mag, u.y, Q.x, Q.y);
    for(int i = 0; i < zarray_size(loc_arr); i++)
    {   
        loc_t* tmp;
        zarray_get(loc_arr, i, &tmp);

        // find the projection of point onto line
        vec_t QP = {Q->x - tmp->x, Q->y - tmp->y};
        double dist = u.x * QP.x  +  u.y * QP.y; 

        // printf("dist:%lf  small:%lf  larg:%lf   QP:(%lf, %lf)  loc:(%d, %d) \n", 
        //        dist, smallest_dist, largest_dist, QP.x, QP.y, node->loc.x, node->loc.y);
        if(dist < smallest_dist) {
            smallest_dist = dist;
            smallest_loc.x = tmp->x;
            smallest_loc.y = tmp->y;
        }
        if(dist > largest_dist) {
            largest_dist = dist;
            largest_loc.x = tmp->x;
            largest_loc.y = tmp->y;
        }
    }

    line_t ret = {  .end = largest_loc,
                    .start = smallest_loc,
                    .nodes = NULL };
    return(ret);
}

loc_t* fit_lines(image_u32_t* im, node_t* n, vx_buffer_t* buf, metrics_t met, loc_t* out)
{
    // usleep(2000000);
    srand(time(NULL));
    // isolate valid entries 
    zarray_t* loc_arr = zarray_create(sizeof(loc_t*));
   
    for(int i = 0; i < im->height; i++) {
        if(n->sides[i].leftmost.x == im->width) continue;        // not apart of blob


        loc_t* loc = malloc(sizeof(loc_t));
        loc->x = n->sides[i].leftmost.x;
        loc->y = n->sides[i].leftmost.y;
        loc->valid = 0;
        zarray_add(loc_arr, &loc);
    }
    for(int i = 0; i < im->height; i++) {
        if(n->sides[i].rightmost.x == -1) continue;

        loc_t* loc = malloc(sizeof(loc_t));
        loc->x = n->sides[i].rightmost.x;
        loc->y = n->sides[i].rightmost.y;
        loc->valid = 0;
        zarray_add(loc_arr, &loc);
    }

    // printf("\n\nall\n");
    // for(int i = 0; i < zarray_size(loc_arr); i++)
    // {
    //     loc_t* p1;
    //     zarray_get(loc_arr, i, &p1);
    //     printf("(%d, %d)\n", p1->x, p1->y);
    // }
    // printf("\n\n");
    int iterations = 0;
    int best_score = 0;
    int lines_found = 0;
    loc_t line_match[8];
    int max_iterations = 500;
    while(lines_found < 2  && zarray_size(loc_arr) > met.num_outliers)      // still a lot of points left 
    {  
        if(iterations > max_iterations) break;
        // reset image and array
        // vx_object_t *vim = vxo_image_from_u32(im, 0, 0);
        // vx_buffer_add_back(buf, vxo_pix_coords(VX_ORIGIN_BOTTOM_LEFT, vim));
        add_arr_of_locs_to_buffer(loc_arr, buf, 1.0, vx_black, met);

        int num_outliers = met.num_outliers/met.consensus_accuracy;

        while(best_score < ((zarray_size(loc_arr) - num_outliers)/(4 - lines_found)))
        {
            reset_locs(loc_arr);
            // pick random sample (2 points)
            loc_t* p1;
            loc_t* p2;
            zarray_get(loc_arr, (rand()%zarray_size(loc_arr)), &p1);
            p2 = p1;
            while(p1 == p2)
                zarray_get(loc_arr, (rand()%zarray_size(loc_arr)), &p2);    // don't get same point

            // find consensus score of line from other points 
            // if inside consensus range add 1
            int tmp_score = 0;
            for(int j = 0; j < zarray_size(loc_arr); j++) {
                loc_t* tmp;
                zarray_get(loc_arr, j, &tmp);
                if(fabs(dist_from_point_to_line(p1, p2, tmp)) < (double)met.consensus_accuracy) 
                {

                    tmp->valid = 1;
                    // printf("dist: (%d, %d, %d)  %lf\n", tmp->x, tmp->y, tmp->valid,
                    //         fabs(dist_from_point_to_line(p1, p2, tmp)));
                    tmp_score++;
                }
            }
            // keep best line so far 
            if(tmp_score > best_score) {
                if(lines_found != 0) {      // if 2nd line intersects, throw it out and find another 
                    loc_t intersect = get_line_intersection(line_match[0], line_match[1], *p1, *p2);
                        if(in_range(im, intersect.x, intersect.y)) continue;
                }
                best_score = tmp_score;
                // printf("      score:%d,  %d,  %d  %lf\n", best_score, ((zarray_size(loc_arr)-1)/(4-lines_found)), 
                //                                     zarray_size(loc_arr), 10/met.std_dev_from_square);
                line_match[lines_found*2] = *p1;
                line_match[lines_found*2+1] = *p2;
            }
            iterations++;
            if(iterations > max_iterations) break;
        }
        // loc_t ext_lines[2];
        // extend_lines_to_edge_of_image(im, line_match[lines_found*2], line_match[lines_found*2+1], ext_lines);
        // add_line_to_buffer(im, buf, 2.0, ext_lines[0], ext_lines[1], vx_yellow);
        // delete all points associated with the found line
        zarray_t* endpoints_arr = zarray_create(sizeof(loc_t*));
        for(int i = 0; i < zarray_size(loc_arr); i++) {
            loc_t* tmp;
            zarray_get(loc_arr, i, &tmp);
            // printf("removed: (%d, %d, %d) \n", tmp->x, tmp->y, tmp->valid);
            if(tmp->valid) 
            {
                // add_circle_to_buffer(buf,  1.0, *tmp, vx_red);
                zarray_add(endpoints_arr, &tmp);
                zarray_remove_index(loc_arr, i, 0);
                i--;
            }
        }
        // find endpoints of line 
        loc_t ext_lines[2];
        extend_lines_to_edge_of_image(im, line_match[lines_found*2], line_match[lines_found*2+1], ext_lines);
        line_t endpoints = find_line_endpoints(endpoints_arr, &ext_lines[0], &ext_lines[1]);
        add_circle_to_buffer(buf,  2.0, endpoints.start, vx_red);
        add_circle_to_buffer(buf,  2.0, endpoints.end, vx_red);
        line_match[lines_found*2] = endpoints.start;
        line_match[lines_found*2+1] = endpoints.end;

        lines_found++;
        best_score = 0;
        // vx_buffer_swap(buf);
        // usleep(500000);
    }

    loc_t* ret = calloc(lines_found*2, sizeof(loc_t));
    for(int i = 0; i < lines_found; i++) {
        ret[i*2] = line_match[i*2];
        ret[i*2+1] = line_match[i*2+1]; 
        loc_t ext_lines[2];
        extend_lines_to_edge_of_image(im, line_match[i*2], line_match[i*2+1], ext_lines);
        add_line_to_buffer(im, buf, 2.0, ext_lines[0], ext_lines[1], vx_blue);
    }

    zarray_vmap(loc_arr, free);
    zarray_destroy(loc_arr);
    return(ret);

    // int corners_found = 0;
    // loc_t corners[4];
    // find_corners_from_lines(im,line_match, 4, &corners_found, corners);
    // for(int i = 0; i < corners_found; i++) {
    //     printf("(%d, %d)\n", corners[i].x, corners[i].y);
    //     add_circle_to_buffer(buf, 3.0, corners[i], vx_blue);
    // }

    // printf("(%d, %d) (%d, %d)   %lf\n", line_match[0].x, line_match[0].y, 
    //                                     line_match[1].x, line_match[1].y, 
    //                                     (double)best_score/N);

}


void add_circle_to_buffer(vx_buffer_t* buf, double size, loc_t loc, const float* color)
{
    vx_buffer_add_back(buf,
             vxo_pix_coords(VX_ORIGIN_BOTTOM_LEFT,
                    vxo_chain(vxo_mat_translate3(loc.x, loc.y, 0),
                        vxo_mat_scale(size),
                        vxo_circle(vxo_mesh_style(color)))));
}

void add_arr_of_locs_to_buffer(zarray_t* arr, vx_buffer_t* buf, double size, 
                                const float* color, metrics_t met)
{
    for(int i = 0; i < zarray_size(arr); i++)
    {
        loc_t* tmp;
        zarray_get(arr, i, &tmp);
        add_circle_to_buffer(buf, size, *tmp, color);
    }
}

void add_sides_to_buffer(image_u32_t* im, vx_buffer_t* buf, double size, 
                            node_t* n, const float* color, metrics_t met)
{
    // go over each row and add circles where left/right most are
    loc_t ll;
    int ll_large = im->width;

    for(int i = 0; i < im->height; i++) {
        add_circle_to_buffer(buf, size, n->sides[i].leftmost, color);
        // if(n->sides[i].leftmost.x  < ll_large) {
        //     ll = n->sides[i].leftmost;
        //     ll_large = n->sides[i].leftmost.x;
        // }

        // add_circle_to_buffer(buf, size, n->sides[i].rightmost, color);
    }

    // add_circle_to_buffer(buf, 4.0, ll, vx_blue);



    // // test slopes
    // int found = 0;
    // double prev_slope = get_slope(n->sides[0].leftmost, n->sides[1].leftmost, 0, 1, NULL);
    // int dist = 5;
    // for(int i = 0; i < (im->height-1); i+=dist) {
    //     if(n->sides[i].leftmost.x == im->width || n->sides[i+dist].leftmost.x == im->width) {  // if comparing side is not really a side, do not
    //         if(found) 
    //         {
    //             return;
    //         }
    //         continue;
    //     }
    //     found = 1; 
    //     double curr_slope = get_slope(n->sides[i].leftmost, n->sides[i+dist].leftmost, 0, 1, NULL);
    //     printf("slope: (%d, %d)  (%d, %d)  %lf)\n", n->sides[i].leftmost.x, n->sides[i].leftmost.y,
    //     n->sides[i+dist].leftmost.x, n->sides[i+dist].leftmost.y,((prev_slope) - (curr_slope)));

    //     // if(fabs(fabs(prev_slope) - fabs(curr_slope)) > met.std_dev_from_square) {
    //     //     add_circle_to_buffer(buf, 3.0, n->sides[i].leftmost, vx_black);
    //     // }
    // }

    
    // // build slope and pixel error to slope moving up the image, when error begins to get worse, that is the turning point
    // int found = 0;
    // int start_point = 0;
    // for(int i = 0; i < (im->height-1); i++) {
    //     while(n->sides[i].leftmost.x == im->width) continue;        //beginning of blob

    //     // find slope of startpoint and i
    //     // m = (n->sides[start_point].leftmost.y - n->sides[i].leftmost.y) / 
    //     //     (n->sides[start_point].leftmost.x - n->sides[i].leftmost.x);

    //     //compare prev line fit error to curr
    //     double error_sum = 0.0;
    //     loc_t start_loc = {n->sides[start_point].leftmost.x, n->sides[start_point].leftmost.y};
    //     loc_t end_loc = {n->sides[i].leftmost.x, n->sides[i].leftmost.y};

    //     for(int j = start_point; j < (im->height-1); j++) {    //find error to new line
    //         loc_t check_loc = {n->sides[j].leftmost.x, n->sides[j].leftmost.y};

    //         error_sum += dist_from_point_to_line(start_loc, end_loc, check_loc);
    //     }


    //     if(found &&  n->sides[i].leftmost.x == im->width) break;    // end of blob
    // }

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
    abgr_t hold = abgr; 
    abgr.r /= num;
    abgr.g /= num;
    abgr.b /= num;
    if(abgr.r > 0xFF || abgr.g > 0xFF || abgr.b > 0xFF) {
        printf("error: (%d, %d, %d, %d)\n", hold.a, hold.b, hold.g, hold.r); 
        assert(0);
    }
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

row_sides_t* init_row_sides(image_u32_t* im)
{
    row_sides_t* ret = calloc(im->height, sizeof(row_sides_t));

    for(int i = 0; i < im->height; i++){
        ret[i].leftmost.x = im->width;
        ret[i].rightmost.x = -1;
        ret[i].rightmost.y = ret[i].leftmost.y = i;
        ret[i].rightmost.valid = ret[i].leftmost.valid = 0;
    }
    return(ret);
}

void check_row_side(image_u32_t* im, node_t* n) 
{
    node_t* p = n->parent_node;
    loc_t n_loc = {n->id % im->stride, n->id / im->stride};
    // allocate space for array if not already allocated
    if(p->sides == NULL) {
        p->sides = init_row_sides(im);
    }

    if(n_loc.x < p->sides[n_loc.y].leftmost.x) p->sides[n_loc.y].leftmost = n_loc;
    if(n_loc.x > p->sides[n_loc.y].rightmost.x) p->sides[n_loc.y].rightmost = n_loc; 
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
                    n->sides = NULL;

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

            check_row_side(im, node);
            // key should exist, if it doesn't find out why
            assert(zhash_remove(node_map, &node->id, NULL, NULL));
            free(node);
        }
    }
    zarray_destroy(vals);

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
        else free(node);
    }
    zarray_destroy(vals);
    zhash_vmap_values(node_map, free);
    zhash_destroy(node_map);
}

// returns 1 if thinks this is a target blob 
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
    // find length of sides
    double side_lengths[4] = {  sqrt((corners[0].x - corners[1].x)*(corners[0].x - corners[1].x) + (corners[0].y - corners[1].y)*(corners[0].y - corners[1].y)),
                                sqrt((corners[1].x - corners[2].x)*(corners[1].x - corners[2].x) + (corners[1].y - corners[2].y)*(corners[1].y - corners[2].y)),
                                sqrt((corners[2].x - corners[3].x)*(corners[2].x - corners[3].x) + (corners[2].y - corners[3].y)*(corners[2].y - corners[3].y)),
                                sqrt((corners[3].x - corners[0].x)*(corners[3].x - corners[0].x) + (corners[3].y - corners[0].y)*(corners[3].y - corners[0].y))};
    // find mean of lengths
    double mean = (side_lengths[0] + side_lengths[1] + side_lengths[2] + side_lengths[3]) / 4;
    // add differences from mean
    double diff_sum = 0;
    for(int i = 0; i < 4; i++) {
        diff_sum += (side_lengths[i] - mean)*(side_lengths[i] - mean);
    }
    // calc variance
    double std_dev = sqrt(diff_sum/4);
    // printf("std_dev %lf    comp %lf\n", std_dev, met.std_dev_from_square);
    if(std_dev > met.std_dev_from_square) {
        for(int i = 0; i < 4; i++) {
            printf("%lf   ", side_lengths[i]);
        }
        printf("   std_dev:  %lf   %lf\n", std_dev, met.std_dev_from_square);
        return(0);
    }


    // int start_distance_from_corner = 2;
    // int line_length = 5;
    // //check that corners of node are on apropriate background
    // // method 1: take average of 5 pixels from two opposite sides, and compare to background colors
    // for(int j = 0; j < 2; j++) {        // diaganol lines
    //     abgr_t ave_color_of_line = {0xFF,0,0,0}; 
    //     int slope = get_slope(corners[j*2], corners[j*2+1], 1, buf); 

    //     for(int k = start_distance_from_corner; k < (start_distance_from_corner+line_length); k++) {    // pixels in line 
    //         loc_t loc = {corners[j*2].x + k, corners[j*2].y + k*slope};

    //         if(in_range(im, loc.x, loc.y)) {
    //             ave_color_of_line.r += (im->buf[loc.y*im->stride + loc.x] & 0xFF);
    //             ave_color_of_line.g += (im->buf[loc.y*im->stride + loc.x] >> 8 & 0xFF);
    //             ave_color_of_line.b += (im->buf[loc.y*im->stride + loc.x] >> 16 & 0xFF);
    //         }
    //     }

    //     float color_out[4];
    //     int matches = matches_background(abgr_ave_and_int(ave_color_of_line, 5), met, color_out);
    //     for(int k = 2; k < 7; k++) {    // if is a match add the color it matches to buf 
    //         loc_t loc = {corners[j*2].x + k, corners[j*2].y + k*slope};
    //         add_circle_to_buffer(buf, 1.5, loc, color_out);
    //     }

    //     if(!matches) return(0); 
    // }
    return(1);
}

// returns the 35 points associated to the test chart in [x1,y1,x2,y2] 
// format if there are more than 35 points will return NULL
matd_t* build_homography(image_u32_t* im, vx_buffer_t* buf, metrics_t met)
{
    frame_t frame = {{0,0}, {im->width-1, im->height-1},
                        {0,0}, {1,1}};
    int good_size = 0;
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
        loc_t center = {    .x = n.ave_loc.x/n.num_children,
                            .y = n.ave_loc.y/n.num_children};
        loc_t parent = {    .x = n.id % im->stride,
                            .y = n.id / im->stride};


        if(buf != NULL) {
            add_circle_to_buffer(buf, size, center, vx_maroon);
            // add_circle_to_buffer(buf, size, parent, vx_olive);

            // add_sides_to_buffer(im, buf, 1.0, &n, vx_orange, met);
            loc_t* lp = fit_lines(im, &n, buf, met, NULL);
            if(lp != NULL) {
                // printf("(%d, %d) (%d, %d) (%d, %d) (%d, %d) \n",
                //         lp[0].x, lp[0].y, lp[1].x, lp[1].y, lp[2].x, lp[2].y, lp[3].x, lp[3].y);
                loc_t intersect = get_line_intersection(lp[0], lp[1], lp[2], lp[3]);
                if(in_range(im, intersect.x, intersect.y)) {
                    loc_t ext_lines[2];
                    extend_lines_to_edge_of_image(im, intersect, center, ext_lines);
                    add_line_to_buffer(im, buf, 2.0, ext_lines[0], ext_lines[1], vx_blue);                
                }
                for(int i = 0; i < 4; i++) {
                    pix_array[i*2] = lp[i].x;
                    pix_array[i*2+1] = lp[i].y;
                    add_circle_to_buffer(buf, 3.0, lp[i], vx_orange);
                }
            }



            free(n.sides);

            // loc_t corners[4] = {{n.box.right, n.box.top},
            //                     {n.box.right, n.box.bottom},
            //                     {n.box.left, n.box.bottom},
            //                     {n.box.left, n.box.top}};
            // print extremes of box
            // if(1) {
            //     add_circle_to_buffer(buf, size, corners[0], vx_green);
            //     add_circle_to_buffer(buf, size, corners[1], vx_yellow);
            //     add_circle_to_buffer(buf, size, corners[2], vx_red);
            //     add_circle_to_buffer(buf, size, corners[3], vx_blue);
            //     for(int j = 0; j < 4; j++) {
            //         // add_circle_to_buffer(buf, size, corners[j], vx_maroon);
            //     }
            // }
        }
    }

    matd_t* H;
    H = dist_homography(pix_array, NUM_TARGETS);

    // if(0) {//zarray_size(blobs) == NUM_CHART_BLOBS){
    //     H = dist_homography(pix_array, NUM_CHART_BLOBS);
    // }
    // else if(zarray_size(blobs) == NUM_TARGETS){
    //     H = dist_homography(pix_array, NUM_TARGETS);
    //     if(met.add_lines) connect_lines(blobs, buf);
    // }
    // else {
    //     if(met.dothis)
    //         printf("num figures: %d\n", zarray_size(blobs));
    //     return(NULL);
    // }

    // make projected points
    // project_measurements_through_homography(H, buf, blobs, zarray_size(blobs));
    zarray_destroy(blobs);

    return(H);
}


/*
{ R00, R01, R02, TX,
   R10, R11, R12, TY,
   R20, R21, R22, TZ,
    0, 0, 0, 1 });
*/
double get_rotation(const char* axis, matd_t* H)
{
    double cosine, sine, theta;

    if(strncmp(axis,"x", 1)) {
        cosine = MATD_EL(H, 1, 1);
        sine = MATD_EL(H, 2, 1);
    }
    else if(strncmp(axis,"y", 1)) {
        cosine = MATD_EL(H, 0, 0);
        sine = MATD_EL(H, 0, 2);
    }
    else if(strncmp(axis,"z", 1)) {
        cosine = MATD_EL(H, 0, 0);
        sine = MATD_EL(H, 1, 0);
    }
    else assert(0);

    theta = atan2(sine, cosine);
    return(theta);
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
    matd_print(Model, "%15f");
    printf("\n\n");
    // matd_print(matd_op("M^-1",Model), matrix_format);
    // printf("\n");
    // extrapolate metrics from model view
    double TX = MATD_EL(Model, 0, 3);
    double TY = MATD_EL(Model, 1, 3);
    double TZ = MATD_EL(Model, 2, 3);

    // double rot_x = get_rotation("x", H);
    // double rot_y = get_rotation("y", H);
    // double rot_z = get_rotation("z", H);

    double cosine = MATD_EL(Model, 0, 0);

    double rot_z = acos(cosine) * 180/1.5 - 180;


    cosine = MATD_EL(Model, 2, 2);
    double rot_x = asin(cosine) * 90/1.3 + 90;

    cosine = MATD_EL(Model, 1, 1);
    double rot_y = asin(cosine);



    char str[200];
    sprintf(str, "<<#00ffff,serif-30>> DIST:%lf  Offset:(%lf, %lf)\n rot: (%lf, %lf, %lf)\n", 
                TZ, TX, TY, rot_x, rot_y, rot_z);
    vx_object_t *text = vxo_text_create(VXO_TEXT_ANCHOR_BOTTOM_LEFT, str); 
    vx_buffer_add_back(buf, vxo_pix_coords(VX_ORIGIN_BOTTOM_LEFT, text));

    // printf("dist: %lf   cos:%lf  angle: %lf\n", TZ, cosine, theta);
}
