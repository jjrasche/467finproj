/*$LICENSE*/

#include <math.h>

#include "matd.h"
#include "zarray.h"
#include "homography.h"

static inline float sq(float v)
{
    return v*v;
}

// correspondences is a list of float[4]s, consisting of the points x
// and y concatenated. We will compute a homography such that y = Hx
matd_t *homography_compute(zarray_t *correspondences)
{
    // compute centroids of both sets of points (yields a better
    // conditioned information matrix)
    double x_cx = 0, x_cy = 0;
    double y_cx = 0, y_cy = 0;

    for (int i = 0; i < zarray_size(correspondences); i++) {
        float *c;
        zarray_get_volatile(correspondences, i, &c);

        x_cx += c[0];
        x_cy += c[1];
        y_cx += c[2];
        y_cy += c[3];
    }

    int sz = zarray_size(correspondences);
    x_cx /= sz;
    x_cy /= sz;
    y_cx /= sz;
    y_cy /= sz;

    // NB We don't normalize scale; it seems implausible that it could
    // possibly make any difference given the dynamic range of IEEE
    // doubles.

    matd_t *A = matd_create(9,9);
    for (int i = 0; i < zarray_size(correspondences); i++) {
        float *c;
        zarray_get_volatile(correspondences, i, &c);

        // (below world is "x", and image is "y")
        double worldx = c[0] - x_cx;
        double worldy = c[1] - x_cy;
        double imagex = c[2] - y_cx;
        double imagey = c[3] - y_cy;

        double a03 = -worldx;
        double a04 = -worldy;
        double a05 = -1;
        double a06 = worldx*imagey;
        double a07 = worldy*imagey;
        double a08 = imagey;

        MATD_EL(A, 3, 3) += a03*a03;
        MATD_EL(A, 3, 4) += a03*a04;
        MATD_EL(A, 3, 5) += a03*a05;
        MATD_EL(A, 3, 6) += a03*a06;
        MATD_EL(A, 3, 7) += a03*a07;
        MATD_EL(A, 3, 8) += a03*a08;
        MATD_EL(A, 4, 4) += a04*a04;
        MATD_EL(A, 4, 5) += a04*a05;
        MATD_EL(A, 4, 6) += a04*a06;
        MATD_EL(A, 4, 7) += a04*a07;
        MATD_EL(A, 4, 8) += a04*a08;
        MATD_EL(A, 5, 5) += a05*a05;
        MATD_EL(A, 5, 6) += a05*a06;
        MATD_EL(A, 5, 7) += a05*a07;
        MATD_EL(A, 5, 8) += a05*a08;
        MATD_EL(A, 6, 6) += a06*a06;
        MATD_EL(A, 6, 7) += a06*a07;
        MATD_EL(A, 6, 8) += a06*a08;
        MATD_EL(A, 7, 7) += a07*a07;
        MATD_EL(A, 7, 8) += a07*a08;
        MATD_EL(A, 8, 8) += a08*a08;

        double a10 = worldx;
        double a11 = worldy;
        double a12 = 1;
        double a16 = -worldx*imagex;
        double a17 = -worldy*imagex;
        double a18 = -imagex;

        MATD_EL(A, 0, 0) += a10*a10;
        MATD_EL(A, 0, 1) += a10*a11;
        MATD_EL(A, 0, 2) += a10*a12;
        MATD_EL(A, 0, 6) += a10*a16;
        MATD_EL(A, 0, 7) += a10*a17;
        MATD_EL(A, 0, 8) += a10*a18;
        MATD_EL(A, 1, 1) += a11*a11;
        MATD_EL(A, 1, 2) += a11*a12;
        MATD_EL(A, 1, 6) += a11*a16;
        MATD_EL(A, 1, 7) += a11*a17;
        MATD_EL(A, 1, 8) += a11*a18;
        MATD_EL(A, 2, 2) += a12*a12;
        MATD_EL(A, 2, 6) += a12*a16;
        MATD_EL(A, 2, 7) += a12*a17;
        MATD_EL(A, 2, 8) += a12*a18;
        MATD_EL(A, 6, 6) += a16*a16;
        MATD_EL(A, 6, 7) += a16*a17;
        MATD_EL(A, 6, 8) += a16*a18;
        MATD_EL(A, 7, 7) += a17*a17;
        MATD_EL(A, 7, 8) += a17*a18;
        MATD_EL(A, 8, 8) += a18*a18;

        double a20 = -worldx*imagey;
        double a21 = -worldy*imagey;
        double a22 = -imagey;
        double a23 = worldx*imagex;
        double a24 = worldy*imagex;
        double a25 = imagex;

        MATD_EL(A, 0, 0) += a20*a20;
        MATD_EL(A, 0, 1) += a20*a21;
        MATD_EL(A, 0, 2) += a20*a22;
        MATD_EL(A, 0, 3) += a20*a23;
        MATD_EL(A, 0, 4) += a20*a24;
        MATD_EL(A, 0, 5) += a20*a25;
        MATD_EL(A, 1, 1) += a21*a21;
        MATD_EL(A, 1, 2) += a21*a22;
        MATD_EL(A, 1, 3) += a21*a23;
        MATD_EL(A, 1, 4) += a21*a24;
        MATD_EL(A, 1, 5) += a21*a25;
        MATD_EL(A, 2, 2) += a22*a22;
        MATD_EL(A, 2, 3) += a22*a23;
        MATD_EL(A, 2, 4) += a22*a24;
        MATD_EL(A, 2, 5) += a22*a25;
        MATD_EL(A, 3, 3) += a23*a23;
        MATD_EL(A, 3, 4) += a23*a24;
        MATD_EL(A, 3, 5) += a23*a25;
        MATD_EL(A, 4, 4) += a24*a24;
        MATD_EL(A, 4, 5) += a24*a25;
        MATD_EL(A, 5, 5) += a25*a25;
    }

    // make symmetric
    for (int i = 0; i < 9; i++)
        for (int j = i+1; j < 9; j++)
            MATD_EL(A, j, i) = MATD_EL(A, i, j);

    matd_svd_t svd = matd_svd(A);
    
    matd_t *Ainv = matd_inverse(A);

    double scale = 0;
    for (int i = 0; i < 9; i++)
        scale += sq(MATD_EL(Ainv, i, 0));
    scale = sqrt(scale);

    if (1) {
        // compute singular vector using SVD. A bit slower, but more accurate.
        matd_svd_t svd = matd_svd(A);

        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                // MATD_EL(H, i, j) = MATD_EL(Ainv, 3*i+j, 0)/ scale;
                MATD_EL(H, i, j) = MATD_EL(svd.U, 3*i+j, 8);

        matd_destroy(svd.U);
        matd_destroy(svd.S);
        matd_destroy(svd.V);

    } else {
        // compute singular vector by (carefully) inverting the rank-deficient matrix.
        matd_t *Ainv = matd_inverse(A);
        double scale = 0;
        for (int i = 0; i < 9; i++)
            scale += sq(MATD_EL(Ainv, i, 0));
        scale = sqrt(scale);

        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                MATD_EL(H, i, j) = MATD_EL(Ainv, 3*i+j, 0)/ scale;

        matd_destroy(Ainv);
    }

    
    matd_t *Tx = matd_identity(3);
    MATD_EL(Tx,0,2) = -x_cx;
    MATD_EL(Tx,1,2) = -x_cy;

    matd_t *Ty = matd_identity(3);
    MATD_EL(Ty,0,2) = y_cx;
    MATD_EL(Ty,1,2) = y_cy;

    matd_t *H2 = matd_op("M*M*M", Ty, H, Tx);

    matd_destroy(A);
    matd_destroy(Tx);
    matd_destroy(Ty);
    matd_destroy(H);

    matd_destroy(svd.U);
    matd_destroy(svd.S);
    matd_destroy(svd.V);

    return H2;
}

void homography_project(const matd_t *H, double x, double y, double *ox, double *oy)
{
    double xx = MATD_EL(H, 0, 0)*x + MATD_EL(H, 0, 1)*y + MATD_EL(H, 0, 2);
    double yy = MATD_EL(H, 1, 0)*x + MATD_EL(H, 1, 1)*y + MATD_EL(H, 1, 2);
    double zz = MATD_EL(H, 2, 0)*x + MATD_EL(H, 2, 1)*y + MATD_EL(H, 2, 2);

    *ox = xx / zz;
    *oy = yy / zz;
}

// assuming that the projection matrix is:
// [ fx 0  cx 0 ]
// [  0 fy cy 0 ]
// [  0  0  1 0 ]
//
// And that the homography is equal to the projection matrix times the model matrix,
// recover the model matrix (which is returned). Note that the third column of the model
// matrix is missing in the expresison below, reflecting the fact that the homography assumes
// all points are at z=0 (i.e., planar) and that the element of z is thus omitted.
// (3x1 instead of 4x1).
//
// [ fx 0  cx 0 ] [ R00  R01  TX ]    [ H00 H01 H02 ]
// [  0 fy cy 0 ] [ R10  R11  TY ] =  [ H10 H11 H12 ]
// [  0  0  1 0 ] [ R20  R21  TZ ] =  [ H20 H21 H22 ]
//                [  0    0    1 ]
//
// fx*R00 + cx*R20 = H00   (note, H only known up to scale; some additional adjustments required; see code.)
// fx*R01 + cx*R21 = H01
// fx*TX  + cx*TZ  = H02
// fy*R10 + cy*R20 = H10
// fy*R11 + cy*R21 = H11
// fy*TY  + cy*TZ  = H12
// R20 = H20
// R21 = H21
// TZ  = H22

matd_t *homography_to_pose(const matd_t *H, double fx, double fy, double cx, double cy)
{
    // Note that every variable that we compute is proportional to the scale factor of H.
    double R20 = MATD_EL(H, 2, 0);
    double R21 = MATD_EL(H, 2, 1);
    double TZ  = MATD_EL(H, 2, 2);
    double R00 = (MATD_EL(H, 0, 0) - cx*R20) / fx;
    double R01 = (MATD_EL(H, 0, 1) - cx*R21) / fx;
    double TX  = (MATD_EL(H, 0, 2) - cx*TZ)  / fx;
    double R10 = (MATD_EL(H, 1, 0) - cy*R20) / fy;
    double R11 = (MATD_EL(H, 1, 1) - cy*R21) / fy;
    double TY  = (MATD_EL(H, 1, 2) - cy*TZ)  / fy;

    // compute the scale by requiring that the rotation columns are unit length
    // (Use geometric average of the two length vectors we have)
    double length1 = sqrtf(R00*R00 + R10*R10 + R20*R20);
    double length2 = sqrtf(R01*R01 + R11*R11 + R21*R21);
    double s = 1.0 / sqrtf(length1 * length2);

    // get sign of S by requiring the tag to be behind the camera.
    if (TZ > 0)
        s *= -1;

    R20 *= s;
    R21 *= s;
    TZ  *= s;
    R00 *= s;
    R01 *= s;
    TX  *= s;
    R10 *= s;
    R11 *= s;
    TY  *= s;

    // now recover [R02 R12 R22] by noting that it is the cross product of the other two columns.
    double R02 = R10*R21 - R20*R11;
    double R12 = R20*R01 - R00*R21;
    double R22 = R00*R11 - R10*R01;

    // Improve rotation matrix by applying polar decomposition.
    if (1) {
        // do polar decomposition. This makes the rotation matrix
        // "proper", but probably increases the reprojection error. An
        // iterative alignment step would be superior.

        matd_t *R = matd_create_data(3, 3, (double[]) { R00, R01, R02,
                                                       R10, R11, R12,
                                                       R20, R21, R22 });

        matd_svd_t svd = matd_svd(R);
        matd_destroy(R);

        R = matd_op("M*M'", svd.U, svd.V);

        matd_destroy(svd.U);
        matd_destroy(svd.S);
        matd_destroy(svd.V);

        R00 = MATD_EL(R, 0, 0);
        R01 = MATD_EL(R, 0, 1);
        R02 = MATD_EL(R, 0, 2);
        R10 = MATD_EL(R, 1, 0);
        R11 = MATD_EL(R, 1, 1);
        R12 = MATD_EL(R, 1, 2);
        R20 = MATD_EL(R, 2, 0);
        R21 = MATD_EL(R, 2, 1);
        R22 = MATD_EL(R, 2, 2);

        matd_destroy(R);
    }

    return matd_create_data(4, 4, (double[]) { R00, R01, R02, TX,
                                               R10, R11, R12, TY,
                                               R20, R21, R22, TZ,
                                                0, 0, 0, 1 });
}

// Similar to above
// Recover the model view matrix assuming that the projection matrix is:
//
// [ F  0  A  0 ]     (see glFrustrum)
// [ 0  G  B  0 ]
// [ 0  0  C  D ]
// [ 0  0 -1  0 ]

matd_t *homography_to_model_view(const matd_t *H, double F, double G, double A, double B, double C, double D)
{
    matd_t *M = matd_create(4,4);

    // Note that every variable that we compute is proportional to the scale factor of H.
    double R20 = -MATD_EL(H, 2, 0);
    double R21 = -MATD_EL(H, 2, 1);
    double TZ  = -MATD_EL(H, 2, 2);
    double R00 = (MATD_EL(H, 0, 0) - A*R20) / F;
    double R01 = (MATD_EL(H, 0, 1) - A*R21) / F;
    double TX  = (MATD_EL(H, 0, 2) - A*TZ)  / F;
    double R10 = (MATD_EL(H, 1, 0) - B*R20) / G;
    double R11 = (MATD_EL(H, 1, 1) - B*R21) / G;
    double TY  = (MATD_EL(H, 1, 2) - B*TZ)  / G;

    // compute the scale by requiring that the rotation columns are unit length
    // (Use geometric average of the two length vectors we have)
    double length1 = sqrtf(R00*R00 + R10*R10 + R20*R20);
    double length2 = sqrtf(R01*R01 + R11*R11 + R21*R21);
    double s = 1.0 / sqrtf(length1 * length2);

    // get sign of S by requiring the tag to be behind the camera.
    if (TZ > 0)
        s *= -1;

    R20 *= s;
    R21 *= s;
    TZ  *= s;
    R00 *= s;
    R01 *= s;
    TX  *= s;
    R10 *= s;
    R11 *= s;
    TY  *= s;

    // now recover [R02 R12 R22] by noting that it is the cross product of the other two columns.
    double R02 = R10*R21 - R20*R11;
    double R12 = R20*R01 - R00*R21;
    double R22 = R00*R11 - R10*R01;

    // TODO XXX: Improve rotation matrix by applying polar decomposition.

    return matd_create_data(4, 4, (double[]) { R00, R01, R02, TX,
        R10, R11, R12, TY,
        R20, R21, R22, TZ,
        0, 0, 0, 1 });
    return M;
}

