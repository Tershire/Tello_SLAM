// math_toolbox.h

/////////////////////////
// IONLAB ISAE-SUPAERO //
// TELLO SLAM PROJECT  //
/////////////////////////

// 2024 MAR 23
// Wonhee LEE

// reference:


#ifndef TELLOSLAM_MATHTOOLBOX_H
#define TELLOSLAM_MATHTOOLBOX_H

#include <exception>

#include "common.h"


namespace tello_slam
{

/**
 * matrix representation of left quaternion multiplication
 * q_2 = q * q_1 ---> q_2 = M * q_1
 * 
 * q = (q_x, q_y, q_z, q_w) following Eigen convention
 */
inline Mat44 left_quaternion_multiplication_matrix(Vec4 q)
{
    Mat44 M;

    double x, y, z, w;
    x = q[0];
    y = q[1];
    z = q[2];
    w = q[3];

    M <<  w, -z,  y, x,
          z,  w, -x, y, 
         -y,  x,  w, z,
         -x, -y, -z, w; 

    return M;
}

} // namespace tello_slam

#endif // TELLOSLAM_MATHTOOLBOX_H
