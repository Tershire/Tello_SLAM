// conversion_toolbox.h

/////////////////////////
// IONLAB ISAE-SUPAERO //
// TELLO SLAM PROJECT  //
/////////////////////////

// 2024 JAN 15
// Wonhee LEE

// reference:


#ifndef TELLOSLAM_CONVERSIONTOOLBOX_H
#define TELLOSLAM_CONVERSIONTOOLBOX_H

#include <exception>

#include "common.h"


namespace tello_slam
{

/**
 * conversion from r_and_t to T
 */
inline SE3 rvec_and_tvec_to_T(const cv::Vec3d& rvec, const cv::Vec3d& tvec)
{
    cv::Matx33d rmat;
    cv::Rodrigues(rvec, rmat);

    Mat33 R;
    R << rmat(0, 0), rmat(0, 1), rmat(0, 2),
         rmat(1, 0), rmat(1, 1), rmat(1, 2),
         rmat(2, 0), rmat(2, 1), rmat(2, 2); 

    Vec3 t;
    t = Vec3(tvec[0], tvec[1], tvec[2]);

    Quaternion q(R);
    return SE3(q, t);
}

} // namespace tello_slam

#endif // TELLOSLAM_CONVERSIONTOOLBOX_H
