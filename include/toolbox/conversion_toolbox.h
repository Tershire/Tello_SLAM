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
 * conversion from rvec and tvec to T
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

/**
 * conversion from T to rvec and tvec
 */
inline void T_to_rvec_and_tvec(const SE3& T, cv::Vec3d& rvec, cv::Vec3d& tvec)
{
    Mat33 R;
    Vec3 t;
    R = T.rotationMatrix();
    t = T.translation();

    cv::Matx33d rmat;
    rmat(0, 0) = R(0, 0);
    rmat(0, 1) = R(0, 1);
    rmat(0, 2) = R(0, 2);
    rmat(1, 0) = R(1, 0);
    rmat(1, 1) = R(1, 1);
    rmat(1, 2) = R(1, 2);
    rmat(2, 0) = R(2, 0);
    rmat(2, 1) = R(2, 1);
    rmat(2, 2) = R(2, 2);

    cv::Rodrigues(rmat, rvec);

    tvec(0) = t(0);
    tvec(1) = t(1);
    tvec(2) = t(2);
}

/**
 * conversion from T to r and t
 */
inline void T_to_r_and_t(const SE3& T, Vec3& r, Vec3& t)
{
    SO3 R;
    R = T.so3();
    r = R.log();

    t = T.translation();
}

/**
 * conversion from T to state so3_t
 */
inline Vec6 T_to_so3_t(const SE3& T)
{
    SO3 R;
    Vec3 r, t;
    R = T.so3();
    r = R.log();

    t = T.translation();

    Vec6 so3_t;
    so3_t << r, t;

    return so3_t;
}

/**
 * conversion from state so3_t to T 
 */
inline SE3 so3_t_to_T(const Vec6& so3_t)
{
    SE3 T;
    SO3 R;
    Vec3 r, t;
    r = so3_t.head(3);
    t = so3_t.tail(3);

    R.exp(r);

    T = SE3(R, t);

    return T;
}

/**
 * conversion from T to state t_q: (t_x, t_y, t_z, q_x, q_y, q_z, q_w)
 */
inline Vec7 T_to_t_q(const SE3& T)
{
    Vec3 t = T.translation();
    Eigen::Quaterniond q = T.unit_quaternion();

    Vec7 t_q;
    t_q << t, q.coeffs();

    return t_q;
}

/**
 * conversion from state so3_t to T
 */
inline SE3 t_q_to_T(const Vec7& t_q)
{
    Vec3 t = t_q.head(3);
    Vec4 q_coeffs = t_q.tail(4);
    Eigen::Quaterniond q(q_coeffs);

    SE3 T = SE3(q, t);

    return T;
}

} // namespace tello_slam

#endif // TELLOSLAM_CONVERSIONTOOLBOX_H
