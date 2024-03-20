// extended_kalman_filter.cpp

/////////////////////////
// IONLAB ISAE-SUPAERO //
// TELLO SLAM PROJECT  //
/////////////////////////

// 2024 MAR 18
// Wonhee LEE

// reference: slambook


#include "extended_kalman_filter.h"


namespace tello_slam
{

// public XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
// constructor & destructor ///////////////////////////////////////////////////
EKF_Camera_Pose::EKF_Camera_Pose(Camera::Ptr camera)
    : camera_(camera)
{
    // motion
    F_ = Mat66::Identity();

    // observation
    // H_;
}

// member methods /////////////////////////////////////////////////////////////
EKF_Camera_Pose::state_distribution EKF_Camera_Pose::estimate(
    const EKF_Camera_Pose::state_distribution& state_distribution_post_prev, 
    const Vec6& u, const Mat66& Q,
    const Vec2& z_meas, const Mat22& R)
{
    Vec6 x_post_prev = std::get<0>(state_distribution_post_prev);
    Mat66 P_post_prev = std::get<1>(state_distribution_post_prev);

    Vec3 p3D_camera(x_post_prev[0], x_post_prev[1], x_post_prev[2]);
    Mat26 H = compute_H(p3D_camera);

    // prediction
    Vec6 x_prio = F_*x_post_prev + u;
    Mat66 P_prio = F_*P_post_prev*F_.transpose() + Q;

    // Kalman gain
    Mat22 S = H*P_prio*H.transpose() + R;
    Mat62 K = P_prio*H.transpose()*S.inverse();

    // update
    Vec6 x_post = x_prio + K*(z_meas - H*x_prio);
    Mat66 P_post = (Mat66::Identity() - K*H)*P_prio;

    return std::make_pair(x_post, P_post);
}

// private XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
// member methods /////////////////////////////////////////////////////////////
Mat26 EKF_Camera_Pose::compute_H(const Vec3& p3D_camera)
{
    Mat26 H;

    double X = p3D_camera[0];
    double Y = p3D_camera[1];
    double Z = p3D_camera[2];

    double Z2 = Z*Z;

    double fx = camera_->fx_;
    double fy = camera_->fy_;

    switch (camera_->camera_model_)
    {
        case Camera::PINHOLE:
            H << fx/Z, 0, -(X*fx)/Z2, 0, 0, 0,
                 0, fy/Z, -(Y*fy)/Z2, 0, 0, 0;
            break;

        case Camera::BROWN_CONRADY: // (TODO) implement B-C, for now it is pinhole.
            H << fx/Z, 0, -(X*fx)/Z2, 0, 0, 0,
                 0, fy/Z, -(Y*fy)/Z2, 0, 0, 0;
            break;
    }

    return H;
}

// ----------------------------------------------------------------------------
void EKF_Camera_Pose::compute_and_set_Q()
{

}

// ----------------------------------------------------------------------------
void EKF_Camera_Pose::compute_and_set_R()
{

}

} // namespace tello_slam

