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
    H_;
};

// member methods /////////////////////////////////////////////////////////////
EKF_Camera_Pose::state_distribution EKF_Camera_Pose::estimate(
    const EKF_Camera_Pose::state_distribution& state_distribution_post_prev, 
    const Vec6& u, const Mat66& Q,
    const Vec6& z_meas, const Mat66& R)
{
    Vec6 x_post_prev = std::get<0>(state_distribution_post_prev);
    Mat66 P_post_prev = std::get<1>(state_distribution_post_prev);

    // prediction
    Vec6 x_prio = F_*x_post_prev + u;
    Mat66 P_prio = F_*P_post_prev*F_.transpose() + Q;

    // Kalman gain
    Mat66 S = H_*P_prio*H_.transpose() + R;
    Mat66 K = P_prio*H_.transpose()*S.inverse();

    // update
    Vec6 x_post = x_prio + K*(z_meas - H_*x_prio);
    Mat66 P_post = (Mat66::Identity() - K*H_)*P_prio;

    return std::make_pair(x_post, P_post);
}

// private XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
// member methods /////////////////////////////////////////////////////////////
void EKF_Camera_Pose::compute_and_set_Q()
{

};

// ----------------------------------------------------------------------------
void EKF_Camera_Pose::compute_and_set_R()
{

};

} // namespace tello_slam
