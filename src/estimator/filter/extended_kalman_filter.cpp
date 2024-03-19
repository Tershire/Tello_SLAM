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
EKF_Camera_Pose::EKF_Camera_Pose()
{

}

// member methods /////////////////////////////////////////////////////////////
EKF_Camera_Pose::state_distribution EKF_Camera_Pose::estimate(
    const EKF_Camera_Pose::state_distribution& state_distribution_post_prev, const Vec6& z_meas)
{
    Vec6 x_post_prev = std::get<0>(state_distribution_post_prev);
    Mat66 P_post_prev = std::get<1>(state_distribution_post_prev);

    // prediction
    Vec6 x_prio = F_*x_post_prev + u_;
    Mat66 P_prio = F_*P_post_prev*F_.transpose() + Q_;

    // Kalman gain
    Mat66 S = H_*P_prio*H_.transpose() + R_;
    Mat66 K = P_prio*H_.transpose()*S.inverse();

    // update
    Vec6 x_post = x_prio + K*(z_meas - H_*x_prio);
    Mat66 P_post = (Mat66::Identity() - K*H_)*P_prio;

    return std::make_pair(x_post, P_post);
}

// private XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
// member methods /////////////////////////////////////////////////////////////


} // namespace tello_slam
