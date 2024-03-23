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
    // F_;
    default_Q_ = Mat77::Identity();

    // observation
    // H_;
    default_R_ = Mat22::Identity();
}

// member methods /////////////////////////////////////////////////////////////
EKF_Camera_Pose::state_distribution EKF_Camera_Pose::estimate(
    const EKF_Camera_Pose::state_distribution& state_distribution_post_prev, 
    const Vec7& u, const Vec2& z_meas)
{
    Vec7 x_post_prev = std::get<0>(state_distribution_post_prev);
    Mat77 P_post_prev = std::get<1>(state_distribution_post_prev);

    Vec3 p3D_camera(x_post_prev[0], x_post_prev[1], x_post_prev[2]);
    Mat27 H = compute_H(p3D_camera);

    Mat77 Q = compute_Q();
    Mat22 R = compute_R();

    // prediction
    Vec7 x_prio = F_*x_post_prev + u;
    Mat77 P_prio = F_*P_post_prev*F_.transpose() + Q;

    // Kalman gain
    Mat22 S = H*P_prio*H.transpose() + R;
    Mat72 K = P_prio*H.transpose()*S.inverse();

    // update
    Vec7 x_post = x_prio + K*(z_meas - H*x_prio);
    Mat77 P_post = (Mat77::Identity() - K*H)*P_prio;

    return std::make_pair(x_post, P_post);
}

// ----------------------------------------------------------------------------
EKF_Camera_Pose::state_distribution EKF_Camera_Pose::estimate(
    const EKF_Camera_Pose::state_distribution& state_distribution_post_prev, 
    const Vec7& delta_x, const Vec2& z_meas, Mat77& Q, Mat22& R)
{
    Vec7 x_post_prev = std::get<0>(state_distribution_post_prev);
    Mat77 P_post_prev = std::get<1>(state_distribution_post_prev);

    Mat77 F = compute_F(delta_x);

    Vec7 u;
    u << delta_x.head(3), Vec4::Zero();

    Vec3 p3D_camera(x_post_prev[0], x_post_prev[1], x_post_prev[2]);
    Mat27 H = compute_H(p3D_camera);

    Q *= compute_Q_factor();
    R *= compute_R_factor();

    // prediction
    Vec7 x_prio = F*x_post_prev + u;
    Mat77 P_prio = F*P_post_prev*F.transpose() + Q;

    // Kalman gain
    Mat22 S = H*P_prio*H.transpose() + R;
    Mat72 K = P_prio*H.transpose()*S.inverse();

    // update
    Vec7 x_post = x_prio + K*(z_meas - H*x_prio);
    Mat77 P_post = (Mat77::Identity() - K*H)*P_prio;

    return std::make_pair(x_post, P_post);
}

// private XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
// member methods /////////////////////////////////////////////////////////////
Mat77 EKF_Camera_Pose::compute_F(const Vec7& delta_x)
{
    // Vec3 dt = delta_x.head(3);
    Vec4 delta_q = delta_x.tail(4);

    Mat44 Delta_q = delta_q.asDiagonal();

    Mat77 F;

    F << Mat33::Identity(), Mat34::Zero(),
        Mat43::Zero(), Delta_q;

    return F;
}

// ----------------------------------------------------------------------------
Mat27 EKF_Camera_Pose::compute_H(const Vec3& p3D_camera)
{
    Mat27 H;

    double X = p3D_camera[0];
    double Y = p3D_camera[1];
    double Z = p3D_camera[2];

    double Z2 = Z*Z;

    double fx = camera_->fx_;
    double fy = camera_->fy_;

    switch (camera_->camera_model_)
    {
        case Camera::PINHOLE:
            H << fx/Z, 0, -(X*fx)/Z2, 0, 0, 0, 0,
                 0, fy/Z, -(Y*fy)/Z2, 0, 0, 0, 0;
            break;

        case Camera::BROWN_CONRADY: // (TODO) implement B-C, for now it is pinhole.
            H << fx/Z, 0, -(X*fx)/Z2, 0, 0, 0, 0,
                 0, fy/Z, -(Y*fy)/Z2, 0, 0, 0, 0;
            break;
    }

    return H;
}

// ----------------------------------------------------------------------------
Mat77 EKF_Camera_Pose::compute_Q()
{
    Mat77 Q = Mat77::Identity()*1E0;

    return Q;
}

// ----------------------------------------------------------------------------
Mat22 EKF_Camera_Pose::compute_R()
{
    Mat22 R = Mat22::Identity()*1E0;

    return R;
}

// ----------------------------------------------------------------------------
double EKF_Camera_Pose::compute_Q_factor()
{
    return 1;
}

// ----------------------------------------------------------------------------
double EKF_Camera_Pose::compute_R_factor()
{
    return 1;
}

} // namespace tello_slam

