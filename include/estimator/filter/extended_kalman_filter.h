// extended_kalman_filter.h

/////////////////////////
// IONLAB ISAE-SUPAERO //
// TELLO SLAM PROJECT  //
/////////////////////////

// 2024 MAR 18
// Wonhee LEE

// reference: slambook


#ifndef TELLOSLAM_ESTIMATOR_FILTER_EKF
#define TELLOSLAM_ESTIMATOR_FILTER_EKF

#include "common.h"
#include "camera.h"


namespace tello_slam
{

/**
 * EKF for camera pose: T_cw
 * x <- F*x + u + w
 * y <- H*x + v
*/
class EKF_Camera_Pose
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<EKF_Camera_Pose> Ptr;

    typedef std::pair<Vec6, Mat66> state_distribution;

    // constructor & destructor ///////////////////////////////////////////////
    EKF_Camera_Pose();

    // getter & setter ////////////////////////////////////////////////////////
    // setter =================================================================
    /**
     * set camera 
     */
    void set_camera(Camera::Ptr camera)
    {
        camera_ = camera;
    }

    // member methods /////////////////////////////////////////////////////////
    /**
     * estimate
     */
    state_distribution estimate(const state_distribution& state_distribution, const Vec6& z_meas);

private:
    // member data ////////////////////////////////////////////////////////////
    // state
    Vec6 x_; // camera pose T_cw as so3
    Vec6 u_; // input
    Mat66 P_; // covariance

    // motion
    Mat66 F_; // Jacobian (state)
    Mat66 Q_; // covariance

    // observation
    Mat66 H_; // Jacobian
    Mat66 R_; // covariance
    
    Camera::Ptr camera_ = nullptr;
};

} // namespace tello_slam

#endif // TELLOSLAM_ESTIMATOR_FILTER_EKF
