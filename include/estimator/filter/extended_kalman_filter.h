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
 * 
 * state: x is se3 <--- SE3: T_cm
 * observation: z is projected pixel point in image of the upper left marker corner point.
*/
class EKF_Camera_Pose
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<EKF_Camera_Pose> Ptr;

    typedef std::pair<Vec6, Mat66> state_distribution;

    // constructor & destructor ///////////////////////////////////////////////
    EKF_Camera_Pose() {};

    EKF_Camera_Pose(Camera::Ptr camera);

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
    state_distribution estimate(const state_distribution& state_distribution_post_prev, 
        const Vec6& u, const Vec2& z_meas);

private:
    // member data ////////////////////////////////////////////////////////////
    // motion
    Mat66 F_; // Jacobian (state)

    // observation
    // Mat66 H_; // Jacobian
    
    Camera::Ptr camera_ = nullptr;

    // member methods /////////////////////////////////////////////////////////
    /**
     * 
     */
    Mat26 compute_H(const Vec3& p3D_camera);

    /**
     *
     */
    Mat66 compute_Q();

    /**
     *
     */
    Mat22 compute_R();
};

} // namespace tello_slam

#endif // TELLOSLAM_ESTIMATOR_FILTER_EKF
