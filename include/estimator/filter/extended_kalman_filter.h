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
 * state: x is (t, q) for T_cm
 * observation: z is projected pixel point in image of the upper left marker corner point.
*/
class EKF_Camera_Pose
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<EKF_Camera_Pose> Ptr;

    typedef std::pair<Vec7, Mat77> state_distribution;

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
        const Vec7& u, const Vec2& z_meas);

    /**
     * estimate
     */
    state_distribution estimate(const state_distribution& state_distribution_post_prev, 
        const Vec7& delta_x, const Vec2& z_meas, Mat77& Q, Mat22& R);

private:
    // member data ////////////////////////////////////////////////////////////
    // motion
    Mat77 F_; // Jacobian (state)
    Mat77 default_Q_;

    // observation
    // Mat27 H_; // Jacobian
    Mat22 default_R_;
    
    Camera::Ptr camera_ = nullptr;

    // member methods /////////////////////////////////////////////////////////
    /**
     * 
     */
    Mat77 compute_F(const Vec7& dx);

    /**
     * 
     */
    Mat27 compute_H(const Vec3& p3D_camera);

    /**
     *
     */
    Mat77 compute_Q();

    /**
     *
     */
    Mat22 compute_R();

    /**
     * 
     */
    double compute_Q_factor();

    /**
     * 
     */
    double compute_R_factor();

    /**
     * 
     */
    Vec2 h_pinhole(const Vec7& x_prio);
};

} // namespace tello_slam

#endif // TELLOSLAM_ESTIMATOR_FILTER_EKF
