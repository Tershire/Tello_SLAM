// aruco_detector.h

/////////////////////////
// IONLAB ISAE-SUPAERO //
// TELLO SLAM PROJECT  //
/////////////////////////

// 2024 JAN 15
// Wonhee LEE

// reference:


#ifndef TELLOSLAM_MARKER_ARUCODETECTOR_H
#define TELLOSLAM_MARKER_ARUCODETECTOR_H

#include "common.h"
#include "camera/camera.h"


namespace tello_slam
{

/**
 * detect, and estimate pose of ArUco code
 * uses IPPE PnP method enforced with RANSAC
 * calculates adjacent poses' error, if not good, it does not update current pose
 * if the pose continues to be bad, it resets within a given buffer time,
 * so that pose can be updated without too much delay.
 */
class ArUco_Detector
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<ArUco_Detector> Ptr;

    // state member ///////////////////////////////////////////////////////////
    enum Input_Mode
    {
        USB,
        VIDEO,
        TELLO,
        RASPBERRY,
        REALSENSE
    };

    // member data ////////////////////////////////////////////////////////////
    int target_id_;

    bool verbose_;

    // interface ==============================================================
    // pthread_mutex_t* t_cm_lock_;
    // double* t_cm_out_[3];

    // constructor & destructor ///////////////////////////////////////////////
    ArUco_Detector();

    ArUco_Detector(const int& target_id,
        const std::string& predifined_dictionary_name,
        const double& marker_length, const Camera::Ptr camera);

    // getter & setter ////////////////////////////////////////////////////////
    // getter =================================================================
    SE3 get_T_cm() const {return T_cm_;}

    // setter =================================================================
    void set_target_id(const int& target_id) {target_id_ = target_id;}
    void set_verbose(const bool& verbose) {verbose_ = verbose;}

    // port -------------------------------------------------------------------
    void set_input_mode(const Input_Mode& input_mode) {input_mode_ = input_mode;}

    // member methods /////////////////////////////////////////////////////////
    bool run();

    int find_target_index(const std::vector<int>& ids) const;

    // interface ==============================================================
    // int update_state_output();

private:
    // member data ////////////////////////////////////////////////////////////
    // ArUco ==================================================================
    double marker_length_;
    
    cv::aruco::Dictionary dictionary_;
    cv::aruco::DetectorParameters detector_parameters_;

    cv::Ptr<cv::aruco::ArucoDetector> detector_;

    // camera =================================================================
    Camera::Ptr camera_;
    cv::Mat cameraMatrix_;
    cv::Mat distCoeffs_;

    // pose estimation ========================================================
    // PnP --------------------------------------------------------------------
    std::vector<cv::Point3d> p3Ds_target_;
    SE3 T_cm_; // current pose
    bool is_pose_ok_;

    // interface ==============================================================
    // double t_cm_[3] = {0, 0, 0};

    // filter =================================================================
    std::shared_ptr<SE3> previous_T_cm_ = nullptr;

    // port ===================================================================
    Input_Mode input_mode_;
    float resize_scale_factor_;

    float mono_camera_scale_factor_;

    // member methods /////////////////////////////////////////////////////////
    inline double compute_pose_error(SE3 T_cm)
    {
        return (T_cm.matrix() - (previous_T_cm_.get())->matrix()).norm();
    }
};

} // namespace tello_slam

#endif // TELLOSLAM_MARKER_ARUCODETECTOR_H
