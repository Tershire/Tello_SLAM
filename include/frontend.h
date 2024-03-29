// frontend.h

/////////////////////////
// IONLAB ISAE-SUPAERO //
// TELLO SLAM PROJECT  //
/////////////////////////

// 2024 MAR 10
// Wonhee LEE

// reference: slambook


#pragma once

#ifndef TELLOSLAM_FRONTEND_H
#define TELLOSLAM_FRONTEND_H

#include <opencv2/features2d.hpp>

#include "common.h"
#include "frame.h"
#include "map.h"
#include "aruco_detector.h"
#include "extended_kalman_filter.h"


namespace tello_slam
{

class Backend;
class Viewer;

/**
 * frontend
 */
class Frontend
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Frontend> Ptr;

    // state member ///////////////////////////////////////////////////////////
    enum Tracking_Status
    {
        NOT_INITIALIZED,
        OK,
        BAD,
        LOST
    };

    // member data ////////////////////////////////////////////////////////////
    bool use_EKF_;

    // constructor & destructor ///////////////////////////////////////////////
    Frontend();

    // getter & setter ////////////////////////////////////////////////////////
    // getter =================================================================
    Tracking_Status get_tracking_status() const {return tracking_status_;}
    Vec3 get_current_camera_position() const {return current_camera_position_;}
    Vec3 get_current_camera_translational_velocity() const {return current_camera_translational_velocity_;}

    // setter =================================================================
    void set_camera(Camera::Ptr camera) {camera_ = camera;}
    void set_map(Map::Ptr map) {map_ = map;}
    void set_backend(std::shared_ptr<Backend> backend) {backend_ = backend;}
    void set_viewer(std::shared_ptr<Viewer> viewer) {viewer_ = viewer;}
    void set_aruco_detector(std::shared_ptr<ArUco_Detector> aruco_detector) {aruco_detector_ = aruco_detector;}
    void set_ekf_camera_pose(std::shared_ptr<EKF_Camera_Pose> ekf_camera_pose) {ekf_camera_pose_ =ekf_camera_pose;}
    
    // member methods /////////////////////////////////////////////////////////
    /**
     * 
     */
    bool step(Frame::Ptr frame);

private:
    // member data ////////////////////////////////////////////////////////////
    Tracking_Status tracking_status_ = Tracking_Status::NOT_INITIALIZED;

    Frame::Ptr current_frame_ = nullptr;
    Frame::Ptr previous_frame_ = nullptr;
    
    Camera::Ptr camera_ = nullptr;

    Map::Ptr map_ = nullptr;
    std::shared_ptr<Backend> backend_ = nullptr;
    std::shared_ptr<Viewer> viewer_ = nullptr;
    std::shared_ptr<EKF_Camera_Pose> ekf_camera_pose_ = nullptr;

    SE3 T_CurrPrev_; // The relative motion between the current frame and the previous frame is used to estimate the initial value of the pose of the current frame

    // system configuration parameters ========================================

    // utilities ==============================================================
    ArUco_Detector::Ptr aruco_detector_ = nullptr;

    // motion log =============================================================
    bool motion_log_on_;
    Quaternion current_camera_angular_velocity_unit_quaternion_;
    Vec3 current_camera_translational_velocity_;
    Vec3 current_camera_position_;

    // member methods /////////////////////////////////////////////////////////
    /**
     * try to initialize the frontend
     * @return true if success
     */
    bool initialize();

    /**
     * build the initial map
     * @return true if succeed
     */
    bool build_initial_map();

    /**
     * reset when lost
     * @return true if success
     */
    bool reset();

    /**
     * build the reset map
     */
    bool build_reset_map();

    /**
     * track in normal mode
     * @return true if success
     */
    bool track();

    /**
     * set current frame as a keyframe and insert it into backend
     * @return true if success
     */
    bool insert_keyframe();

    /**
     * 
     */
    void add_observation();

    /**
     * 
     */
    int detect_aruco_features();

    /**
     * 
     */
    std::vector<unsigned int> track_aruco_features();

    /**
     * 
     */
    int compute_aruco_poses();

    /**
     * 
     */
    int estimate_camera_pose();

    /**
     * 
     */
    std::pair<Mat77, Mat22> propose_Q_and_R(const double& v);

    /**
     * 
     */
    inline bool determine_pose_stability(const SE3& T1, const SE3& T2)
    {
        // rotation error 
        double rotation_error = (T2.so3().unit_quaternion().vec() - T1.so3().unit_quaternion().vec()).norm();

        // translation error
        double translation_error = (T2.translation() - T1.translation()).norm() / aruco_detector_->get_marker_length();

        //
        // std::cout << "rotation_error: " << rotation_error << ", " << "translation_error: " << translation_error << std::endl;

        return (rotation_error < 0.3) & (translation_error < 10);
    }
};

} // namespace tello_slam

#endif // TELLOSLAM_FRONTEND_H
