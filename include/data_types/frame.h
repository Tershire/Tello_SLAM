// frame.h

/////////////////////////
// IONLAB ISAE-SUPAERO //
// TELLO SLAM PROJECT  //
/////////////////////////

// 2024 MAR 09
// Wonhee LEE

// reference: slambook


#pragma once

#ifndef TELLOSLAM_FRAME_H
#define TELLOSLAM_FRAME_H

#include "camera/camera.h"
#include "common.h"


namespace tello_slam
{

struct Landmark;
struct Feature;
struct ArUco_Feature;

/**
 * each frame has a unique frame ID
 * if a frame is selected as a keyframe, 
 * it is endowed with a unique keyframe ID, which is independent of the frame ID
 */
struct Frame
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Frame> Ptr;

    // member data ////////////////////////////////////////////////////////////
    unsigned long id_ = 0; // frame ID
    unsigned long keyframe_id_ = 0; // keyframe ID
    bool is_keyframe_ = false; // is it a keyframe

    double timestamp_; // timestamp
    SE3 T_cw_; // camera pose w.r.t the world
    cv::Mat image_; // image
    std::vector<std::shared_ptr<Feature>> features_;
    std::vector<std::shared_ptr<ArUco_Feature>> aruco_features_;

    std::mutex T_cw_mutex_; // pose data mutex

public:
    // constructor & destructor ///////////////////////////////////////////////
    Frame() {}

    Frame(long id, double timestamp, const SE3& T_cw, const cv::Mat& image);

    // getter & setter ////////////////////////////////////////////////////////
    // getter =================================================================
    SE3 get_T_cw()
    {
        std::unique_lock<std::mutex> lock(T_cw_mutex_);
        return T_cw_;
    }
    
    // setter =================================================================
    void set_timestamp(const double& timestamp)
    {
        timestamp_ = timestamp;
    }

    void set_T_cw(const SE3& T_cw)
    {
        std::unique_lock<std::mutex> lock(T_cw_mutex_);
        T_cw_ = T_cw;
    }

    // member methods /////////////////////////////////////////////////////////
    /**
     * set a frame as keyframe and assign a keyframe id 
     */
    void set_as_keyframe();

    // static methods /////////////////////////////////////////////////////////
    // factory function =======================================================
    static std::shared_ptr<Frame> create_frame();
};

} // namespace tello_slam

#endif // TELLOSLAM_FRAME_H
