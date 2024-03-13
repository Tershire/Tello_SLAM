// aruco_feature.h

/////////////////////////
// IONLAB ISAE-SUPAERO //
// TELLO SLAM PROJECT  //
/////////////////////////

// 2024 MAR 09
// Wonhee LEE

// reference:


#pragma once

#ifndef TELLOSLAM_ARUCOFEATURE_H
#define TELLOSLAM_ARUCOFEATURE_H

#include <memory>
#include <opencv2/features2d.hpp>

#include "common.h"


namespace tello_slam
{

struct Frame;
struct ArUco_Landmark;

/**
 * keypoints and associated landmarks
 */
struct ArUco_Feature
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<ArUco_Feature> Ptr;

    // member data ////////////////////////////////////////////////////////////
    std::weak_ptr<Frame> frame_; // frame that holds the feature

    unsigned long aruco_id_; // ArUco marker ID
    std::vector<cv::Point2f> corner_keypoints_;
    SE3 T_cm_; // marker pose w.r.t camera
    
    std::weak_ptr<ArUco_Landmark> aruco_landmark_; // associated landmark

    bool is_outlier_ = false; // is it an outlier

public:
    // constructor & destructor ///////////////////////////////////////////////
    ArUco_Feature() {}

    ArUco_Feature(std::shared_ptr<Frame> frame, const long& aruco_id, 
        const std::vector<cv::Point2f>& corner_keypoints, const SE3& T_cm);

    // getter & setter ////////////////////////////////////////////////////////
    // getter =================================================================
    SE3 get_T_cm()
    {
        return T_cm_;
    }
};

} // namespace tello_slam

#endif // TELLOSLAM_ARUCOFEATURE_H
