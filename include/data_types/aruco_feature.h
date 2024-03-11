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

struct ArUco_Landmark;

/**
 * keypoints and associated landmarks
 */
struct ArUco_Feature: public Feature
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<ArUco_Feature> Ptr;

    // member data ////////////////////////////////////////////////////////////
    unsigned long aruco_id_; // ArUco marker ID
    std::vector<cv::Point2f> corner_keypoints_;
    SE3 T_cm_; // marker pose w.r.t camera
    std::weak_ptr<ArUco_Landmark> aruco_landmark_; // associated landmark

public:
    // constructor & destructor ///////////////////////////////////////////////
    ArUco_Feature() {}

    ArUco_Feature(std::shared_ptr<Frame> frame, const cv::KeyPoint& center_keypoint, 
        const long& aruco_id, const std::vector<cv::Point2f>& corner_keypoints, const SE3& T_cm)
        : Feature(frame, center_keypoint)
    {
        aruco_id_ = aruco_id;
        T_cm_ = T_cm;
        corner_keypoints_ = corner_keypoints;
    }

    // getter =================================================================
    unsigned long get_aruco_id()
    {
        return aruco_id_;
    }

    SE3 get_T_cm()
    {
        return T_cm_;
    }
};

} // namespace tello_slam

#endif // TELLOSLAM_ARUCOFEATURE_H
