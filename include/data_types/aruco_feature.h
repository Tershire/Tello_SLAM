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

/**
 * keypoints and associated landmarks
 */
struct Aruco_Feature: public Feature
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Aruco_Feature> Ptr;

    // member data ////////////////////////////////////////////////////////////
    int aruco_id_; // ArUco marker ID
    SE3 T_mw_; // marker pose w.r.t the world

public:
    // constructor & destructor ///////////////////////////////////////////////
    Aruco_Feature() {}

    Aruco_Feature(std::shared_ptr<Frame> frame, const cv::KeyPoint& keypoint, const int& aruco_id, const SE3& T_mw)
        : Feature(frame, keypoint)
    {
        aruco_id_ = aruco_id;
        T_mw_ = T_mw;
    }

    // getter =================================================================
    Vec3 get_aruco_id()
    {
        return aruco_id_;
    }

    SE3 get_T_mw()
    {
        return T_mw_;
    }
};

} // namespace tello_slam

#endif // TELLOSLAM_ARUCOFEATURE_H