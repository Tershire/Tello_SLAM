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

// struct Frame;
// struct Map_Point;

/**
 * keypoints and associated map points
 */
struct Aruco_Feature: public Feature
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Aruco_Feature> Ptr;

    // member data ////////////////////////////////////////////////////////////
    int id_; // marker ID
    // maybe axes

public:
    // constructor & destructor ///////////////////////////////////////////////
    Aruco_Feature() {}

    Aruco_Feature(std::shared_ptr<Frame> frame, const cv::KeyPoint& keypoint, const int id)
        : Feature(frame, keypoint)
    {
        id_ = id;
    }
};

} // namespace tello_slam

#endif // TELLOSLAM_ARUCOFEATURE_H
