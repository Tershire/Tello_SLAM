// feature.h

/////////////////////////
// IONLAB ISAE-SUPAERO //
// TELLO SLAM PROJECT  //
/////////////////////////

// 2024 MAR 09
// Wonhee LEE

// reference: slambook


#pragma once

#ifndef TELLOSLAM_FEATURE_H
#define TELLOSLAM_FEATURE_H

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
struct Feature
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Feature> Ptr;

    // member data ////////////////////////////////////////////////////////////
    std::weak_ptr<Frame> frame_; // frame that holds the feature
    cv::KeyPoint keypoint_; // keypoint
    std::weak_ptr<Map_Point> map_point_; // associated map point
    
    bool is_outlier_ = false; // is it an outlier
    // bool is_on_image_L_ = true;

public:
    // constructor & destructor ///////////////////////////////////////////////
    Feature() {}

    Feature(std::shared_ptr<Frame> frame, const cv::KeyPoint& keypoint):
        frame_(frame), keypoint_(keypoint) {}
};

} // namespace tello_slam

#endif // TELLOSLAM_FEATURE_H
