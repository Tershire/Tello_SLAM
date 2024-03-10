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

struct Frame;
struct Landmark;

/**
 * keypoints and associated landmarks
 * and the frame that holds them.
 */
struct Feature
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Feature> Ptr;

    // member data ////////////////////////////////////////////////////////////
    std::weak_ptr<Frame> frame_; // frame that holds the feature
    
    cv::KeyPoint keypoint_; // keypoint
    std::weak_ptr<Landmark> landmark_; // associated landmark

    bool is_outlier_ = false; // is it an outlier

public:
    // constructor & destructor ///////////////////////////////////////////////
    Feature() {}

    Feature(std::shared_ptr<Frame> frame, const cv::KeyPoint& keypoint);
};

} // namespace tello_slam

#endif // TELLOSLAM_FEATURE_H
