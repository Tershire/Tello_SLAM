// aruco_feature.cpp

/////////////////////////
// IONLAB ISAE-SUPAERO //
// TELLO SLAM PROJECT  //
/////////////////////////

// 2024 MAR 11
// Wonhee LEE

// reference:


#include "aruco_feature.h"


namespace tello_slam
{

// public XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
// constructor & destructor ///////////////////////////////////////////////////
ArUco_Feature::ArUco_Feature(std::shared_ptr<Frame> frame, const long& aruco_id, 
    const std::vector<cv::Point2f>& corner_keypoints, const SE3& T_cm):
    frame_(frame), aruco_id_(aruco_id), corner_keypoints_(corner_keypoints), T_cm_(T_cm) {}

} // namespace tello_slam
