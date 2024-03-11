// camera.cpp

/////////////////////////
// IONLAB ISAE-SUPAERO //
// TELLO SLAM PROJECT  //
/////////////////////////

// 2024 JAN 15
// Wonhee LEE

// reference:


#include <iostream>

#include "camera/camera.h"


namespace tello_slam
{

// public XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
// constructor & destructor ///////////////////////////////////////////////////
Camera::Camera(const std::vector<cv::Mat>& intrinsic_parameters, const SE3& T_cNc)
    : intrinsic_parameters_(intrinsic_parameters), T_cNc_(T_cNc)
{
    cameraMatrix_ = intrinsic_parameters.at(0);
    
    fx_ = cameraMatrix_.at<double>(0, 0);
    fy_ = cameraMatrix_.at<double>(1, 1);
    cx_ = cameraMatrix_.at<double>(0, 2);
    cy_ = cameraMatrix_.at<double>(1, 2); 

    T_cNc_inv_ = T_cNc_.inverse();
}

///////////////////////////////////////////////////////////////////////////////
void Camera::rescale(const float& scale_factor)
{
    cameraMatrix_ *= scale_factor;

    std::cout << "\ncameraMatrix_: " << cameraMatrix_ << std::endl;

    fx_ = cameraMatrix_.at<double>(0, 0);
    fy_ = cameraMatrix_.at<double>(1, 1);
    cx_ = cameraMatrix_.at<double>(0, 2);
    cy_ = cameraMatrix_.at<double>(1, 2);

    intrinsic_parameters_.at(0) = cameraMatrix_;
}

} // namespace tello_slam