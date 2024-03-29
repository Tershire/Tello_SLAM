// common.h

/////////////////////////
// IONLAB ISAE-SUPAERO //
// TELLO SLAM PROJECT  //
/////////////////////////

// 2024 JAN 15
// Wonhee LEE

// reference:


#ifndef TELLOSLAM_COMMON_H
#define TELLOSLAM_COMMON_H


// std ////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <thread>

// OpenCV /////////////////////////////////////////////////////////////////////
#include <opencv2/opencv.hpp>

// Eigen //////////////////////////////////////////////////////////////////////
#include <Eigen/Core>
#include <Eigen/Geometry>

// typedefs ===================================================================
// double matricies
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatXX;
typedef Eigen::Matrix<double, 2, 2> Mat22;
typedef Eigen::Matrix<double, 2, 6> Mat26;
typedef Eigen::Matrix<double, 2, 7> Mat27;
typedef Eigen::Matrix<double, 3, 3> Mat33;
typedef Eigen::Matrix<double, 3, 4> Mat34;
typedef Eigen::Matrix<double, 4, 3> Mat43;
typedef Eigen::Matrix<double, 4, 4> Mat44;
typedef Eigen::Matrix<double, 6, 2> Mat62;
typedef Eigen::Matrix<double, 6, 6> Mat66;
typedef Eigen::Matrix<double, 7, 2> Mat72;
typedef Eigen::Matrix<double, 7, 7> Mat77;

// double vectors
typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VecX;
typedef Eigen::Matrix<double, 2, 1> Vec2;
typedef Eigen::Matrix<double, 3, 1> Vec3;
typedef Eigen::Matrix<double, 4, 1> Vec4;
typedef Eigen::Matrix<double, 6, 1> Vec6;
typedef Eigen::Matrix<double, 7, 1> Vec7;

// double quaternion
typedef Eigen::Quaterniond Quaternion;

// Sophus /////////////////////////////////////////////////////////////////////
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

// typedefs ===================================================================
typedef Sophus::SE3d SE3;
typedef Sophus::SO3d SO3;


#endif // TELLOSLAM_COMMON_H
