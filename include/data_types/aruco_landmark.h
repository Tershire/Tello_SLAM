// aruco_landmark.h

/////////////////////////
// IONLAB ISAE-SUPAERO //
// TELLO SLAM PROJECT  //
/////////////////////////

// 2024 MAR 10
// Wonhee LEE

// reference: slambook


#pragma once

#ifndef TELLOSLAM_ARUCOLANDMARK_H
#define TELLOSLAM_ARUCOLANDMARK_H

#include "common.h"


namespace tello_slam
{

struct ArUco_Feature;

/**
 * representation of a ArUco landmark as a 6D pose.
 */
struct ArUco_Landmark: public Landmark
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<ArUco_Landmark> Ptr;

    // member data ////////////////////////////////////////////////////////////
    unsigned long aruco_id_ = 0; // ArUco marker ID
    SE3 T_wm_; // marker pose

    std::mutex T_wm_mutex_;
    
    // constructor & destructor ///////////////////////////////////////////////
    ArUco_Landmark() {}

    ArUco_Landmark(long id, Vec3 position, long aruco_id, SE3 T_wm)
        : Landmark(id, position)
    {
        aruco_id_ = aruco_id;
        T_wm_ = T_wm;
    }

    // getter & setter ////////////////////////////////////////////////////////
    // getter =================================================================
    unsigned long get_aruco_id()
    {
        return aruco_id_;
    }

    SE3 get_T_wm()
    {
        return T_wm_;
    }

    // setter =================================================================
    void set_T_wm(const SE3& T_wm)
    {
        std::unique_lock<std::mutex> lock(T_wm_mutex_);
        T_wm_ = T_wm;
    }

    // static methods /////////////////////////////////////////////////////////
    // factory function =======================================================
    static ArUco_Landmark::Ptr create_aruco_landmark()
    {
        static long factory_id = 0;

        ArUco_Landmark::Ptr aruco_landmark(new ArUco_Landmark);
        aruco_landmark->id_ = factory_id += 1;

        return aruco_landmark;
    }
};

} // namespace tello_slam

#endif // TELLOSLAM_ARUCOLANDMARK_H
