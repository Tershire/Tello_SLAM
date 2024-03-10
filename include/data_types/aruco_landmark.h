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

/**
 * representation of a ArUco landmark as a 6D pose.
 */
struct Aruco_Landmark : public Landmark
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Aruco_Landmark> Ptr;

    // member data ////////////////////////////////////////////////////////////
    unsigned long aruco_id_ = 0; // ArUco marker ID
    SE3 T_mw_; // marker pose w.r.t the world

    // std::mutex_T_mw_;
    
    // constructor & destructor ///////////////////////////////////////////////
    Aruco_Landmark() {}

    Aruco_Landmark(long id, Vec3 position):
        : Landmark(id, position)
    {
        aruco_id_ = aruco_id;
        T_mw_ = T_mw;
    }

    // getter & setter ////////////////////////////////////////////////////////
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

#endif // TELLOSLAM_ARUCOLANDMARK_H
