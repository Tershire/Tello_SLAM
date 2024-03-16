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

struct Frame;
struct ArUco_Feature;

/**
 * representation of a ArUco landmark as a 6D pose.
 */
struct ArUco_Landmark
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<ArUco_Landmark> Ptr;

    // member data ////////////////////////////////////////////////////////////
    unsigned long id_ = 0; // landmark ID
    
    unsigned long aruco_id_ = 0; // ArUco marker ID
    SE3 T_wm_; // marker pose

    Vec3 position_ = Vec3::Zero(); // 3D position in the world

    std::list<std::weak_ptr<ArUco_Feature>> observations_; // list of features corresponding to observations
    int num_observations_ = 0; // number of observations

    std::mutex T_wm_mutex_;
    std::mutex position_mutex_;
    std::mutex observations_mutex_;
    
    // constructor & destructor ///////////////////////////////////////////////
    ArUco_Landmark() {}

    ArUco_Landmark(long id, long aruco_id, SE3 T_wm);

    // getter & setter ////////////////////////////////////////////////////////
    // getter =================================================================
    unsigned long get_aruco_id()
    {
        return aruco_id_;
    }

    SE3 get_T_wm()
    {
        std::unique_lock<std::mutex> lock(T_wm_mutex_);
        return T_wm_;
    }

    Vec3 get_position()
    {
        std::unique_lock<std::mutex> lock(position_mutex_);
        return position_;
    }

    std::list<std::weak_ptr<ArUco_Feature>> get_observations()
    {
        std::unique_lock<std::mutex> lock(observations_mutex_);
        return observations_;
    }

    // setter =================================================================
    void set_aruco_id(const unsigned long& aruco_id)
    {
        aruco_id_ = aruco_id;
    }

    void set_T_wm(const SE3& T_wm)
    {
        std::unique_lock<std::mutex> lock(T_wm_mutex_);
        T_wm_ = T_wm;
    }

    void set_position(const Vec3& position)
    {
        std::unique_lock<std::mutex> lock(position_mutex_);
        position_ = position;
    }

    // member methods /////////////////////////////////////////////////////////////
    void add_observation(std::shared_ptr<ArUco_Feature> aruco_feature);

    /**
     * 
     */
    void remove_observation(std::shared_ptr<ArUco_Feature> aruco_feature);

    // static methods /////////////////////////////////////////////////////////
    // factory function =======================================================
    static ArUco_Landmark::Ptr create_aruco_landmark();
};

} // namespace tello_slam

#endif // TELLOSLAM_ARUCOLANDMARK_H
