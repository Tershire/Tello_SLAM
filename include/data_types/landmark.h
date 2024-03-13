// landmark.h

/////////////////////////
// IONLAB ISAE-SUPAERO //
// TELLO SLAM PROJECT  //
/////////////////////////

// 2024 MAR 10
// Wonhee LEE

// reference: slambook


#pragma once

#ifndef TELLOSLAM_LANDMARK_H
#define TELLOSLAM_LANDMARK_H

#include "common.h"


namespace tello_slam
{

struct Frame;
struct Feature;

/**
 * representation of a landmark.
 */
struct Landmark
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Landmark> Ptr;

    // member data ////////////////////////////////////////////////////////////
    unsigned long id_ = 0; // landmark ID

    Vec3 position_ = Vec3::Zero(); // 3D position in the world
    
    std::list<std::weak_ptr<Feature>> observations_; // list of features corresponding to observations
    int num_observations_ = 0; // number of observations 
    
    bool is_outlier_ = false;

    std::mutex position_mutex_;
    std::mutex observations_mutex_;
    
    // constructor & destructor ///////////////////////////////////////////////
    Landmark() {}

    Landmark(long id, Vec3 position);

    // getter & setter ////////////////////////////////////////////////////////
    // getter =================================================================
    Vec3 get_position()
    {
        std::unique_lock<std::mutex> lock(position_mutex_);
        return position_;
    }

    std::list<std::weak_ptr<Feature>> get_observations()
    {
        std::unique_lock<std::mutex> lock(observations_mutex_);
        return observations_;
    }

    // setter =================================================================
    void set_position(const Vec3& position)
    {
        std::unique_lock<std::mutex> lock(position_mutex_);
        position_ = position;
    };

    // member methods /////////////////////////////////////////////////////////
    /**
     * 
     */
    void add_observation(std::shared_ptr<Feature> feature);

    /**
     * 
     */
    void remove_observation(std::shared_ptr<Feature> feature);

    // static methods /////////////////////////////////////////////////////////
    // factory function =======================================================
    static Landmark::Ptr create_landmark();
};

} // namespace tello_slam

#endif // TELLOSLAM_LANDMARK_H
