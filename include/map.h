// map.h

/////////////////////////
// IONLAB ISAE-SUPAERO //
// TELLO SLAM PROJECT  //
/////////////////////////

// 2024 MAR 10
// Wonhee LEE

// reference: slambook


#pragma once

#ifndef TELLOSLAM_MAP_H
#define TELLOSLAM_MAP_H

#include "common.h"
#include "frame.h"
#include "landmark.h"


namespace tello_slam
{

/**
 * frontend
 * backend
 */
class Map
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Map> Ptr;
    
    typedef std::unordered_map<unsigned long, Frame::Ptr> keyframes_type;
    typedef std::unordered_map<unsigned long, Landmark::Ptr> landmarks_type;

    // constructor & destructor ///////////////////////////////////////////////
    Map();

    // getter & setter ////////////////////////////////////////////////////////
    // getter =================================================================
    landmarks_type get_all_landmarks()
    {
        std::unique_lock<std::mutex> lock(data_mutex_);
        return landmarks_;
    }

    keyframes_type get_all_keyframes()
    {
        std::unique_lock<std::mutex> lock(data_mutex_);
        return keyframes_;
    }

    landmarks_type get_active_landmarks()
    {
        std::unique_lock<std::mutex> lock(data_mutex_);
        return active_landmarks_;
    }

    keyframes_type get_active_keyframes()
    {
        std::unique_lock<std::mutex> lock(data_mutex_);
        return active_keyframes_;
    }

    // member methods /////////////////////////////////////////////////////////
    /**
     * insert a keyframe 
     */
    void insert_keyframe(Frame::Ptr frame);

    /**
     * insert a landmark
     */
    void insert_landmark(Landmark::Ptr landmark);

    /**
     * clean up points in the map where the number of observations is zero
     */
    void clear();

private:
    // member data ////////////////////////////////////////////////////////////
    landmarks_type landmarks_; // all landmarks
    landmarks_type active_landmarks_; // active landmarks
    keyframes_type keyframes_; // all keyframes
    keyframes_type active_keyframes_; // active keyframes

    Frame::Ptr current_frame_ = nullptr;

    std::mutex data_mutex_;

    // settings
    unsigned int num_active_keyframes_ = 7; // number of active keyframes

    // member methods /////////////////////////////////////////////////////////
    /**
     * make old keyframes inactive 
     */
    void remove_old_keyframe();
};  

} // namespace tello_slam

#endif // TELLOSLAM_MAP_H
