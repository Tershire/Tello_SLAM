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
#include "aruco_landmark.h"


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
    typedef std::unordered_map<unsigned long, ArUco_Landmark::Ptr> aruco_landmarks_type;

    // constructor & destructor ///////////////////////////////////////////////
    Map();

    // getter & setter ////////////////////////////////////////////////////////
    // getter =================================================================
    keyframes_type get_all_keyframes()
    {
        std::unique_lock<std::mutex> lock(data_mutex_);
        return keyframes_;
    }

    keyframes_type get_active_keyframes()
    {
        std::unique_lock<std::mutex> lock(data_mutex_);
        return active_keyframes_;
    }

    landmarks_type get_all_landmarks()
    {
        std::unique_lock<std::mutex> lock(data_mutex_);
        return landmarks_;
    }

    landmarks_type get_active_landmarks()
    {
        std::unique_lock<std::mutex> lock(data_mutex_);
        return active_landmarks_;
    }

    aruco_landmarks_type get_all_aruco_landmarks()
    {
        std::unique_lock<std::mutex> lock(data_mutex_);
        return aruco_landmarks_;
    }

    aruco_landmarks_type get_active_aruco_landmarks()
    {
        std::unique_lock<std::mutex> lock(data_mutex_);
        return active_aruco_landmarks_;
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
     * insert a ArUco landmark
     */
    void insert_aruco_landmark(ArUco_Landmark::Ptr aruco_landmark);

    /**
     * clean up points in the map where the number of observations is zero
     */
    void clear();

private:
    // member data ////////////////////////////////////////////////////////////
    keyframes_type keyframes_; // all keyframes
    keyframes_type active_keyframes_; // active keyframes
    landmarks_type landmarks_; // all landmarks
    landmarks_type active_landmarks_; // active landmarks
    aruco_landmarks_type aruco_landmarks_; // all ArUco landmarks
    aruco_landmarks_type active_aruco_landmarks_; // active ArUco landmarks

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
