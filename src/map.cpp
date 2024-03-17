// map.cpp

/////////////////////////
// IONLAB ISAE-SUPAERO //
// TELLO SLAM PROJECT  //
/////////////////////////

// 2024 MAR 10
// Wonhee LEE

// reference: slambook


#include "map.h"
#include "feature.h"
#include "aruco_feature.h"
#include "config.h"


namespace tello_slam
{

// public XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
// constructor & destructor ///////////////////////////////////////////////////
Map::Map()
{
    num_active_keyframes_ = Config::read<int>("num_active_keyframes");
}

// member methods /////////////////////////////////////////////////////////////
void Map::insert_keyframe(Frame::Ptr frame)
{
    current_frame_ = frame;
    if (keyframes_.find(frame->keyframe_id_) == keyframes_.end()) // if ID not found
    {
        // insert 
        keyframes_.insert(make_pair(frame->keyframe_id_, frame));
        active_keyframes_.insert(make_pair(frame->keyframe_id_, frame));
    }
    else // if ID exists, replace (TO DO) when such kind of case can happen ?
    {
        keyframes_[frame->keyframe_id_] = frame;
        active_keyframes_[frame->keyframe_id_] = frame;
    }

    // if sliding window full, remove by given criteria
    if (active_keyframes_.size() > num_active_keyframes_)
    {
        remove_old_keyframe();
    }
}

// ----------------------------------------------------------------------------
void Map::insert_landmark(Landmark::Ptr landmark)
{
    if (landmarks_.find(landmark->id_) == landmarks_.end())
    {
        landmarks_.insert(make_pair(landmark->id_, landmark));
        active_landmarks_.insert(make_pair(landmark->id_, landmark));
    }
    else
    {
        landmarks_[landmark->id_] = landmark;
        active_landmarks_[landmark->id_] = landmark;
    }
}

// ----------------------------------------------------------------------------
void Map::insert_aruco_landmark(ArUco_Landmark::Ptr aruco_landmark)
{
    if (aruco_landmarks_.find(aruco_landmark->aruco_id_) == aruco_landmarks_.end())
    {
        aruco_landmarks_.insert(make_pair(aruco_landmark->aruco_id_, aruco_landmark));
        active_aruco_landmarks_.insert(make_pair(aruco_landmark->aruco_id_, aruco_landmark));
    }
    else
    {
        aruco_landmarks_[aruco_landmark->aruco_id_] = aruco_landmark;
        active_aruco_landmarks_[aruco_landmark->aruco_id_] = aruco_landmark;
    }
}

// ----------------------------------------------------------------------------
void Map::remove_old_keyframe()
{
    if (current_frame_ == nullptr)
    {
        return;
    }

    // find the farthest and closest keyframes from the current frame
    double max_displacement = 0, min_displacement = 9999;
    double farthest_keyframe_id = 0, closest_keyframe_id = 0;
    auto T_wc = current_frame_->get_T_cw().inverse();
    for (auto& active_keyframe : active_keyframes_)
    {
        if (active_keyframe.second == current_frame_) 
        {
            continue;
        }

        auto displacement = (active_keyframe.second->get_T_cw() * T_wc).log().norm();
        if (displacement > max_displacement)
        {
            max_displacement = displacement;
            farthest_keyframe_id = active_keyframe.first;
        }

        if (displacement < min_displacement)
        {
            min_displacement = displacement;
            closest_keyframe_id = active_keyframe.first;
        }
    }

    const double min_displacement_threshold = 0.2;
    Frame::Ptr frame_to_remove = nullptr;
    if (min_displacement < min_displacement_threshold)
    {
        // if there are very close frames, delete the most recent first
        frame_to_remove = keyframes_.at(closest_keyframe_id);
    }
    else
    {
        // delete the farthest
        frame_to_remove = keyframes_.at(farthest_keyframe_id);
    }

    // remove keyframe and landmark observations
    active_keyframes_.erase(frame_to_remove->keyframe_id_);
    for (auto feature : frame_to_remove->features_)
    {
        auto landmark = feature->landmark_.lock();
        if (landmark)
        {
            landmark->remove_observation(feature);
        }
    }

    clear();
}

// ----------------------------------------------------------------------------
void Map::clear()
{
    int num_landmarks_removed = 0;
    for (auto iterator = active_landmarks_.begin(); iterator != active_landmarks_.end();)
    {
        if (iterator->second->num_observations_ == 0)
        {
            iterator = active_landmarks_.erase(iterator);
            num_landmarks_removed += 1;
        }
        else
        {
            ++iterator;
        }
    }

    // std::cout << "removed " << num_landmarks_removed << " active landmarks" << std::endl;
}

} // namespace tello_slam
