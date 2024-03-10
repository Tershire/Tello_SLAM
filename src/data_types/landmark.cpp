// landmark.cpp

/////////////////////////
// IONLAB ISAE-SUPAERO //
// TELLO SLAM PROJECT  //
/////////////////////////

// 2024 MAR 10
// Wonhee LEE

// reference: slambook


#include "landmark.h"
#include "feature.h"


namespace tello_slam
{

// public XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
// constructor & destructor ///////////////////////////////////////////////////
Landmark::Landmark(long id, Vec3 position): 
    id_(id), position_(position) {}

// member methods /////////////////////////////////////////////////////////////
void Landmark::add_observation(std::shared_ptr<Feature> feature)
    {
        std::unique_lock<std::mutex> lock(observations_mutex_);
        observations_.push_back(feature);
        num_observations_ += 1;
    }

void Landmark::remove_observation(std::shared_ptr<Feature> feature)
{
    std::unique_lock<std::mutex> lock(observations_mutex_);
    for (auto iterator = observations_.begin(); iterator != observations_.end(); ++iterator)
    {
        if (iterator->lock() == feature)
        {
            // dissociate feature and landmark
            observations_.erase(iterator);
            feature->landmark_.reset();

            num_observations_ -= 1;
            break;
        }
    }
}

// static methods /////////////////////////////////////////////////////////////
Landmark::Ptr Landmark::create_landmark()
{
    static long factory_id = 0;

    Landmark::Ptr landmark(new Landmark);
    landmark->id_ = factory_id += 1;

    return landmark;
}

} // namespace tello_slam
