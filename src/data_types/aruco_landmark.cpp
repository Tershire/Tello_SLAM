// aruco_landmark.cpp

/////////////////////////
// IONLAB ISAE-SUPAERO //
// TELLO SLAM PROJECT  //
/////////////////////////

// 2024 MAR 11
// Wonhee LEE

// reference: slambook


#include "aruco_landmark.h"
#include "aruco_feature.h"


namespace tello_slam
{

// public XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
// constructor & destructor ///////////////////////////////////////////////////
ArUco_Landmark::ArUco_Landmark(long id, long aruco_id, SE3 T_wm):
    id_(id), aruco_id_(aruco_id), T_wm_(T_wm) {}

// member methods /////////////////////////////////////////////////////////////
void ArUco_Landmark::add_observation(std::shared_ptr<ArUco_Feature> aruco_feature)
{
    //
    std::cout << "add_observation() [0]" << std::endl;

    std::unique_lock<std::mutex> lock(observations_mutex_);

    //
    std::cout << "add_observation() [1]" << std::endl;

    observations_.push_back(aruco_feature);

    //
    std::cout << "add_observation() [2]" << std::endl;

    num_observations_ += 1;
}

void ArUco_Landmark::remove_observation(std::shared_ptr<ArUco_Feature> aruco_feature)
{
    std::unique_lock<std::mutex> lock(observations_mutex_);
    for (auto iterator = observations_.begin(); iterator != observations_.end(); ++iterator)
    {
        if (iterator->lock() == aruco_feature)
        {
            // dissociate feature and landmark
            observations_.erase(iterator);
            aruco_feature->aruco_landmark_.reset();

            num_observations_ -= 1;
            break;
        }
    }
}

// static methods /////////////////////////////////////////////////////////////
ArUco_Landmark::Ptr ArUco_Landmark::create_aruco_landmark()
{
    static long factory_id = 0;

    ArUco_Landmark::Ptr aruco_landmark(new ArUco_Landmark);
    aruco_landmark->id_ = factory_id += 1;

    return aruco_landmark;
}

} // namespace tello_slam
