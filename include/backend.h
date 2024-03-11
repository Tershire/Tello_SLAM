// backend.h

/////////////////////////
// IONLAB ISAE-SUPAERO //
// TELLO SLAM PROJECT  //
/////////////////////////

// 2024 MAR 10
// Wonhee LEE

// reference: slambook


#ifndef TELLOSLAM_BACKEND_H
#define TELLOSLAM_BACKEND_H

#include "common.h"
#include "frame.h"
#include "map.h"


namespace tello_slam
{

class Map;

/**
 * backend
*/
class Backend
{
// public:
//     EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
//     typedef std::shared_ptr<Backend> Ptr;

//     // constructor & destructor ///////////////////////////////////////////////
//     Backend();

//     // getter & setter ////////////////////////////////////////////////////////
//     // setter =================================================================
//     /**
//      * set camera 
//      */
//     void set_camera(Camera::Ptr camera)
//     {
//         camera_ = camera;
//     }

//     void set_map(std::shared_ptr<Map> map) {map_ = map;}

//     void set_optimizer_stop_flag(const bool& optimizer_stop_flag)
//     {
//         optimizer_stop_flag_ = optimizer_stop_flag;
//     }

//     // member methods /////////////////////////////////////////////////////////
//     /**
//      * trigger map update, start optimization 
//      */
//     void update_map();

//     /**
//      * close the backend thread 
//      */
//     void close();

// private:
//     // member data ////////////////////////////////////////////////////////////
//     std::shared_ptr<Map> map_;
//     std::thread backend_thread_;
//     std::mutex data_mutex_;

//     std::condition_variable do_map_update_;
//     std::atomic<bool> backend_running_;

//     Camera::Ptr camera_ = nullptr;

//     // Backend configuration ==================================================
//     int num_g2o_iterations_backend_ = 5;

//     // g2o ====================================================================
//     bool optimizer_stop_flag_ = false;
    
//     // member methods /////////////////////////////////////////////////////////
//     /**
//      * backend thread 
//      */
//     void thread_loop();

//     /**
//      * optimize for a given keyframe and waypoint 
//      */
//     void optimize(Map::map_points_type& map_points, Map::keyframes_type& keyframes);
};

} // namespace tello_slam

#endif // TELLOSLAM_BACKEND_H
