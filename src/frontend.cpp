// frontend.cpp

/////////////////////////
// IONLAB ISAE-SUPAERO //
// TELLO SLAM PROJECT  //
/////////////////////////

// 2024 MAR 10
// Wonhee LEE

// reference: slambook


#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <unistd.h>
#include <Sophus/sophus/common.hpp>
#include <Sophus/sophus/average.hpp>

#include "backend.h"
#include "config.h"
#include "feature.h"
#include "aruco_feature.h"
#include "frontend.h"
#include "map.h"
#include "viewer.h"


namespace tello_slam
{

// public XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
// constructor & destructor ///////////////////////////////////////////////////
Frontend::Frontend()
{   
    motion_log_on_ = Config::read<int>("motion_log_on");
}

// member_methods /////////////////////////////////////////////////////////////
bool Frontend::step(Frame::Ptr frame)
{
    current_frame_ = frame;

    switch (tracking_status_)
    {
        case Tracking_Status::NOT_INITIALIZED:
            initialize();
            break;
        case Tracking_Status::OK:
        case Tracking_Status::BAD:
            track();
            break;
        case Tracking_Status::LOST:
            track();
            // reset();
            break;
    }

    previous_frame_ = current_frame_;

    return true;
}

// private XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
// member_methods /////////////////////////////////////////////////////////////
bool Frontend::initialize()
{
    int num_detected_aruco_features = detect_aruco_features();

    if (num_detected_aruco_features < 1)
    {
        std::cout << "could not initialize frontend" << std::endl;
        return false;
    }

    bool initial_map_built = build_initial_map();
    if (initial_map_built)
    {
        tracking_status_ = Tracking_Status::OK;
        if (viewer_)
        {
            viewer_->set_current_frame(current_frame_);
            viewer_->request_update_map();
        }
        return true;
    }
    return false;
}

// ----------------------------------------------------------------------------
bool Frontend::build_initial_map()
{
    int num_aruco_landmarks = compute_aruco_poses();

    current_frame_->set_as_keyframe();
    map_->insert_keyframe(current_frame_);

    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // (TO DO) uncomment backend update for extending VO to SLAM
    // update backend because we have a new keyframe
    // backend_->update_map();
    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    std::cout << "initial map created with " << num_aruco_landmarks
        << " map points" << std::endl;

    // viewer -----------------------------------------------------------------
    if (viewer_) 
    {
        viewer_->set_current_frame(current_frame_);
        viewer_->request_update_map();
        // --------------------------------------------------------------------
        // viewer: step by step control
        if (viewer_->do_pause_)
        {
            std::cout << "paused: waiting..." << std::endl;
            while (viewer_->do_pause_)
            {
                usleep(500);

                if (viewer_->step_to_next_frame_)
                {
                    std::cout << "stepping to the next frame..." << std::endl;
                    break;
                }
            }
        }
        // --------------------------------------------------------------------
    }
    // ------------------------------------------------------------------------

    return true;
}

// ----------------------------------------------------------------------------
bool Frontend::reset()
{
    // NOT_IMPLEMENTED_YET
    std::cout << "////////// <!><!><!><!> TRACKING LOST <!><!><!><!> //////////" << std::endl;

    int num_detected_aruco_features = detect_aruco_features();
    if (num_detected_aruco_features < 1)
    {
        std::cout << "could not reset frontend" << std::endl;
        return false;
    }

    bool reset_map_built = build_reset_map();
    if (reset_map_built)
    {
        tracking_status_ = Tracking_Status::OK;
        if (viewer_)
        {
            viewer_->set_current_frame(current_frame_);
            viewer_->request_update_map();
        }
        return true;
    }
    return false;
}

// ----------------------------------------------------------------------------
bool Frontend::build_reset_map()
{
    return true;
}

// ----------------------------------------------------------------------------
bool Frontend::track()
{
    // constant velocity assumption
    // if (previous_frame_)
    // {
    //     current_frame_->set_T_cw(T_CurrPrev_ * previous_frame_->get_T_cw());
    // }

    // detect ArUco
    int num_detected_aruco_features = detect_aruco_features();
    if (num_detected_aruco_features < 1)
    {
        tracking_status_ = Tracking_Status::LOST;

        std::cout << "[Tracking Status]: LOST" << std::endl;
        return false;
    }

    // estimate | compute current camera pose
    int success = compute_camera_pose();
    if (!success)
    {
        tracking_status_ = Tracking_Status::LOST;
    }

    if (true) // (TODO) keyframe condition
    {
        insert_keyframe();
    }

    // compute pose difference
    T_CurrPrev_ = current_frame_->get_T_cw() * previous_frame_->get_T_cw().inverse();

    // deduce current instantaneous velocity
    if (motion_log_on_)
    {
        double delta_time = (current_frame_->timestamp_ - previous_frame_->timestamp_)*1E-3;
        SE3 T_PrevCurr = T_CurrPrev_.inverse();

        current_camera_translational_velocity_ = T_PrevCurr.translation() / delta_time;
        // std::cout << "current_camera_translational_velocity_: " << current_camera_translational_velocity_.transpose() << std::endl;

        // (TO DO) quaternion derivative
        // current_camera_angular_velocity_unit_quaternion_ = relative_pose_T_PrevCurr.unit_quaternion() / delta_time;
        // std::cout << "current_camera_angular_velocity_unit_quaternion_: " << current_camera_angular_velocity_unit_quaternion_ << std::endl;  
    }

    // viewer -----------------------------------------------------------------
    if (viewer_) 
    {
        viewer_->set_current_frame(current_frame_);

        // --------------------------------------------------------------------
        // viewer: step by step control
        if (viewer_->do_pause_)
        {
            std::cout << "paused: waiting..." << std::endl;
            while (viewer_->do_pause_)
            {
                usleep(500);

                if (viewer_->step_to_next_frame_)
                {
                    std::cout << "stepping to the next frame..." << std::endl;
                    break;
                }
            }
        }
        // --------------------------------------------------------------------
    }
    // ------------------------------------------------------------------------
    
    return true;
}

// ----------------------------------------------------------------------------
bool Frontend::insert_keyframe()
{
    // set current frame as keyframe
    current_frame_->set_as_keyframe();
    map_->insert_keyframe(current_frame_);

    std::cout << "set frame " << current_frame_->id_ << " as keyframe "
        << current_frame_->keyframe_id_ << std::endl;

    add_observation();

    compute_aruco_poses();

    // update backend because we have a new keyframe
    
    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // (TO DO) uncomment backend optimizer interrupt for extending VO to SLAM if needed
    // flag to stop backend optimization if it is still running
    // backend_->set_optimizer_stop_flag(true);
    // std::cout << "F->B: 'STOP OPTIMIZING!' XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX" << std::endl;
    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // (TO DO) uncomment backend update for extending VO to SLAM
    // backend_->update_map();
    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    if (viewer_) viewer_->request_update_map();
    
    return true;
}

// ----------------------------------------------------------------------------
void Frontend::add_observation()
{
    for (auto& feature : current_frame_->features_)
    {
        auto landmark = feature->landmark_.lock();
        if (landmark) landmark->add_observation(feature);
    }

    for (auto& aruco_feature : current_frame_->aruco_features_)
    {
        auto aruco_landmark = aruco_feature->aruco_landmark_.lock();
        if (aruco_landmark) aruco_landmark->add_observation(aruco_feature);
    }
}

// ----------------------------------------------------------------------------
int Frontend::detect_aruco_features()
{
    // detect
    std::vector<int> aruco_ids;
    std::vector<std::vector<cv::Point2f>> corner_keypointss;
    aruco_detector_->detect(current_frame_->image_, aruco_ids, corner_keypointss);

    // estimate poses
    std::vector<SE3> Ts_cm;
    aruco_detector_->estimate_poses(aruco_ids, corner_keypointss, Ts_cm);

    // register features to current frame
    int num_aruco_features_detected = 0;
    for (size_t i = 0; i < aruco_ids.size(); ++i)
    {
        ArUco_Feature::Ptr aruco_feature(new ArUco_Feature(current_frame_, 
            aruco_ids[i], corner_keypointss[i], Ts_cm[i]));
        
        current_frame_->aruco_features_.push_back(aruco_feature);

        num_aruco_features_detected += 1;
    }

    return num_aruco_features_detected;
}

// ----------------------------------------------------------------------------
int Frontend::compute_aruco_poses()
{
    SE3 T_wc = current_frame_->get_T_cw().inverse();
           
    int num_aruco_landmarks = 0;
    for (size_t i = 0; i < current_frame_->aruco_features_.size(); ++i)
    {
        SE3 T_cm = current_frame_->aruco_features_[i]->T_cm_;
        Vec3 p3D_camera = T_cm.translation();

        auto aruco_landmark = ArUco_Landmark::create_aruco_landmark();
        aruco_landmark->set_position(T_wc * p3D_camera);
        aruco_landmark->set_T_wm(T_wc * T_cm);
        aruco_landmark->add_observation(current_frame_->aruco_features_[i]);

        current_frame_->aruco_features_[i]->aruco_landmark_ = aruco_landmark;

        map_->insert_aruco_landmark(aruco_landmark);

        num_aruco_landmarks += 1;
    }

    return num_aruco_landmarks;
}

// ----------------------------------------------------------------------------
int Frontend::compute_camera_pose()
{
    Map::aruco_landmarks_type registered_aruco_landmarks = map_->get_all_aruco_landmarks();

    SE3 T_wm, T_cm;
    std::vector<SE3> Ts_cw;
    for (auto& aruco_feature : current_frame_->aruco_features_)
    {
        for (auto& registered_aruco_landmark : registered_aruco_landmarks)
        {
            if (aruco_feature->aruco_id_ == registered_aruco_landmark.first)
            {
                T_wm = registered_aruco_landmark.second->T_wm_;
                T_cm = aruco_feature->T_cm_;

                Ts_cw.push_back(T_cm * T_wm.inverse());
            }
        }
    }

    if (Ts_cw.size() > 0)
    {
        if (Ts_cw.size() > 1)
        {
            Sophus::enable_if_t<true, Sophus::optional<Sophus::SE3<double>>> mean_T_cw = Sophus::average(Ts_cw);
            current_frame_->set_T_cw(SE3(mean_T_cw->matrix()));
        }
        else
        {
            current_frame_->set_T_cw(Ts_cw[0]);
        }

        //
        SE3 T_cw = current_frame_->get_T_cw();
        std::cout << "camera position: " << T_cw.translation().transpose() << std::endl;
    }
    else
    {
        return -1;
    }

    return 0;
}

} // namespace tello_slam
