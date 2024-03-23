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
#include "extended_kalman_filter.h"
#include "conversion_toolbox.h"


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

    // track ArUco
    std::vector<unsigned int> tracked_aruco_ids = track_aruco_features();
    if (tracked_aruco_ids.size() < 1)
    {
        tracking_status_ = Tracking_Status::LOST;

        std::cout << "[Tracking Status]: LOST" << std::endl;
        return false;
    }

    // estimate | compute current camera pose
    int success = estimate_camera_pose();
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
    SE3 T_PrevCurr = T_CurrPrev_.inverse();
    current_frame_->T_PrevCurr_ = T_PrevCurr;

    //
    // std::cout << "T_CurrPrev_: \n" << T_CurrPrev_.matrix() << std::endl;
    Vec7 delta_x = T_to_t_q(T_CurrPrev_);
    std::cout << "wc delta_x: " << delta_x.transpose() << std::endl;

    //
    // for (auto& active_aruco_landmark : map_->get_active_aruco_landmarks())
    // {
    //     std::cout << "#obsv.s [ID " << active_aruco_landmark.first << "]: " << active_aruco_landmark.second->num_observations_ << std::endl;
    // }

    // deduce current instantaneous velocity
    if (motion_log_on_)
    {
        current_camera_position_ = current_frame_->get_T_cw().inverse().translation();

        double delta_time = (current_frame_->timestamp_ - previous_frame_->timestamp_)*1E-3;

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

    current_frame_->aruco_ids_ = aruco_ids;
    current_frame_->corner_keypointss_ = corner_keypointss;

    // estimate poses
    std::vector<SE3> Ts_cm;
    aruco_detector_->estimate_poses(aruco_ids, corner_keypointss, Ts_cm);

    // register features to current frame
    int num_detected_aruco_features = 0;
    for (size_t i = 0; i < aruco_ids.size(); ++i)
    {
        ArUco_Feature::Ptr aruco_feature(new ArUco_Feature(current_frame_, 
            aruco_ids[i], corner_keypointss[i], Ts_cm[i]));
        
        current_frame_->aruco_features_.push_back(aruco_feature);

        num_detected_aruco_features += 1;
    }

    return num_detected_aruco_features;
}

// ----------------------------------------------------------------------------
std::vector<unsigned int> Frontend::track_aruco_features()
{
    detect_aruco_features();

    std::vector<unsigned int> tracked_aruco_ids;
    Map::aruco_landmarks_type registered_aruco_landmarks = map_->get_all_aruco_landmarks();
    SE3 T_cm;
    for (auto& aruco_feature : current_frame_->aruco_features_)
    {
        if (registered_aruco_landmarks.find(aruco_feature->aruco_id_) != registered_aruco_landmarks.end())
        {
            // check pose stability
            bool is_pose_stable = determine_pose_stability(aruco_feature->T_cm_, 
                registered_aruco_landmarks[aruco_feature->aruco_id_]->observations_.back().lock()->T_cm_);

            if (is_pose_stable)
            {
                // set the landmark of the detected feature as the corresponding registered aruco landmark 
                aruco_feature->aruco_landmark_ = registered_aruco_landmarks[aruco_feature->aruco_id_];

                tracked_aruco_ids.push_back(aruco_feature->aruco_id_);
            }
        }
    }

    return tracked_aruco_ids;
}

// ----------------------------------------------------------------------------
int Frontend::compute_aruco_poses()
{
    SE3 T_wc = current_frame_->get_T_cw().inverse();
           
    int num_aruco_landmarks = 0;
    for (auto& aruco_feature : current_frame_->aruco_features_)
    {
        ArUco_Landmark::Ptr aruco_landmark;
        if (aruco_feature->aruco_landmark_.lock())
        {
            aruco_landmark = aruco_feature->aruco_landmark_.lock();
        }
        else
        {
            aruco_landmark = ArUco_Landmark::create_aruco_landmark();
        }

        SE3 T_cm = aruco_feature->T_cm_;
        Vec3 p3D_camera = T_cm.translation();

        aruco_landmark->set_aruco_id(aruco_feature->aruco_id_);
        aruco_landmark->set_position(T_wc * p3D_camera);
        aruco_landmark->set_T_wm(T_wc * T_cm);

        aruco_landmark->add_observation(aruco_feature);

        aruco_feature->aruco_landmark_ = aruco_landmark;

        map_->insert_aruco_landmark(aruco_landmark);

        //
        // std::cout << "\t-inserted an aruco landmark to the map." << std::endl;
        
        num_aruco_landmarks += 1;
    }

    //
    std::cout << "\tnumber of active aruco landmarks: " << map_->get_active_aruco_landmarks().size() << std::endl;

    return num_aruco_landmarks;
}

// ----------------------------------------------------------------------------
int Frontend::estimate_camera_pose()
{
    Map::aruco_landmarks_type registered_aruco_landmarks = map_->get_all_aruco_landmarks();

    SE3 T_wm, T_cm, T_cw;
    std::vector<SE3> Ts_cw;
    for (auto& aruco_feature : current_frame_->aruco_features_)
    {
        // if a newly detected marker is already registered to the map,
        // deduce the current camera pose and collect these poses for all detected markers.
        if (registered_aruco_landmarks.find(aruco_feature->aruco_id_) != registered_aruco_landmarks.end())
        {
            ArUco_Landmark::Ptr registered_aruco_landmark = registered_aruco_landmarks[aruco_feature->aruco_id_];

            T_wm = registered_aruco_landmark->T_wm_;
            T_cm = aruco_feature->T_cm_;
            
            // ----------------------------------------------------------------
            // EKF: based on the translation difference, raise question on the T_cm observation.

            if (use_EKF_)
            {
                //
                std::cout << "ArUco ID: " << aruco_feature->aruco_id_ << std::endl;

                // calculate pose difference
                SE3 previous_T_cm = previous_frame_->T_cw_ * T_wm;
                SE3 T_CurrPrev = T_cm * previous_T_cm.inverse();
                SE3 T_PrevCurr = T_CurrPrev.inverse();
                double delta_time = (current_frame_->timestamp_ - previous_frame_->timestamp_)*1E-3;
                double v = T_PrevCurr.translation().norm() / delta_time;

                // prepare inputs to EKF
                std::pair<Mat77, Mat22> Q_and_R = propose_Q_and_R(v);

                //
                std::cout << "R:\n" << std::get<1>(Q_and_R) << std::endl;

                Vec7 x_post_prev = T_to_t_q(previous_T_cm);
                EKF_Camera_Pose::state_distribution state_distribution_post_prev = 
                    std::make_pair(x_post_prev, Mat77::Identity());

                Vec7 delta_x = T_to_t_q(previous_frame_->T_PrevCurr_);

                //
                std::cout << "cm delta_x: " << delta_x.transpose() << std::endl;
                
                std::vector<int> aruco_ids = current_frame_->aruco_ids_;
                auto iterator = std::find(aruco_ids.begin(), aruco_ids.end(), aruco_feature->aruco_id_);
                int index = iterator - aruco_ids.begin();
                std::vector<cv::Point2f> corner_keypoints = current_frame_->corner_keypointss_[index];
                cv::Point2f keypoints_sum;
                for (auto& corner_keypoint : corner_keypoints)
                {
                    keypoints_sum += corner_keypoint;
                }
                Vec2 z_meas = Vec2(keypoints_sum.x, keypoints_sum.y) / corner_keypoints.size();

                //
                // std::cout << "aruco_feature->aruco_id_: " << aruco_feature->aruco_id_ << std::endl;
                // cv::Mat image_out;
                // cv::cvtColor(current_frame_->image_, image_out, cv::COLOR_GRAY2BGR);
                // cv::circle(image_out, cv::Point2f(z_meas[0], z_meas[1]), 3, cv::Scalar(0, 255, 0));
                // cv::imshow("EKF Projection", image_out);
                // cv::waitKey(10);

                // run EKF
                EKF_Camera_Pose::state_distribution state_distribution = 
                    ekf_camera_pose_->estimate(state_distribution_post_prev, delta_x, z_meas, std::get<0>(Q_and_R), std::get<1>(Q_and_R));

                Vec7 x = std::get<0>(state_distribution);

                //
                std::cout << "post t_cm [cm]: " << x.head(3).transpose()*1E2 << std::endl;

                T_cm = t_q_to_T(x);
                // Mat77 P = std::get<1>(state_distribution);
            }
            // ----------------------------------------------------------------

            T_cw = T_cm * T_wm.inverse();

            Ts_cw.push_back(T_cw);
        }
    }

    if (Ts_cw.size() > 0)
    {
        if (Ts_cw.size() > 1)
        {
            // if the collection has at least one entry, take the average and set it as the current camera pose.
            Sophus::enable_if_t<true, Sophus::optional<Sophus::SE3<double>>> mean_T_cw = Sophus::average(Ts_cw);
            current_frame_->set_T_cw(SE3(mean_T_cw->matrix()));
        }
        else
        {
            current_frame_->set_T_cw(Ts_cw[0]);
        }

        //
        Vec3 p3D_world = current_frame_->get_T_cw().inverse().translation();
        std::printf("\tcamera position [cm]: (%7.1f, %7.1f, %7.1f)", p3D_world[0]*1E2, p3D_world[1]*1E2, p3D_world[2]*1E2);
        std::cout << std::endl;
    }
    else
    {
        return -1;
    }

    return 0;
}

// ----------------------------------------------------------------------------
std::pair<Mat77, Mat22> Frontend::propose_Q_and_R(const double& v)
{
    double mu = 0; // (TODO) can we say this assumption is ok ?
    double sigma = 1.5;

    double p = 1/(sigma*std::sqrt(2*M_PI))*std::exp(-0.5*((v - mu)/sigma)*((v - mu)/sigma));

    Mat77 Q = Mat77::Identity();
    Mat22 R = Mat22::Identity()*10*1/(10*(std::tanh(3.6*p) + 0.1));

    return std::make_pair(Q, R);
}

} // namespace tello_slam
