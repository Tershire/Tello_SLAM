// system.h

/////////////////////////
// IONLAB ISAE-SUPAERO //
// TELLO SLAM PROJECT  //
/////////////////////////

// 2024 JAN 15
// Wonhee LEE

// reference:


#ifndef TELLOSLAM_SYSTEM_H
#define TELLOSLAM_SYSTEM_H

#include "common.h"
#include "frontend.h"
#include "backend.h"
#include "viewer.h"
#include "dataset.h"
#include "data_stream.h"
#include "port/setting.h"
#include "camera/camera.h"
#include "marker/aruco_detector.h"
#include "imu/imu_reader.h"
#include "supervisor.h"
#include "door/door_detector.h"
#include "extended_kalman_filter.h"


namespace tello_slam
{

/**
 * vision system
 */
class System
{
public:
    typedef std::shared_ptr<System> Ptr;

    // state member ///////////////////////////////////////////////////////////
    enum Input_Mode
    {
        TELLO,
        USB,
        VIDEO,
        DATASET
    };

    // member data ////////////////////////////////////////////////////////////
    Input_Mode input_mode_;
    bool verbose_;
    bool use_EKF_;

    // constructor & destructor ///////////////////////////////////////////////
    System(const std::string& configuration_file_path);

    // getter & setter ////////////////////////////////////////////////////////
    // getter =================================================================
    Frontend::Tracking_Status get_tracking_status() const 
    {
      return frontend_->get_tracking_status();
    }
    
    ArUco_Detector::Ptr get_aruco_detector() const {return aruco_detector_;}
    IMU_Reader::Ptr get_imu_reader() const {return imu_reader_;}
    Door_Detector::Ptr get_door_detector() const {return door_detector_;}

    // member methods /////////////////////////////////////////////////////////
    /**
     * initialize system
     * @return true if success
     */
    bool initialize();
    
    /**
     * run system in infinite loop
     */
    void run();

    /**
     * investigate roll output delay from camera and IMU
     */
    void run_roll_reader();

    /**
     * step forward in dataset
     */
    bool step();

private:
    // member data ////////////////////////////////////////////////////////////
    std::string configuration_file_path_;
    
    // port ===================================================================
    Setting::Ptr setting_ = nullptr;
    Dataset::Ptr dataset_ = nullptr;
    Data_Stream::Ptr data_stream_ = nullptr;

    // camera -----------------------------------------------------------------
    std::string mono_camera_to_use_;
    Camera::Ptr mono_camera_;

    float pre_resize_factor_;

    // system components ======================================================
    Frontend::Ptr frontend_ = nullptr;
    Backend::Ptr backend_ = nullptr;
    Map::Ptr map_ = nullptr;
    Viewer::Ptr viewer_ = nullptr;
    
    ArUco_Detector::Ptr aruco_detector_ = nullptr;
    EKF_Camera_Pose::Ptr ekf_camera_pose_ = nullptr;
    IMU_Reader::Ptr imu_reader_ = nullptr;
    Door_Detector::Ptr door_detector_ = nullptr;

    // ArUco Detector =========================================================
    std::string predifined_dictionary_name_;
    float marker_length_;
};

} // namespace tello_slam

#endif // TELLOSLAM_SYSTEM_H
