// system.cpp

/////////////////////////
// IONLAB ISAE-SUPAERO //
// TELLO SLAM PROJECT  //
/////////////////////////

// 2024 JAN 15
// Wonhee LEE

// reference:


#include "system.h"
#include "port/config.h"


namespace tello_slam
{

// public XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
// constructor & destructor ///////////////////////////////////////////////////
System::System(const std::string& configuration_file_path)
    : configuration_file_path_(configuration_file_path) {}

// member methods /////////////////////////////////////////////////////////////
bool System::initialize()
{
    // read from config file ==================================================
    if (Config::initialize(configuration_file_path_) == false)
    {
        return false;
    }

    // read configuration =====================================================
    int verbose = Config::read<int>("verbose");
    if (verbose == 0)
        verbose_ = false;
    else
        verbose_ = true;

    input_mode_ = Config::read<std::string>("input_mode");
    mono_camera_to_use_ = Config::read<std::string>("mono_camera_to_use");
    std::cout << "mono camera to use: " << mono_camera_to_use_ << std::endl;

    // port ===================================================================
    setting_ = std::make_shared<Setting>(Config::read<std::string>("setting_file_path"));
    setting_->set_mono_camera_to_use(mono_camera_to_use_);

    // get and set mono camera ------------------------------------------------
    if (mono_camera_to_use_ == "tello")
    {
        mono_camera_ = setting_->get_tello_camera();
    }
    else if (mono_camera_to_use_ == "usb")
    {
        mono_camera_ = setting_->get_usb_camera();
    }
    else
        std::cout << "ERROR: no such mono camera to use" << std::endl;

    // pre-rescale ------------------------------------------------------------
    pre_resize_factor_ = Config::read<float>("pre_resize_factor");
    mono_camera_->rescale(pre_resize_factor_);
    
    // create vision system components ========================================
    // ArUco Detector ---------------------------------------------------------
    predifined_dictionary_name_ = Config::read<std::string>("predifined_dictionary_name");
    marker_length_ = Config::read<float>("marker_length");

    int target_id = Config::read<int>("target_ID");
    aruco_detector_ = std::make_shared<ArUco_Detector>(
        target_id, predifined_dictionary_name_, marker_length_, mono_camera_);
    aruco_detector_->set_verbose(verbose);

    // door detector ----------------------------------------------------------
    door_detector_ = std::make_shared<Door_Detector>(mono_camera_);
    door_detector_->set_is_for_tracking_only_one(false);
    door_detector_->set_verbose(verbose);
    door_detector_->set_pre_resize_factor(pre_resize_factor_);
    
    return true;
}

// member methods /////////////////////////////////////////////////////////////
// ----------------------------------------------------------------------------
void System::run()
{
    std::cout << "System: initiate" << std::endl;
    std::cout << "System: exit" << std::endl;
}

} // namespace tello_slam
