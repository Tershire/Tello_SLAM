// system.cpp

/////////////////////////
// IONLAB ISAE-SUPAERO //
// TELLO SLAM PROJECT  //
/////////////////////////

// 2024 JAN 15
// Wonhee LEE

// reference:


#include "port/config.h"
#include "tool/tool_system.h"


namespace tello_slam
{

// public XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
// constructor & destructor ///////////////////////////////////////////////////
Tool_System::Tool_System(const std::string& configuration_file_path)
    : configuration_file_path_(configuration_file_path) {}

// member methods /////////////////////////////////////////////////////////////
bool Tool_System::initialize()
{
    // read from config file ==================================================
    if (Config::initialize(configuration_file_path_) == false)
    {
        return false;
    }

    // read configuration =====================================================
    std::string mode = Config::read<std::string>("mode");
    if (mode == "test")
        mode_ = TEST;
    else if (mode == "mission")
        mode_ = MISSION;
    else
        std::cout << "ERROR: Wrong Mode!" << std::endl;

    int verbose = Config::read<int>("verbose");
    if (verbose == 0)
        verbose_ = false;
    else
        verbose_ = true;

    // ArUco Detector ---------------------------------------------------------
    predifined_dictionary_name_ = Config::read<std::string>("predifined_dictionary_name");
    marker_length_ = Config::read<float>("marker_length");

    // port ===================================================================
    setting_ = std::make_shared<Setting>(Config::read<std::string>("setting_file_path"));

    // load camera ------------------------------------------------------------
    usb_camera_ = setting_->get_usb_camera();
    raspberry_camera_ = setting_->get_raspberry_camera();
    color_imager_ = setting_->get_color_imager();

    // set mono camera --------------------------------------------------------
    mono_camera_type_ = Config::read<std::string>("mono_camera_type");
    std::cout << "mono_camera_type_: " << mono_camera_type_ << std::endl;

    if (mono_camera_type_ == "usb")
        mono_camera_ = usb_camera_;
    else if (mono_camera_type_ == "raspberry")
        mono_camera_ = raspberry_camera_;
    else
        std::cout << "ERROR: no such mono camera type..." << std::endl;

    // rescale ----------------------------------------------------------------
    mono_camera_scale_factor_ = Config::read<float>("mono_camera_scale_factor");
    mono_camera_->rescale(mono_camera_scale_factor_);
    
    // create system components ===============================================
    int target_id = 0; // initial value
    
    // aruco detector ---------------------------------------------------------
    aruco_detector_ = std::make_shared<ArUco_Detector>(
        target_id, predifined_dictionary_name_, marker_length_, mono_camera_);
    aruco_detector_->set_verbose(verbose);
    if (mode_ == MISSION)
        aruco_detector_->set_input_mode(ArUco_Detector::Input_Mode::RASPBERRY);

    // IMU reader -------------------------------------------------------------
    imu_reader_ = std::make_shared<IMU_Reader>();

    // supervisor -------------------------------------------------------------
    supervisor_ = std::make_shared<Supervisor>();
    supervisor_->set_aruco_detector(aruco_detector_);
    supervisor_->set_imu_reader(imu_reader_);
    
    return true;
}

// member methods /////////////////////////////////////////////////////////////
// ----------------------------------------------------------------------------
void Tool_System::run()
{
    std::cout << "System: initiate" << std::endl;
    std::cout << "System: exit" << std::endl;
}

// ----------------------------------------------------------------------------
void Tool_System::run_roll_reader()
{
    std::cout << "System: initiate" << std::endl;

    // aruco_detector_->run();
    while (true)
    {

    }

    std::cout << "System: close" << std::endl;
    imu_reader_->close();
    supervisor_->close();

    std::cout << "System: exit" << std::endl;
}

} // namespace tello_slam
