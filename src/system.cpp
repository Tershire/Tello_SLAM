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

    std::string input_mode = Config::read<std::string>("input_mode");
    if (input_mode == "tello")
        input_mode_ = Input_Mode::TELLO;
    else if (input_mode == "usb")
        input_mode_ = Input_Mode::USB;
    else if (input_mode == "video")
        input_mode_ = Input_Mode::VIDEO;
    else if (input_mode == "dataset")
        input_mode_ = Input_Mode::DATASET;
    else
        std::cout << "ERROR: input mode wrong\n";
    
    mono_camera_to_use_ = Config::read<std::string>("mono_camera_to_use");

    // port ===================================================================
    setting_ = std::make_shared<Setting>(Config::read<std::string>("setting_file_path"));

    switch (input_mode_)
    {
        case TELLO:
        case USB:
        case VIDEO:
        {
            data_stream_ = std::make_shared<Data_Stream>();
            break;
        }

        case DATASET:
            dataset_ = std::make_shared<Dataset>(Config::read<std::string>("dataset_directory_path"));
    }
    
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

    std::cout << "\t-set mono camera: " << mono_camera_to_use_ << std::endl;

    // create utility components ==============================================
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
    
    // create SLAM system components ==========================================
    frontend_ = Frontend::Ptr(new Frontend);
    backend_ = Backend::Ptr(new Backend);
    map_ = Map::Ptr(new Map);

    bool viewer_on = Config::read<int>("viewer_on");
    if (viewer_on)
    {
        viewer_ = Viewer::Ptr(new Viewer);
        viewer_->set_map(map_);
        viewer_->set_frontend(frontend_);
    }

    frontend_->set_backend(backend_);
    frontend_->set_map(map_);
    frontend_->set_viewer(viewer_);
    frontend_->set_camera(mono_camera_);
    frontend_->set_aruco_detector(aruco_detector_);

    // backend_->set_map(map_);
    // backend_->set_camera(mono_camera_);

    return true;
}

// member methods /////////////////////////////////////////////////////////////
void System::run()
{
    std::cout << "Tello SLAM: initiate" << std::endl;

    while (true)
    {
        if (step() == false)
        {
            break;
        }
    }

    // backend_->close(); // (TODO) enable when backend is completed
    if (viewer_) 
    {
        viewer_ ->close();
    }

    std::cout << "Tello SLAM: exit" << std::endl;
}

// ----------------------------------------------------------------------------
bool System::step()
{
    // load next frame
    // (TODO) integrate Data_Stream to Dataset to reduce switches.
    Frame::Ptr frame;
    switch (input_mode_)
    {
        case TELLO:
        case USB:
        case VIDEO:
        {
            frame = data_stream_->load_next_frame();

            //
            // std::time_t t = std::chrono::system_clock::to_time_t();
            std::cout << std::fixed << ">>> frame loaded with timestamp: " << frame->timestamp_ << std::endl;

            break;
        }

        case DATASET:
            frame = dataset_->load_next_frame();
    }

    if (frame == nullptr) return false;

    // timer ------------------------------------------------------------------
    // auto t1 = std::chrono::steady_clock::now();
    // ------------------------------------------------------------------------

    bool success = frontend_->step(frame);

    // timer ------------------------------------------------------------------
    // auto t2 = std::chrono::steady_clock::now();
    // auto time_used =
    //     std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    // LOG(INFO) << "VO cost time: " << time_used.count() << " seconds.";
    // ------------------------------------------------------------------------

    return success;
}

} // namespace tello_slam
