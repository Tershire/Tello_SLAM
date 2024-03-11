// data_stream.cpp

/////////////////////////
// IONLAB ISAE-SUPAERO //
// TELLO SLAM PROJECT  //
/////////////////////////

// 2024 MAR 11
// Wonhee LEE

// reference: slambook


#include <fstream>
#include <opencv2/opencv.hpp>

#include "data_stream.h"
#include "frame.h"
#include "config.h"


namespace tello_slam
{

// public XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
// constructor & destructor ///////////////////////////////////////////////////
Data_Stream::Data_Stream()
{
    std::string input_mode = Config::read<std::string>("input_mode");
    if (input_mode == "tello")
        input_mode_ = Input_Mode::TELLO;
    else if (input_mode == "usb")
        input_mode_ = Input_Mode::USB;
    else if (input_mode == "video")
        input_mode_ = Input_Mode::VIDEO;
    else
        std::cout << "ERROR: input mode wrong\n";
    
    switch (input_mode_)
    {
        case TELLO:
            cap_ = cv::VideoCapture(Config::read<std::string>("tello_video_stream"), cv::CAP_FFMPEG);
            break;

        case USB:
            cap_ = cv::VideoCapture(Config::read<int>("USB_camera_ID"));
            break;

        case VIDEO:
            cap_ = cv::VideoCapture(Config::read<std::string>("video_file_path"));
    }

    // check capture
    if (!cap_.isOpened()) 
    {
        std::cerr << "ERROR: capturer is not open\n";
    }

    // get FPS
    double fps = cap_.get(cv::CAP_PROP_FPS);
    std::cout << "FPS: " << fps << std::endl;

    // resize =================================================================
    pre_resize_factor_ = Config::read<float>("pre_resize_factor"); 

    // pre-processing =========================================================
    clahe_on_ = Config::read<int>("clahe_on");
    if (clahe_on_)
    {
        clahe_ = cv::createCLAHE(3.0, cv::Size(8, 8));
    }

    // load UNIX timestamps ===================================================
    // to estimate motion data (ex. velocity)
    motion_log_on_ = Config::read<int>("motion_log_on");
}

// ----------------------------------------------------------------------------
Frame::Ptr Data_Stream::load_next_frame()
{
    cv::Mat image;

    cap_ >> image;

    // check frame
    if (image.empty()) 
    {
        std::cerr << "ERROR: blank image\n";
    }

    // get timestamp
    auto now = std::chrono::system_clock::now();
    auto t = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch());
    long timestamp = t.count();

    // CLAHE --------------------------------------------------------------
    // probably slower than reading as grayscale
    // image = cv::imread(file_paths_[current_image_index_], cv::IMREAD_COLOR);

    // cv::cvtColor(image, image, cv::COLOR_BGR2Lab);
    // --------------------------------------------------------------------

    // pre-resizing -----------------------------------------------------------
    cv::resize(image, image, cv::Size(), pre_resize_factor_, pre_resize_factor_, cv::INTER_NEAREST);    
    // ------------------------------------------------------------------------

    // pre-processing: CLAHE ==================================================
    if (clahe_on_)
    {   
        // // Extract L channel from LAB
        // cv::extractChannel(image, image, 0);

        // apply CLAHE
        clahe_->apply(image, image);
    }
    // ========================================================================
    
    auto frame = Frame::create_frame();

    frame->image_ = image;

    current_image_index_ += 1;

    if (motion_log_on_) 
    {
        frame->timestamp_ = timestamp;
    }

    return frame;
}

} // namespace tello_slam
